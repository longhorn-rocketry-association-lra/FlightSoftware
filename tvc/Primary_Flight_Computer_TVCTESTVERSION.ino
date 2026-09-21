#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <utility/quaternion.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo_pitch;
Servo servo_yaw;

#define PARACHUTE_PIN 7
const float DEG2RAD = 0.01745329251;

enum FlightPhase { IDLE, BOOST, COAST, RECOVERY };

struct State {
  float vz;
  float p_err, y_err; // Calculated from quaternion
};

State state;
FlightPhase flight_phase = BOOST;

unsigned long last_time;
int descent_counter = 0;
int arm_counter = 0;
int weak_counter = 0;
bool is_armed = true; //MAKE FALSE FOR REAL FLIGHT
bool deployed = false;

// ==== USER CONFIG ====
int PITCH_DIR = 1;
int YAW_DIR   = 1;

// PID Gains
float Kp = 0.7;
float Kd = 0.00;

void setup() {
  Serial.begin(115200);
  Serial.print("Setup Success");

  if (!bno.begin()) while (1);
  bno.setExtCrystalUse(true);

  servo_pitch.attach(5);
  servo_yaw.attach(4);

  pinMode(PARACHUTE_PIN, OUTPUT);
  digitalWrite(PARACHUTE_PIN, LOW);

  center_servos();
  last_time = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) * 1e-6;

  if (dt < 0.01) return;     // 100 Hz
  if (dt > 0.1) {           // FAILSAFE
    center_servos();
    return;
  }
  last_time = now;

  // ==== SENSOR READ ====
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Check for sensor failure
  if (isnan(quat.w()) || isnan(quat.x()) || isnan(quat.y()) || isnan(quat.z()) || isnan(acc.z())) {
    center_servos();
    return;
  }

  // ==== QUATERNION CONTROL LOGIC ====
  // We extract the "Gravity Vector" from the quaternion. 
  // This represents where the rocket thinks 'Down' is relative to its own body.
  // Full gravity vector in body frame
  // float grav_x = 2 * (quat.x() * quat.z() - quat.w() * quat.y());
  float grav_y = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
  float grav_z = quat.w()*quat.w() - quat.x()*quat.x()
             - quat.y()*quat.y() + quat.z()*quat.z();

  // When board is UPRIGHT (rocket axis = board's Y axis):
  state.p_err = grav_z;  // was grav_y
  state.y_err = grav_y;  // unchanged


  // ==== VELOCITY & STATE MACHINE ====
  // Rotate body-frame accel into world frame using quaternion
  float ax = acc.x(), ay = acc.y(), az = acc.z();

  float world_az = 2*(quat.x()*quat.z() - quat.w()*quat.y()) * ax
              + 2*(quat.w()*quat.x() + quat.y()*quat.z()) * ay
              + (quat.w()*quat.w() - quat.x()*quat.x() 
              - quat.y()*quat.y() + quat.z()*quat.z()) * az;

  state.vz += world_az * dt;
  state.vz *= 0.9999;

  update_state_machine(world_az);
  Serial.print("p:"); Serial.print(state.p_err);
  Serial.print(" y:"); Serial.print(state.y_err);
  Serial.print(" vz:"); Serial.print(state.vz);
  Serial.print(" az:"); Serial.println(world_az);

  // ==== CONTROL LOOP ====
  if (is_armed && (flight_phase == BOOST)) {
    
    // Safety: If tilted more than ~45 degrees (0.7 in vector magnitude)
    if (abs(state.p_err) > 0.7 || abs(state.y_err) > 0.7) {
      center_servos();
      return;
    }

    // 1. Calculate Error in Radians
    float pitch_error_rad = asin(constrain(state.p_err, -1.0, 1.0));
    float yaw_error_rad   = asin(constrain(state.y_err, -1.0, 1.0));
    //Serial.print(pitch_error_rad);
    Serial.print(", ");
    //Serial.println(yaw_error_rad);

    // PD Control: (Proportional * Vector Error) - (Derivative * Angular Velocity)
    float cmd_p = (Kp * pitch_error_rad) - (Kd * gyro.y());
    float cmd_y = (Kp * yaw_error_rad) - (Kd * gyro.z());

    cmd_p *= PITCH_DIR;
    cmd_y *= YAW_DIR;

    // Convert to Microseconds (Assumes +/- 250us throw)
    int p_out = constrain(1500 + (cmd_p * 100), 1400, 1650);
    int y_out = constrain(1500 + (cmd_y * 100), 1400, 1650);
    //top servo (1400<microsec<1700)
    //bottom servo (1400<microsec<1650)
    Serial.println(p_out);
    Serial.println(y_out);
    Serial.print(", ");
    servo_pitch.writeMicroseconds(p_out);
    servo_yaw.writeMicroseconds(y_out);

  } else {
    center_servos();
  }
}

void update_state_machine(float vert_accel) {
  switch (flight_phase) {
    case IDLE:
      if (vert_accel > 15.0) {
        arm_counter++;
        if (arm_counter > 5) {    
          state.vz = 0;    
          is_armed = true;
          flight_phase = IDLE;
        }
      } else { arm_counter = 0; }
      break;

    case BOOST:
      if (vert_accel < 2.0) { 
        weak_counter++;
        if (weak_counter > 5) {
          flight_phase = BOOST;
        }
      } else { weak_counter = 0; }
      break;

    case COAST:
      if (state.vz < -2.0) {
        descent_counter++;
        if (descent_counter > 15) flight_phase = RECOVERY;
      } else { descent_counter = 0; }
      break;

    case RECOVERY:
      if (!deployed) {
        digitalWrite(PARACHUTE_PIN, HIGH);
        delay(500); 
        digitalWrite(PARACHUTE_PIN, LOW);
        deployed = true;
      }
      break;
  }
}

void center_servos() {
  servo_pitch.write(1500);
  servo_yaw.writeMicroseconds(1500);
}