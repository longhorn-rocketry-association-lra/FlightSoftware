#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
//NOTE: THE CS SD DATA PIN IS 15
//.CSV FILE WILL SAVE IN THE ROOT DIRECTORY OF SD CARD
//SD CARD MUST BE FORMATTED IN FAT32, exFAT WILL NOT WORK
// ----------------- STATE STRUCT (MUST MATCH SENDER) -----------------
struct State {
    float yaw;
    float pitch;
    float roll;
    float altitude;
    float accel_z;
    float accel_y;
    float accel_x;
    float TVCPitch;
    float TVCYaw;
    float time;
};

// ----------------- PACKET PARSER -----------------
const uint8_t HEADER0 = 0xAA;
const uint8_t HEADER1 = 0x55;

enum ParseState {
  WAIT_HEADER0,
  WAIT_HEADER1,
  READ_PAYLOAD
};

ParseState parseState = WAIT_HEADER0;

State currentState;
uint8_t payloadBuf[sizeof(State)];
size_t payloadIndex = 0;

// ----------------- SD CARD -----------------
const int SD_CS_PIN = 15;    // <<< CS pin updated here

File logFile;
bool headerWritten = false;

// ----------------- PRINT HELPERS -----------------
void PrintStateDebug(const State &s) {
  Serial.print("t:");      Serial.print(s.time, 3);
  Serial.print("  yaw:");  Serial.print(s.yaw, 2);
  Serial.print("  pitch:");Serial.print(s.pitch, 2);
  Serial.print("  roll:"); Serial.print(s.roll, 2);
  Serial.print("  alt:");  Serial.print(s.altitude, 2);
  Serial.print("  ax:");   Serial.print(s.accel_x, 2);
  Serial.print("  ay:");   Serial.print(s.accel_y, 2);
  Serial.print("  az:");   Serial.print(s.accel_z, 2);
  Serial.print("  TVC_p:");Serial.print(s.TVCPitch, 2);
  Serial.print("  TVC_y:");Serial.println(s.TVCYaw, 2);
}

void PrintStatePlot(const State &s) {
  Serial.print(s.time, 3);     Serial.print(',');
  Serial.print(s.yaw, 3);      Serial.print(',');
  Serial.print(s.pitch, 3);    Serial.print(',');
  Serial.print(s.roll, 3);     Serial.print(',');
  Serial.print(s.altitude, 3); Serial.print(',');
  Serial.print(s.accel_x, 3);  Serial.print(',');
  Serial.print(s.accel_y, 3);  Serial.print(',');
  Serial.print(s.accel_z, 3);  Serial.print(',');
  Serial.print(s.TVCPitch, 3); Serial.print(',');
  Serial.println(s.TVCYaw, 3);
}

// ----------------- SD LOGGING -----------------
void WriteHeaderIfNeeded() {
  if (!logFile || headerWritten) return;

  logFile.println(
    "time,yaw,pitch,roll,altitude,accel_x,accel_y,accel_z,TVCPitch,TVCYaw"
  );
  logFile.flush();
  headerWritten = true;
}

void LogStateCSV(const State &s) {
  if (!logFile) return;

  WriteHeaderIfNeeded();

  logFile.print(s.time, 6);     logFile.print(',');
  logFile.print(s.yaw, 6);      logFile.print(',');
  logFile.print(s.pitch, 6);    logFile.print(',');
  logFile.print(s.roll, 6);     logFile.print(',');
  logFile.print(s.altitude, 6); logFile.print(',');
  logFile.print(s.accel_x, 6);  logFile.print(',');
  logFile.print(s.accel_y, 6);  logFile.print(',');
  logFile.print(s.accel_z, 6);  logFile.print(',');
  logFile.print(s.TVCPitch, 6); logFile.print(',');
  logFile.println(s.TVCYaw, 6);

  logFile.flush();
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial1.begin(115200);

  Serial.println("Logger Teensy starting...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init FAILED. Logging disabled.");
  } else {
    Serial.println("SD init OK.");

    logFile = SD.open("flight.csv", FILE_WRITE);
    if (!logFile) {
      Serial.println("Failed to open flight.csv for writing.");
    } else {
      Serial.println("Logging to flight.csv");
      headerWritten = false;
    }
  }

  Serial.print("Expecting State size = ");
  Serial.println(sizeof(State));
}

// ----------------- LOOP -----------------
void loop() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();

    switch (parseState) {

      case WAIT_HEADER0:
        if (b == HEADER0) parseState = WAIT_HEADER1;
        break;

      case WAIT_HEADER1:
        if (b == HEADER1) {
          payloadIndex = 0;
          parseState = READ_PAYLOAD;
        } else if (b != HEADER0) {
          parseState = WAIT_HEADER0;
        }
        break;

      case READ_PAYLOAD:
        payloadBuf[payloadIndex++] = b;

        if (payloadIndex >= sizeof(State)) {
          memcpy(&currentState, payloadBuf, sizeof(State));

          PrintStatePlot(currentState);     // serial plotter
          LogStateCSV(currentState);        // SD logging

          parseState = WAIT_HEADER0;
        }
        break;
    }
  }
}
