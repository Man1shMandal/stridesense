#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <math.h>

// ============================================================
// PROJECT HEALTH - SMART TREADMILL
// ============================================================

// ---------------- Wi-Fi ----------------

const char* AP_SSID     = "Project Health";
const char* AP_PASSWORD = "12345678";

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

const byte DNS_PORT = 53;

// ---------------- Pins ----------------

#define MPU_ADDR       0x68

#define RFID_SS_PIN    5
#define RFID_RST_PIN   4

#define BUZZER_PIN     15
#define LED_PIN        2

#define I2C_SDA        21
#define I2C_SCL        22

#define SPI_SCK        18
#define SPI_MISO       19
#define SPI_MOSI       23

// ============================================================
// OBJECTS
// ============================================================

WebServer server(80);
DNSServer dnsServer;

MFRC522 rfid(
  RFID_SS_PIN,
  RFID_RST_PIN
);

// ============================================================
// SESSION STATES
// ============================================================

enum SessionState {
  READY,
  COUNTDOWN,
  RUNNING,
  PAUSED,
  COMPLETE
};

SessionState sessionState = READY;

// ============================================================
// SYSTEM STATUS
// ============================================================

bool mpuOK = false;
bool rfidOK = false;
bool wifiOK = false;
bool systemFault = false;

// ============================================================
// MPU DATA
// ============================================================

float ax = 0;
float ay = 0;
float az = 0;

float gx = 0;
float gy = 0;
float gz = 0;

float accelMagnitude = 1.0;
float baselineMagnitude = 1.0;

// Y-axis is the treadmill incline axis
float pitchZeroOffset = 0;
float filteredPitch = 0;
float inclination = 0;

unsigned long lastMPURead = 0;

// ============================================================
// MOTION
// ============================================================

float vibration = 0;
float previousVibration = 0;

unsigned long lastStepTime = 0;

unsigned long stepCount = 0;

float cadence = 0;
float speedKmh = 0;

const float STEP_LENGTH = 0.70;

// ============================================================
// SESSION
// ============================================================

unsigned long sessionStartTime = 0;
unsigned long pausedAt = 0;
unsigned long totalPausedTime = 0;

float sessionDistance = 0;

// ============================================================
// COUNTDOWN
// ============================================================

int countdownValue = 3;

unsigned long countdownStart = 0;

const unsigned long COUNTDOWN_STEP_TIME = 1000;

// ============================================================
// COMPLETE
// ============================================================

unsigned long completeStart = 0;

const unsigned long COMPLETE_DISPLAY_TIME = 4500;

// ============================================================
// BOOT LED
// ============================================================

unsigned long bootStartTime = 0;
bool bootSequenceActive = true;

// ============================================================
// FAULT LED
// ============================================================

unsigned long faultLedTimer = 0;
bool faultLedState = false;

const unsigned long FAULT_BLINK_INTERVAL = 100;

// ============================================================
// BUZZER
// ============================================================

enum SoundType {
  SOUND_NONE,
  SOUND_BOOT,
  SOUND_RFID,
  SOUND_TICK,
  SOUND_START,
  SOUND_PAUSE,
  SOUND_RESUME,
  SOUND_STOP,
  SOUND_COMPLETE,
  SOUND_ERROR
};

SoundType currentSound = SOUND_NONE;

const int MAX_SOUND_NOTES = 8;

int soundNotes[MAX_SOUND_NOTES];
int soundDurations[MAX_SOUND_NOTES];

int soundNoteCount = 0;
int soundNoteIndex = 0;

unsigned long soundNoteStart = 0;
unsigned long soundGapUntil = 0;

bool soundNotePlaying = false;
bool soundActive = false;

// ============================================================
// ACTIVITY HISTORY
// ============================================================

#define HISTORY_SIZE 90

float activityHistory[HISTORY_SIZE];

int historyIndex = 0;

unsigned long lastHistoryUpdate = 0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

bool initMPU();
bool readMPUData();
bool calibrateMPU();

float calculatePitch();
void updateInclination();

void updateMotion();

void handleRFID();

void startCountdown();
void updateCountdown();

void startSession();
void pauseSession();
void resumeSession();
void stopSession();
void completeSession();

void startSound(SoundType sound);
void updateSound();
void stopSound();

void updateBootLED();
void updateFaultLED();

void checkSystemFault();

void updateHistory();

String makeJSON();

void startAccessPoint();
void setupServer();

void handleRoot();
void handleData();
void handlePause();
void handleStop();
void handleNotFound();

// ============================================================
// MPU INITIALIZATION
// ============================================================

bool initMPU() {

  Serial.println();
  Serial.println("================================");
  Serial.println("Initializing MPU6050...");
  Serial.println("================================");

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() != 0) {

    Serial.println("MPU6050 not responding.");

    return false;
  }

  Serial.println("MPU6050 found at 0x68.");

  // WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  uint8_t whoAmI = Wire.read();

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  // Wake MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);

  // Accelerometer ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyroscope ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Digital low-pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  delay(50);

  int validSamples = 0;

  for (int i = 0; i < 10; i++) {

    if (readMPUData()) {

      float magnitude =
        sqrt(
          ax * ax +
          ay * ay +
          az * az
        );

      if (
        magnitude > 0.5 &&
        magnitude < 1.5
      ) {
        validSamples++;
      }
    }

    delay(20);
  }

  if (validSamples >= 5) {

    Serial.println(
      "MPU6050 initialized successfully."
    );

    return true;
  }

  Serial.println(
    "MPU6050 data invalid."
  );

  return false;
}

// ============================================================
// READ MPU DATA
// ============================================================

bool readMPUData() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (
    Wire.requestFrom(
      MPU_ADDR,
      (uint8_t)14
    ) != 14
  ) {
    return false;
  }

  int16_t rawAx =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  int16_t rawAy =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  int16_t rawAz =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  // Temperature
  Wire.read();
  Wire.read();

  int16_t rawGx =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  int16_t rawGy =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  int16_t rawGz =
    ((int16_t)Wire.read() << 8) |
    Wire.read();

  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;

  gx = rawGx / 131.0f;
  gy = rawGy / 131.0f;
  gz = rawGz / 131.0f;

  accelMagnitude =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );

  return true;
}

// ============================================================
// Y-AXIS INCLINATION
// ============================================================

float calculatePitch() {

  /*
     Physical testing confirmed:

       Y = treadmill incline axis
       X = left/right tilt

     Therefore X is NOT used for inclination.
  */

  float horizontal =
    sqrt(
      ax * ax +
      az * az
    );

  return atan2(
    ay,
    horizontal
  ) * 180.0 / PI;
}

// ============================================================
// ZERO CALIBRATION
// ============================================================

bool calibrateMPU() {

  Serial.println();
  Serial.println("================================");
  Serial.println("MPU ZERO CALIBRATION");
  Serial.println("================================");

  Serial.println(
    "Keep treadmill at mechanical 0°."
  );

  Serial.println(
    "Keep treadmill stationary."
  );

  delay(1500);

  const int samples = 300;

  float pitchSum = 0;
  float magnitudeSum = 0;

  int validSamples = 0;

  for (int i = 0; i < samples; i++) {

    if (readMPUData()) {

      if (
        accelMagnitude > 0.75 &&
        accelMagnitude < 1.25
      ) {

        pitchSum += calculatePitch();
        magnitudeSum += accelMagnitude;

        validSamples++;
      }
    }

    delay(7);
  }

  if (validSamples < 150) {

    Serial.println(
      "MPU calibration failed."
    );

    return false;
  }

  pitchZeroOffset =
    pitchSum / validSamples;

  baselineMagnitude =
    magnitudeSum / validSamples;

  filteredPitch =
    pitchZeroOffset;

  inclination = 0;

  Serial.println(
    "Calibration successful."
  );

  Serial.print(
    "Zero angle: "
  );

  Serial.print(
    pitchZeroOffset,
    2
  );

  Serial.println("°");

  return true;
}

// ============================================================
// INCLINATION UPDATE
// ============================================================

void updateInclination() {

  float rawPitch =
    calculatePitch();

  filteredPitch =
    0.94 * filteredPitch +
    0.06 * rawPitch;

  float relativeAngle =
    filteredPitch -
    pitchZeroOffset;

  /*
     A treadmill cannot go downhill.

     Therefore both directions of physical
     displacement from zero are represented
     as positive incline.

       +5° -> 5°
       -5° -> 5°
  */

  inclination =
    fabs(relativeAngle);

  if (inclination < 0.25) {
    inclination = 0;
  }

  inclination =
    constrain(
      inclination,
      0.0,
      60.0
    );
}

// ============================================================
// MOTION / STEP DETECTION
// ============================================================

void updateMotion() {

  if (!mpuOK) {
    return;
  }

  if (
    millis() - lastMPURead < 20
  ) {
    return;
  }

  lastMPURead = millis();

  if (!readMPUData()) {

    mpuOK = false;

    return;
  }

  updateInclination();

  float difference =
    fabs(
      accelMagnitude -
      baselineMagnitude
    );

  vibration =
    0.80 * vibration +
    0.20 * difference;

  unsigned long now = millis();

  bool risingPeak =
    vibration > previousVibration;

  bool aboveThreshold =
    vibration > 0.035;

  bool refractoryPassed =
    now - lastStepTime > 250;

  if (
    risingPeak &&
    aboveThreshold &&
    refractoryPassed &&
    sessionState == RUNNING
  ) {

    stepCount++;

    if (lastStepTime > 0) {

      unsigned long interval =
        now - lastStepTime;

      if (
        interval > 250 &&
        interval < 2000
      ) {

        float instantCadence =
          60000.0 / interval;

        cadence =
          0.75 * cadence +
          0.25 * instantCadence;
      }
    }

    lastStepTime = now;

    sessionDistance =
      stepCount *
      STEP_LENGTH /
      1000.0;

    speedKmh =
      cadence *
      STEP_LENGTH *
      60.0 /
      1000.0;

    speedKmh =
      constrain(
        speedKmh,
        0,
        25
      );
  }

  previousVibration =
    vibration;

  // Speed and cadence decay when movement stops
  if (
    sessionState == RUNNING &&
    lastStepTime > 0
  ) {

    unsigned long timeSinceStep =
      now - lastStepTime;

    if (
      timeSinceStep > 1800
    ) {

      float decay =
        (
          timeSinceStep -
          1800
        ) / 2000.0;

      decay =
        constrain(
          decay,
          0,
          1
        );

      speedKmh *=
        (1.0 - decay);

      cadence *=
        (1.0 - decay);

      if (speedKmh < 0.05)
        speedKmh = 0;

      if (cadence < 1)
        cadence = 0;
    }
  }
}

// ============================================================
// RFID
// ============================================================

void handleRFID() {

  if (!rfidOK)
    return;

  if (!rfid.PICC_IsNewCardPresent())
    return;

  if (!rfid.PICC_ReadCardSerial())
    return;

  Serial.print("RFID detected: ");

  for (
    byte i = 0;
    i < rfid.uid.size;
    i++
  ) {

    if (
      rfid.uid.uidByte[i] < 0x10
    ) {
      Serial.print("0");
    }

    Serial.print(
      rfid.uid.uidByte[i],
      HEX
    );
  }

  Serial.println();

  if (sessionState == READY) {

    startSound(SOUND_RFID);

    delay(180);

    startCountdown();
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ============================================================
// COUNTDOWN
// ============================================================

void startCountdown() {

  if (sessionState != READY)
    return;

  sessionState =
    COUNTDOWN;

  countdownValue = 3;

  countdownStart =
    millis();

  startSound(
    SOUND_TICK
  );
}

void updateCountdown() {

  if (sessionState != COUNTDOWN)
    return;

  unsigned long now =
    millis();

  if (
    now - countdownStart >=
    COUNTDOWN_STEP_TIME
  ) {

    countdownStart =
      now;

    countdownValue--;

    if (countdownValue > 0) {

      startSound(
        SOUND_TICK
      );

    } else {

      startSession();
    }
  }
}

// ============================================================
// START SESSION
// ============================================================

void startSession() {

  sessionState =
    RUNNING;

  sessionStartTime =
    millis();

  pausedAt = 0;
  totalPausedTime = 0;

  stepCount = 0;

  sessionDistance = 0;

  cadence = 0;
  speedKmh = 0;

  /*
     IMPORTANT:
     Animation will NOT start here.
     It starts only after the first
     valid step updates lastStepTime.
  */

  lastStepTime = 0;

  vibration = 0;
  previousVibration = 0;

  startSound(
    SOUND_START
  );

  digitalWrite(
    LED_PIN,
    HIGH
  );
}

// ============================================================
// PAUSE
// ============================================================

void pauseSession() {

  if (sessionState != RUNNING)
    return;

  pausedAt =
    millis();

  sessionState =
    PAUSED;

  speedKmh = 0;
  cadence = 0;

  startSound(
    SOUND_PAUSE
  );

  digitalWrite(
    LED_PIN,
    LOW
  );
}

// ============================================================
// RESUME
// ============================================================

void resumeSession() {

  if (sessionState != PAUSED)
    return;

  totalPausedTime +=
    millis() - pausedAt;

  sessionState =
    RUNNING;

  /*
     Do not start animation on resume.
     A new step must be detected first.
  */

  lastStepTime = 0;

  startSound(
    SOUND_RESUME
  );

  digitalWrite(
    LED_PIN,
    HIGH
  );
}

// ============================================================
// STOP
// ============================================================

void stopSession() {

  if (
    sessionState != RUNNING &&
    sessionState != PAUSED
  )
    return;

  sessionState =
    COMPLETE;

  speedKmh = 0;
  cadence = 0;

  completeStart =
    millis();

  startSound(
    SOUND_STOP
  );

  digitalWrite(
    LED_PIN,
    LOW
  );
}

// ============================================================
// COMPLETE
// ============================================================

void completeSession() {

  if (
    millis() -
    completeStart >=
    COMPLETE_DISPLAY_TIME
  ) {

    sessionState =
      READY;

    stepCount = 0;
    sessionDistance = 0;

    speedKmh = 0;
    cadence = 0;

    lastStepTime = 0;

    digitalWrite(
      LED_PIN,
      LOW
    );
  }
}

// ============================================================
// BUZZER
// ============================================================

void startSound(
  SoundType sound
) {

  stopSound();

  currentSound =
    sound;

  switch (sound) {

    case SOUND_BOOT:

      soundNotes[0] = 523;
      soundNotes[1] = 659;
      soundNotes[2] = 784;
      soundNotes[3] = 1047;

      soundDurations[0] = 80;
      soundDurations[1] = 80;
      soundDurations[2] = 80;
      soundDurations[3] = 180;

      soundNoteCount = 4;

      break;

    case SOUND_RFID:

      soundNotes[0] = 880;
      soundNotes[1] = 1175;

      soundDurations[0] = 80;
      soundDurations[1] = 160;

      soundNoteCount = 2;

      break;

    case SOUND_TICK:

      soundNotes[0] = 880;

      soundDurations[0] = 70;

      soundNoteCount = 1;

      break;

    case SOUND_START:

      soundNotes[0] = 659;
      soundNotes[1] = 880;
      soundNotes[2] = 1175;

      soundDurations[0] = 70;
      soundDurations[1] = 70;
      soundDurations[2] = 180;

      soundNoteCount = 3;

      break;

    case SOUND_PAUSE:

      soundNotes[0] = 988;
      soundNotes[1] = 784;

      soundDurations[0] = 90;
      soundDurations[1] = 160;

      soundNoteCount = 2;

      break;

    case SOUND_RESUME:

      soundNotes[0] = 784;
      soundNotes[1] = 988;
      soundNotes[2] = 1175;

      soundDurations[0] = 70;
      soundDurations[1] = 70;
      soundDurations[2] = 180;

      soundNoteCount = 3;

      break;

    case SOUND_STOP:

      soundNotes[0] = 784;
      soundNotes[1] = 659;

      soundDurations[0] = 90;
      soundDurations[1] = 180;

      soundNoteCount = 2;

      break;

    case SOUND_COMPLETE:

      soundNotes[0] = 784;
      soundNotes[1] = 988;
      soundNotes[2] = 1175;

      soundDurations[0] = 80;
      soundDurations[1] = 80;
      soundDurations[2] = 220;

      soundNoteCount = 3;

      break;

    case SOUND_ERROR:

      soundNotes[0] = 330;
      soundNotes[1] = 247;

      soundDurations[0] = 120;
      soundDurations[1] = 220;

      soundNoteCount = 2;

      break;

    default:

      soundNoteCount = 0;
      return;
  }

  soundNoteIndex = 0;

  soundNoteStart =
    millis();

  soundGapUntil = 0;

  soundNotePlaying = false;

  soundActive = true;
}

// ============================================================
// BUZZER UPDATE
// ============================================================

void updateSound() {

  if (!soundActive)
    return;

  unsigned long now =
    millis();

  if (
    soundGapUntil > 0 &&
    now < soundGapUntil
  ) {
    return;
  }

  if (!soundNotePlaying) {

    if (
      soundNoteIndex >=
      soundNoteCount
    ) {

      stopSound();

      return;
    }

    tone(
      BUZZER_PIN,
      soundNotes[soundNoteIndex]
    );

    soundNoteStart =
      now;

    soundNotePlaying = true;

    return;
  }

  if (
    now - soundNoteStart >=
    (unsigned long)
    soundDurations[soundNoteIndex]
  ) {

    noTone(BUZZER_PIN);

    soundNotePlaying = false;

    soundNoteIndex++;

    soundGapUntil =
      now + 25;

    if (
      soundNoteIndex >=
      soundNoteCount
    ) {

      stopSound();
    }
  }
}

// ============================================================
// STOP SOUND
// ============================================================

void stopSound() {

  noTone(BUZZER_PIN);

  soundActive = false;
  soundNotePlaying = false;

  soundNoteIndex = 0;
  soundNoteCount = 0;

  soundGapUntil = 0;

  currentSound =
    SOUND_NONE;
}

// ============================================================
// BOOT LED
// ============================================================

void updateBootLED() {

  if (!bootSequenceActive)
    return;

  unsigned long elapsed =
    millis() -
    bootStartTime;

  if (elapsed < 150) {

    analogWrite(
      LED_PIN,
      35
    );

  } else if (elapsed < 300) {

    analogWrite(
      LED_PIN,
      180
    );

  } else if (elapsed < 450) {

    analogWrite(
      LED_PIN,
      35
    );

  } else if (elapsed < 600) {

    analogWrite(
      LED_PIN,
      255
    );

  } else if (elapsed < 750) {

    analogWrite(
      LED_PIN,
      35
    );

  } else if (elapsed < 900) {

    analogWrite(
      LED_PIN,
      255
    );

  } else {

    bootSequenceActive = false;

    digitalWrite(
      LED_PIN,
      LOW
    );
  }
}

// ============================================================
// FAULT CHECK
// ============================================================

void checkSystemFault() {

  systemFault =
    !mpuOK ||
    !rfidOK ||
    !wifiOK;
}

// ============================================================
// FAULT LED
// ============================================================

void updateFaultLED() {

  if (!systemFault)
    return;

  if (
    millis() -
    faultLedTimer >=
    FAULT_BLINK_INTERVAL
  ) {

    faultLedTimer =
      millis();

    faultLedState =
      !faultLedState;

    digitalWrite(
      LED_PIN,
      faultLedState
    );
  }
}

// ============================================================
// ACTIVITY HISTORY
// ============================================================

void updateHistory() {

  if (
    millis() -
    lastHistoryUpdate <
    200
  )
    return;

  lastHistoryUpdate =
    millis();

  float activity = 0;

  /*
     Activity graph only responds to
     actual detected movement.
  */

  if (
    sessionState == RUNNING &&
    lastStepTime > 0 &&
    millis() - lastStepTime < 1800
  ) {

    activity =
      constrain(
        cadence / 160.0,
        0,
        1
      );
  }

  activityHistory[
    historyIndex
  ] = activity;

  historyIndex++;

  if (
    historyIndex >=
    HISTORY_SIZE
  ) {

    historyIndex = 0;
  }
}

// ============================================================
// WI-FI ACCESS POINT
// ============================================================

void startAccessPoint() {

  Serial.println();
  Serial.println(
    "Starting Project Health AP..."
  );

  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(
    AP_IP,
    AP_GATEWAY,
    AP_SUBNET
  );

  bool started =
    WiFi.softAP(
      AP_SSID,
      AP_PASSWORD
    );

  if (started) {

    wifiOK = true;

    dnsServer.start(
      DNS_PORT,
      "*",
      AP_IP
    );

    Serial.println(
      "Wi-Fi AP started."
    );

    Serial.print(
      "SSID: "
    );

    Serial.println(
      AP_SSID
    );

    Serial.println(
      "Password: 12345678"
    );

    Serial.print(
      "IP: "
    );

    Serial.println(
      WiFi.softAPIP()
    );

  } else {

    wifiOK = false;

    Serial.println(
      "Wi-Fi AP FAILED."
    );
  }
}

// ============================================================
// JSON DATA
// ============================================================

String makeJSON() {

  String json = "{";

  json += "\"state\":\"";

  switch (sessionState) {

    case READY:
      json += "READY";
      break;

    case COUNTDOWN:
      json += "COUNTDOWN";
      break;

    case RUNNING:
      json += "RUNNING";
      break;

    case PAUSED:
      json += "PAUSED";
      break;

    case COMPLETE:
      json += "COMPLETE";
      break;
  }

  json += "\",";

  json += "\"user\":\"User\",";

  json += "\"speed\":";
  json += String(speedKmh, 1);
  json += ",";

  json += "\"distance\":";
  json += String(sessionDistance, 2);
  json += ",";

  json += "\"steps\":";
  json += String(stepCount);
  json += ",";

  json += "\"cadence\":";
  json += String(cadence, 0);
  json += ",";

  json += "\"inclination\":";
  json += String(inclination, 1);
  json += ",";

  // Session time
  unsigned long sessionSeconds = 0;

  if (
    sessionState == RUNNING ||
    sessionState == PAUSED
  ) {

    unsigned long now =
      millis();

    unsigned long paused =
      totalPausedTime;

    if (
      sessionState == PAUSED
    ) {

      paused +=
        now - pausedAt;
    }

    if (
      now >=
      sessionStartTime +
      paused
    ) {

      sessionSeconds =
        (
          now -
          sessionStartTime -
          paused
        ) / 1000;
    }
  }

  json += "\"session\":";
  json += String(sessionSeconds);
  json += ",";

  json += "\"countdown\":";
  json += String(countdownValue);
  json += ",";

  json += "\"mpu\":";
  json +=
    mpuOK ? "true" : "false";

  json += ",";

  json += "\"rfid\":";
  json +=
    rfidOK ? "true" : "false";

  json += ",";

  json += "\"wifi\":";
  json +=
    wifiOK ? "true" : "false";

  json += ",";

  /*
     Animation state.

     The treadmill/person moves ONLY if:
       1. Session is RUNNING
       2. At least one step has been detected
       3. The last step was within 1.8 seconds
  */

  bool moving =
    sessionState == RUNNING &&
    lastStepTime > 0 &&
    (millis() - lastStepTime) < 1800;

  json += "\"moving\":";
  json +=
    moving ? "true" : "false";

  json += ",";

  json += "\"fault\":";
  json +=
    systemFault
    ? "true"
    : "false";

  json += "}";

  return json;
}

// ============================================================
// WEB UI
// ============================================================

void handleRoot() {

  String html = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta name="viewport"
content="width=device-width,
initial-scale=1,
maximum-scale=1,
user-scalable=no,
viewport-fit=cover">

<title>Project Health</title>

<style>

* {
  box-sizing: border-box;
}

html,
body {

  margin: 0;

  width: 100%;
  height: 100%;

  overflow: hidden;

  background: #f4f6f8;

  color: #111827;

  font-family:
    -apple-system,
    BlinkMacSystemFont,
    "Segoe UI",
    Roboto,
    Arial,
    sans-serif;
}

body {
  min-height: 100dvh;
}

.app {

  width: 100%;

  max-width: 560px;

  height: 100dvh;

  margin: auto;

  padding:
    max(10px, env(safe-area-inset-top))
    12px
    max(8px, env(safe-area-inset-bottom))
    12px;

  display: flex;

  flex-direction: column;

  overflow: hidden;
}

/* HEADER */

.header {

  flex: 0 0 auto;

  display: flex;

  justify-content:
    space-between;

  align-items: center;

  margin-bottom: 2px;
}

.title {

  font-size: 22px;

  font-weight: 750;

  line-height: 1.1;
}

.subtitle {

  font-size: 12px;

  color: #6b7280;

  margin-top: 3px;
}

.status {

  display: flex;

  align-items: center;

  gap: 6px;

  font-size: 11px;

  color: #6b7280;
}

.dot {

  width: 8px;
  height: 8px;

  border-radius: 50%;

  background: #22c55e;
}

/* MACHINE */

.machineArea {

  flex: 1 1 auto;

  min-height: 170px;

  max-height: 245px;

  display: flex;

  align-items: center;

  justify-content: center;

  position: relative;

  overflow: visible;
}

.machine {

  position: relative;

  width: min(340px, 90vw);

  height: 150px;

  transform-origin:
    center center;

  transition:
    transform .25s ease;
}

/* BELT */

.belt {

  position: absolute;

  left: 7%;

  bottom: 15px;

  width: 86%;

  height: 30px;

  border-radius: 18px;

  background:
    repeating-linear-gradient(
      90deg,
      #1f2937 0px,
      #1f2937 18px,
      #4b5563 19px,
      #4b5563 22px
    );

  background-position:
    0 0;

  border: 5px solid #374151;

  box-shadow:
    0 5px 15px
    rgba(0,0,0,.12);
}

.running .belt {

  animation:
    treadmillRoll
    .35s
    linear
    infinite;
}

@keyframes treadmillRoll {

  from {
    background-position-x: 0;
  }

  to {
    background-position-x: -40px;
  }
}

.beltLine {

  position: absolute;

  left: 12%;

  bottom: 26px;

  width: 76%;

  height: 3px;

  border-radius: 2px;

  background: #9ca3af;

  opacity: .5;
}

/* CONSOLE */

.console {

  position: absolute;

  top: 8px;

  right: 7%;

  width: 70px;

  height: 34px;

  border-radius: 8px;

  background: #111827;
}

.consoleScreen {

  position: absolute;

  top: 6px;

  left: 7px;

  right: 7px;

  bottom: 6px;

  border-radius: 4px;

  background: #2563eb;
}

.support {

  position: absolute;

  top: 32px;

  right: 18%;

  width: 7px;

  height: 70px;

  border-radius: 5px;

  background: #9ca3af;

  transform:
    rotate(-8deg);
}

/* PERSON */

.person {

  position: absolute;

  left: 29%;

  bottom: 46px;

  width: 90px;

  height: 120px;

  transform-origin:
    center bottom;
}

.head {

  position: absolute;

  top: 0;

  left: 33px;

  width: 23px;

  height: 23px;

  border-radius: 50%;

  background: #111827;
}

.body {

  position: absolute;

  top: 23px;

  left: 38px;

  width: 14px;

  height: 46px;

  border-radius: 8px;

  background: #111827;
}

.arm {

  position: absolute;

  top: 27px;

  width: 9px;

  height: 41px;

  border-radius: 7px;

  background: #111827;

  transform-origin:
    top center;
}

.arm.left {

  left: 28px;

  transform:
    rotate(25deg);
}

.arm.right {

  left: 50px;

  transform:
    rotate(-25deg);
}

.leg {

  position: absolute;

  top: 65px;

  width: 10px;

  height: 48px;

  border-radius: 7px;

  background: #111827;

  transform-origin:
    top center;
}

.leg.left {
  left: 34px;
}

.leg.right {
  left: 44px;
}

/* PERSON MOVEMENT ONLY WHEN ACTUALLY MOVING */

.active .leg.left {

  animation:
    legLeft .55s
    infinite
    ease-in-out;
}

.active .leg.right {

  animation:
    legRight .55s
    infinite
    ease-in-out;
}

.active .arm.left {

  animation:
    armLeft .55s
    infinite
    ease-in-out;
}

.active .arm.right {

  animation:
    armRight .55s
    infinite
    ease-in-out;
}

@keyframes legLeft {

  0%,100% {
    transform:
      rotate(25deg);
  }

  50% {
    transform:
      rotate(-30deg);
  }
}

@keyframes legRight {

  0%,100% {
    transform:
      rotate(-30deg);
  }

  50% {
    transform:
      rotate(25deg);
  }
}

@keyframes armLeft {

  0%,100% {
    transform:
      rotate(25deg);
  }

  50% {
    transform:
      rotate(-25deg);
  }
}

@keyframes armRight {

  0%,100% {
    transform:
      rotate(-25deg);
  }

  50% {
    transform:
      rotate(25deg);
  }
}

/* INCLINATION */

.inclineLabel {

  flex: 0 0 auto;

  text-align: center;

  font-size: 14px;

  color: #6b7280;

  margin:
    0 0 7px;
}

.inclineLabel strong {

  color: #111827;

  font-size: 15px;
}

/* GRAPH */

.graphCard {

  flex: 0 0 auto;

  background: white;

  border-radius: 15px;

  padding: 8px 12px;

  margin-bottom: 8px;

  box-shadow:
    0 2px 10px
    rgba(0,0,0,.04);
}

.graphTitle {

  font-size: 14px;

  font-weight: 650;
}

.graphSubtitle {

  font-size: 10px;

  color: #9ca3af;

  margin-bottom: 3px;
}

#graph {

  width: 100%;

  height: 42px;

  display: block;
}

/* METRICS */

.metrics {

  flex: 0 0 auto;

  display: grid;

  grid-template-columns:
    repeat(2,1fr);

  gap: 8px;

  margin-bottom: 8px;
}

.metric {

  background: white;

  border-radius: 14px;

  padding: 9px 12px;

  min-height: 57px;

  box-shadow:
    0 2px 10px
    rgba(0,0,0,.04);
}

.metricLabel {

  font-size: 10px;

  color: #9ca3af;

  margin-bottom: 3px;
}

.metricValue {

  font-size: 22px;

  font-weight: 700;

  line-height: 1.05;
}

.metricUnit {

  font-size: 10px;

  color: #9ca3af;
}

/* CONTROLS */

.controls {

  flex: 0 0 auto;

  display: grid;

  grid-template-columns:
    1fr 1fr;

  gap: 8px;

  margin-bottom: 7px;
}

button {

  border: none;

  border-radius: 13px;

  min-height: 44px;

  padding: 10px;

  font-size: 14px;

  font-weight: 650;

  -webkit-tap-highlight-color:
    transparent;
}

.pause {

  background: #111827;

  color: white;
}

.stop {

  background: #fee2e2;

  color: #b91c1c;
}

/* FOOTER */

.footer {

  flex: 0 0 auto;

  display: flex;

  justify-content:
    space-between;

  align-items: center;

  font-size: 9px;

  color: #9ca3af;
}

.sensorGroup {

  display: flex;

  gap: 12px;

  align-items: center;
}

.sensor {

  display: flex;

  align-items: center;

  gap: 4px;

  font-size: 9px;

  color: #6b7280;
}

.sensorDot {

  width: 7px;

  height: 7px;

  border-radius: 50%;

  background: #22c55e;

  box-shadow:
    0 0 5px
    rgba(34,197,94,.35);
}

.brand {

  text-align: right;
}

.brand strong {

  display: block;

  color: #6b7280;

  font-size: 10px;
}

.brand span {

  font-size: 8px;
}

/* OVERLAYS */

.overlay {

  position: fixed;

  inset: 0;

  display: none;

  align-items: center;

  justify-content: center;

  background:
    rgba(255,255,255,.94);

  z-index: 10;
}

.overlay.show {
  display: flex;
}

.overlayContent {
  text-align: center;
}

.countdown {

  font-size: 70px;

  font-weight: 800;
}

.overlayText {

  font-size: 13px;

  color: #6b7280;
}

.completeTitle {

  font-size: 28px;

  font-weight: 800;
}

.completeStats {

  font-size: 13px;

  color: #6b7280;
}

/* SHORT PHONES */

@media (max-height: 650px) {

  .machineArea {

    min-height: 145px;

    max-height: 190px;
  }

  .machine {

    width:
      min(310px, 88vw);

    height: 135px;
  }

  .graphCard {

    padding: 6px 10px;

    margin-bottom: 6px;
  }

  #graph {
    height: 32px;
  }

  .metric {

    min-height: 50px;

    padding: 7px 10px;
  }

  .metricValue {
    font-size: 19px;
  }

  button {
    min-height: 40px;
  }
}

</style>

</head>

<body>

<div class="app">

<!-- HEADER -->

<div class="header">

<div>

<div class="title">
Project Health
</div>

<div class="subtitle">
Activity Monitor - User
</div>

</div>

<div class="status">

<span
class="dot"
id="wifiDot">
</span>

<span id="state">
READY
</span>

</div>

</div>

<!-- TREADMILL -->

<div class="machineArea">

<div
class="machine"
id="machine">

<div class="console">

<div class="consoleScreen"></div>

</div>

<div class="support"></div>

<div class="belt"></div>

<div class="beltLine"></div>

<div
class="person"
id="person">

<div class="head"></div>

<div class="body"></div>

<div class="arm left"></div>

<div class="arm right"></div>

<div class="leg left"></div>

<div class="leg right"></div>

</div>

</div>

</div>

<!-- INCLINATION -->

<div class="inclineLabel">

Inclination:
<strong id="inclination">
0.0°
</strong>

</div>

<!-- GRAPH -->

<div class="graphCard">

<div class="graphTitle">
Activity
</div>

<div class="graphSubtitle">
Movement intensity
</div>

<svg
id="graph"
viewBox="0 0 500 70"
preserveAspectRatio="none">

<defs>

<linearGradient
id="gradient"
x1="0"
x2="0"
y1="0"
y2="1">

<stop
offset="0%"
stop-color="#60a5fa"
stop-opacity=".30"/>

<stop
offset="100%"
stop-color="#60a5fa"
stop-opacity=".02"/>

</linearGradient>

</defs>

<path
id="graphFill"
fill="url(#gradient)"
stroke="none"/>

<path
id="graphLine"
fill="none"
stroke="#2563eb"
stroke-width="2"
stroke-linecap="round"
stroke-linejoin="round"/>

</svg>

</div>

<!-- METRICS -->

<div class="metrics">

<div class="metric">

<div class="metricLabel">
SPEED
</div>

<div class="metricValue">

<span id="speed">
0.0
</span>

<span class="metricUnit">
km/h
</span>

</div>

</div>

<div class="metric">

<div class="metricLabel">
DISTANCE
</div>

<div class="metricValue">

<span id="distance">
0.00
</span>

<span class="metricUnit">
km
</span>

</div>

</div>

<div class="metric">

<div class="metricLabel">
STEPS
</div>

<div class="metricValue">

<span id="steps">
0
</span>

</div>

</div>

<div class="metric">

<div class="metricLabel">
SESSION
</div>

<div class="metricValue">

<span id="session">
00:00
</span>

</div>

</div>

</div>

<!-- CONTROLS -->

<div class="controls">

<button
class="pause"
id="pauseBtn"
onclick="togglePause()">

Pause

</button>

<button
class="stop"
onclick="stopSession()">

Stop

</button>

</div>

<!-- STATUS -->

<div class="footer">

<div class="sensorGroup">

<div class="sensor">

<span
class="sensorDot"
id="mpuDot">
</span>

<span>MPU</span>

</div>

<div class="sensor">

<span
class="sensorDot"
id="rfidDot">
</span>

<span>RFID</span>

</div>

<div class="sensor">

<span
class="sensorDot"
id="wifiStatusDot">
</span>

<span>Wi-Fi</span>

</div>

</div>

<div class="brand">

<strong>
Project Health
</strong>

<span>
Smart Treadmill
</span>

</div>

</div>

</div>

<!-- COUNTDOWN -->

<div
class="overlay"
id="countdownOverlay">

<div class="overlayContent">

<div
class="countdown"
id="countdownNumber">
3
</div>

<div class="overlayText">
Get ready
</div>

</div>

</div>

<!-- COMPLETE -->

<div
class="overlay"
id="completeOverlay">

<div class="overlayContent">

<div class="completeTitle">
Session Complete
</div>

<div class="completeStats">

<span id="completeSteps">
0
</span>

steps

&nbsp;·&nbsp;

<span id="completeDistance">
0.00
</span>

km

</div>

</div>

</div>

<script>

let history = [];

const HISTORY_SIZE = 90;

// ============================================================
// TIME
// ============================================================

function formatTime(seconds) {

  seconds =
    Number(seconds);

  let m =
    Math.floor(
      seconds / 60
    );

  let s =
    seconds % 60;

  return (
    String(m).padStart(2,"0")
    +
    ":"
    +
    String(s).padStart(2,"0")
  );
}

// ============================================================
// GRAPH
// ============================================================

function updateGraph(value) {

  history.push(
    Number(value)
  );

  if (
    history.length >
    HISTORY_SIZE
  ) {

    history.shift();
  }

  let points = [];

  const width = 500;
  const bottom = 66;

  for (
    let i = 0;
    i < history.length;
    i++
  ) {

    let x =
      i *
      width /
      (HISTORY_SIZE - 1);

    let v =
      Math.max(
        0,
        Math.min(
          1,
          history[i]
        )
      );

    let y =
      bottom -
      v * 55;

    points.push([
      x,
      y
    ]);
  }

  if (
    points.length < 2
  ) {
    return;
  }

  let line = "";

  for (
    let i = 0;
    i < points.length;
    i++
  ) {

    line +=
      (
        i === 0
        ? "M "
        : " L "
      )
      +
      points[i][0].toFixed(1)
      +
      " "
      +
      points[i][1].toFixed(1);
  }

  let last =
    points[
      points.length - 1
    ];

  let fill =
    line
    +
    " L "
    +
    last[0]
    +
    " "
    +
    bottom
    +
    " L 0 "
    +
    bottom
    +
    " Z";

  document
    .getElementById(
      "graphLine"
    )
    .setAttribute(
      "d",
      line
    );

  document
    .getElementById(
      "graphFill"
    )
    .setAttribute(
      "d",
      fill
    );
}

// ============================================================
// STATUS DOT
// ============================================================

function setDot(
  id,
  ok
) {

  const dot =
    document.getElementById(id);

  dot.style.background =
    ok
    ? "#22c55e"
    : "#ef4444";

  dot.style.boxShadow =
    ok
    ? "0 0 5px rgba(34,197,94,.35)"
    : "0 0 5px rgba(239,68,68,.45)";
}

// ============================================================
// UI UPDATE
// ============================================================

function updateUI(data) {

  document.getElementById(
    "speed"
  ).textContent =
    Number(
      data.speed
    ).toFixed(1);

  document.getElementById(
    "distance"
  ).textContent =
    Number(
      data.distance
    ).toFixed(2);

  document.getElementById(
    "steps"
  ).textContent =
    data.steps;

  document.getElementById(
    "session"
  ).textContent =
    formatTime(
      data.session
    );

  document.getElementById(
    "inclination"
  ).textContent =
    Number(
      data.inclination
    ).toFixed(1)
    +
    "°";

  document.getElementById(
    "state"
  ).textContent =
    data.state;

  // Three bottom status indicators
  setDot(
    "mpuDot",
    data.mpu
  );

  setDot(
    "rfidDot",
    data.rfid
  );

  setDot(
    "wifiStatusDot",
    data.wifi
  );

  setDot(
    "wifiDot",
    data.wifi
  );

  // ==========================================================
  // TREADMILL INCLINATION
  // ==========================================================

  /*
     Console/front is on the RIGHT.

     Negative CSS rotation makes the
     right/front side rise.
  */

  const visualAngle =
    -Number(
      data.inclination
    ) * 0.42;

  const machine =
    document.getElementById(
      "machine"
    );

  machine.style.transform =
    "rotate("
    +
    visualAngle
    +
    "deg)";

  // ==========================================================
  // MOTION ANIMATION
  // ==========================================================

  const person =
    document.getElementById(
      "person"
    );

  /*
     IMPORTANT:

     Animation does NOT start merely because
     RFID started the session.

     It starts ONLY when ESP32 detects
     a real step.
  */

  if (
    data.moving === true
  ) {

    person.classList.add(
      "active"
    );

    machine.classList.add(
      "running"
    );

  } else {

    person.classList.remove(
      "active"
    );

    machine.classList.remove(
      "running"
    );
  }

  // ==========================================================
  // PAUSE BUTTON
  // ==========================================================

  const pauseBtn =
    document.getElementById(
      "pauseBtn"
    );

  pauseBtn.textContent =
    data.state === "PAUSED"
    ? "Resume"
    : "Pause";

  // ==========================================================
  // COUNTDOWN
  // ==========================================================

  const countdownOverlay =
    document.getElementById(
      "countdownOverlay"
    );

  if (
    data.state ===
    "COUNTDOWN"
  ) {

    countdownOverlay.classList.add(
      "show"
    );

    document.getElementById(
      "countdownNumber"
    ).textContent =
      data.countdown;

  } else {

    countdownOverlay.classList.remove(
      "show"
    );
  }

  // ==========================================================
  // COMPLETE
  // ==========================================================

  const completeOverlay =
    document.getElementById(
      "completeOverlay"
    );

  if (
    data.state ===
    "COMPLETE"
  ) {

    completeOverlay.classList.add(
      "show"
    );

    document.getElementById(
      "completeSteps"
    ).textContent =
      data.steps;

    document.getElementById(
      "completeDistance"
    ).textContent =
      Number(
        data.distance
      ).toFixed(2);

  } else {

    completeOverlay.classList.remove(
      "show"
    );
  }

  // ==========================================================
  // ACTIVITY GRAPH
  // ==========================================================

  let activity = 0;

  if (
    data.moving === true
  ) {

    activity =
      Math.min(
        1,
        Number(
          data.cadence
        ) / 160
      );
  }

  updateGraph(
    activity
  );
}

// ============================================================
// FETCH DATA
// ============================================================

async function update() {

  try {

    const response =
      await fetch(
        "/data",
        {
          cache:
            "no-store"
        }
      );

    const data =
      await response.json();

    updateUI(data);

  } catch (error) {

    setDot(
      "wifiStatusDot",
      false
    );

    setDot(
      "wifiDot",
      false
    );
  }
}

// ============================================================
// PAUSE / RESUME
// ============================================================

async function togglePause() {

  try {

    await fetch(
      "/pause",
      {
        method: "POST"
      }
    );

  } catch (error) {

    console.log(error);
  }
}

// ============================================================
// STOP
// ============================================================

async function stopSession() {

  try {

    await fetch(
      "/stop",
      {
        method: "POST"
      }
    );

  } catch (error) {

    console.log(error);
  }
}

// ============================================================
// DATA POLLING
// ============================================================

setInterval(
  update,
  200
);

update();

</script>

</body>

</html>

)rawliteral";

  server.send(
    200,
    "text/html",
    html
  );
}

// ============================================================
// DATA ENDPOINT
// ============================================================

void handleData() {

  server.send(
    200,
    "application/json",
    makeJSON()
  );
}

// ============================================================
// PAUSE / RESUME ENDPOINT
// ============================================================

void handlePause() {

  if (
    sessionState == RUNNING
  ) {

    pauseSession();

  } else if (
    sessionState == PAUSED
  ) {

    resumeSession();
  }

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// ============================================================
// STOP ENDPOINT
// ============================================================

void handleStop() {

  stopSession();

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// ============================================================
// CAPTIVE PORTAL
// ============================================================

void handleNotFound() {

  server.sendHeader(
    "Location",
    "http://192.168.4.1/",
    true
  );

  server.send(
    302,
    "text/plain",
    ""
  );
}

// ============================================================
// SERVER SETUP
// ============================================================

void setupServer() {

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/data",
    HTTP_GET,
    handleData
  );

  server.on(
    "/pause",
    HTTP_POST,
    handlePause
  );

  server.on(
    "/stop",
    HTTP_POST,
    handleStop
  );

  // Android
  server.on(
    "/generate_204",
    HTTP_GET,
    []() {

      server.sendHeader(
        "Location",
        "http://192.168.4.1/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );

  server.on(
    "/gen_204",
    HTTP_GET,
    []() {

      server.sendHeader(
        "Location",
        "http://192.168.4.1/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );

  // Apple
  server.on(
    "/hotspot-detect.html",
    HTTP_GET,
    []() {

      server.sendHeader(
        "Location",
        "http://192.168.4.1/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );

  // Windows
  server.on(
    "/connecttest.txt",
    HTTP_GET,
    []() {

      server.sendHeader(
        "Location",
        "http://192.168.4.1/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );

  server.on(
    "/ncsi.txt",
    HTTP_GET,
    []() {

      server.sendHeader(
        "Location",
        "http://192.168.4.1/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );

  server.onNotFound(
    handleNotFound
  );

  server.begin();

  Serial.println(
    "Web server started."
  );
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );

  delay(500);

  Serial.println();
  Serial.println();
  Serial.println(
    "================================"
  );
  Serial.println(
    "       PROJECT HEALTH"
  );
  Serial.println(
    "      SMART TREADMILL"
  );
  Serial.println(
    "================================"
  );

  pinMode(
    LED_PIN,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    LED_PIN,
    LOW
  );

  noTone(
    BUZZER_PIN
  );

  // ==========================================================
  // BOOT SEQUENCE
  // ==========================================================

  bootStartTime =
    millis();

  bootSequenceActive = true;

  startSound(
    SOUND_BOOT
  );

  // ==========================================================
  // I2C
  // ==========================================================

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  Wire.setClock(
    400000
  );

  // ==========================================================
  // MPU
  // ==========================================================

  mpuOK =
    initMPU();

  if (mpuOK) {

    if (!calibrateMPU()) {

      mpuOK = false;
    }
  }

  // ==========================================================
  // RFID
  // ==========================================================

  Serial.println();
  Serial.println(
    "Initializing RFID..."
  );

  SPI.begin(
    SPI_SCK,
    SPI_MISO,
    SPI_MOSI,
    RFID_SS_PIN
  );

  rfid.PCD_Init();

  delay(50);

  byte version =
    rfid.PCD_ReadRegister(
      MFRC522::VersionReg
    );

  if (
    version == 0x00 ||
    version == 0xFF
  ) {

    rfidOK = false;

    Serial.println(
      "RFID initialization FAILED."
    );

  } else {

    rfidOK = true;

    Serial.print(
      "RFID firmware: 0x"
    );

    Serial.println(
      version,
      HEX
    );

    Serial.println(
      "RFID initialized."
    );
  }

  // ==========================================================
  // WI-FI
  // ==========================================================

  startAccessPoint();

  if (wifiOK) {

    setupServer();
  }

  // ==========================================================
  // FAULT CHECK
  // ==========================================================

  checkSystemFault();

  if (systemFault) {

    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "SYSTEM FAULT"
    );

    if (!mpuOK) {

      Serial.println(
        "FAULT: MPU6050"
      );
    }

    if (!rfidOK) {

      Serial.println(
        "FAULT: RFID"
      );
    }

    if (!wifiOK) {

      Serial.println(
        "FAULT: Wi-Fi AP"
      );
    }

    Serial.println(
      "================================"
    );

    startSound(
      SOUND_ERROR
    );

  } else {

    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "PROJECT HEALTH READY"
    );

    Serial.println(
      "SSID: Project Health"
    );

    Serial.println(
      "Password: 12345678"
    );

    Serial.println(
      "Dashboard: 192.168.4.1"
    );

    Serial.println(
      "================================"
    );
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // Captive portal
  if (wifiOK) {

    dnsServer.processNextRequest();

    server.handleClient();
  }

  // Buzzer
  updateSound();

  // Boot LED
  updateBootLED();

  // MPU
  if (mpuOK) {

    updateMotion();
  }

  // Fault monitoring
  checkSystemFault();

  // Fault LED has priority
  if (systemFault) {

    bootSequenceActive = false;

    updateFaultLED();

  } else {

    if (
      sessionState == RUNNING
    ) {

      digitalWrite(
        LED_PIN,
        HIGH
      );

    } else if (
      sessionState != COUNTDOWN &&
      bootSequenceActive == false
    ) {

      digitalWrite(
        LED_PIN,
        LOW
      );
    }
  }

  // RFID
  if (
    sessionState == READY &&
    !systemFault
  ) {

    handleRFID();
  }

  // Countdown
  updateCountdown();

  // Complete
  if (
    sessionState == COMPLETE
  ) {

    completeSession();
  }

  // Activity graph
  updateHistory();

  delay(2);
}