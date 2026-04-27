#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

#define SDA_PIN 13
#define SCL_PIN 14

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ      50
#define PCA_OSCILLATOR  27000000

#define CH_SHOULDER    0
#define CH_ELBOW       1
#define CH_WRIST_PITCH 2
#define CH_GRIPPER     3
#define NUM_SERVOS     4

#define BLACK_MIN_US  500
#define BLACK_MAX_US  2500
#define BLUE_MIN_US   1000
#define BLUE_MAX_US   2000

#define TORQUE_HOLD_MS    800
#define GRIPPER_HOLD_MS  1500

#define DEADZONE        30
#define AXIS_MAX        512.0f
#define SPEED_NORMAL    1.2f
#define SPEED_PRECISION 0.35f

struct ServoConfig {
  uint8_t       channel;
  float         minAngle;
  float         maxAngle;
  int           minUs;
  int           maxUs;
  float         currentAngle;
  unsigned long lastMoveTime;
  bool          torqueActive;
};

ServoConfig servos[NUM_SERVOS] = {
  { CH_SHOULDER,    10, 170, BLACK_MIN_US, BLACK_MAX_US, 90.0f, 0, true },
  { CH_ELBOW,       10, 170, BLACK_MIN_US, BLACK_MAX_US, 90.0f, 0, true },
  { CH_WRIST_PITCH, 10, 170, BLUE_MIN_US,  BLUE_MAX_US,  90.0f, 0, true },
  { CH_GRIPPER,      5, 175, BLUE_MIN_US,  BLUE_MAX_US,  90.0f, 0, true },
};

GamepadPtr myGamepad    = nullptr;
bool       precisionMode   = false;
float      speedMultiplier = 1.0f;
float      baseSpeed       = SPEED_NORMAL;

unsigned long lastTriangle = 0;
unsigned long lastSquare   = 0;
unsigned long lastR1       = 0;
unsigned long lastL1       = 0;
#define DEBOUNCE_MS 250

int angleToPWM(float angle, int minUs, int maxUs) {
  float us = minUs + (constrain(angle, 0.0f, 180.0f) / 180.0f) * (maxUs - minUs);
  return (int)(us * 4096.0f / 20000.0f);
}

void disableTorque(uint8_t idx) {
  if (servos[idx].torqueActive) {
    pca.setPWM(servos[idx].channel, 0, 0);
    servos[idx].torqueActive = false;
  }
}

void setServoAngle(uint8_t idx, float angle) {
  ServoConfig &s = servos[idx];
  s.currentAngle = constrain(angle, s.minAngle, s.maxAngle);
  s.lastMoveTime = millis();
  s.torqueActive = true;
  pca.setPWM(s.channel, 0, angleToPWM(s.currentAngle, s.minUs, s.maxUs));
}

void moveServo(uint8_t idx, float delta) {
  setServoAngle(idx, servos[idx].currentAngle + delta);
}

void updateTorqueManagement() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].torqueActive) continue;
    unsigned long holdTime = (i == CH_GRIPPER) ? GRIPPER_HOLD_MS : TORQUE_HOLD_MS;
    if (now - servos[i].lastMoveTime > holdTime) {
      disableTorque(i);
      Serial.printf("💤 Ch%d torque OFF (%.1f°)\n", i, servos[i].currentAngle);
    }
  }
}

float applyDeadzone(int raw) {
  if (abs(raw) < DEADZONE) return 0.0f;
  float sign = (raw > 0) ? 1.0f : -1.0f;
  return sign * ((float)(abs(raw) - DEADZONE) / (AXIS_MAX - DEADZONE));
}

void resetAllServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) setServoAngle(i, 90.0f);
  Serial.println("🔄 Reset → 90°");
}

void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
  Serial.println("✅ PS4 Connected!");
  gp->setRumble(0x50, 0x50);
  delay(150);
  gp->setRumble(0, 0);
}

void onDisconnectedGamepad(GamepadPtr gp) {
  myGamepad = nullptr;
  Serial.println("❌ PS4 Disconnected!");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Robot Arm Init ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pca.begin();
  pca.setOscillatorFrequency(PCA_OSCILLATOR);
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);

  resetAllServos();
  Serial.println("✅ Servos ready at 90°");

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  Serial.println("✅ Bluepad32 ready");
}

void loop() {
  BP32.update();

  updateTorqueManagement();

  if (!myGamepad || !myGamepad->isConnected()) {
    delay(10);
    return;
  }

  unsigned long now = millis();

  // △ Triangle → Reset
  if (myGamepad->y() && (now - lastTriangle > DEBOUNCE_MS)) {
    lastTriangle = now;
    resetAllServos();
    myGamepad->setRumble(0x30, 0x30);
    delay(100);
    myGamepad->setRumble(0, 0);
  }

  // □ Square → Precision Mode
  if (myGamepad->x() && (now - lastSquare > DEBOUNCE_MS)) {
    lastSquare    = now;
    precisionMode = !precisionMode;
    baseSpeed     = precisionMode ? SPEED_PRECISION : SPEED_NORMAL;
    Serial.printf("⚙️ %s Mode\n", precisionMode ? "PRECISION" : "NORMAL");
    myGamepad->setRumble(precisionMode ? 0x20 : 0x60, 0x00);
    delay(200);
    myGamepad->setRumble(0, 0);
  }

  // R1 → رفع السرعة
  if (myGamepad->r1() && (now - lastR1 > DEBOUNCE_MS)) {
    lastR1 = now;
    speedMultiplier = constrain(speedMultiplier + 0.25f, 0.25f, 2.0f);
    Serial.printf("⚡ Speed: %.2fx\n", speedMultiplier);
  }

  // L1 → تقليل السرعة
  if (myGamepad->l1() && (now - lastL1 > DEBOUNCE_MS)) {
    lastL1 = now;
    speedMultiplier = constrain(speedMultiplier - 0.25f, 0.25f, 2.0f);
    Serial.printf("🐢 Speed: %.2fx\n", speedMultiplier);
  }

  float speed = baseSpeed * speedMultiplier;

  float fLY = applyDeadzone(myGamepad->axisY());
  float fRY = applyDeadzone(myGamepad->axisRY());
  float fRX = applyDeadzone(myGamepad->axisRX());
  float fL2 = myGamepad->brake()    / 1023.0f;
  float fR2 = myGamepad->throttle() / 1023.0f;

  // Shoulder   ← Left  Stick Y
  if (fLY != 0.0f) moveServo(CH_SHOULDER,    -fLY * speed);

  // Elbow      ← Right Stick Y
  if (fRY != 0.0f) moveServo(CH_ELBOW,       -fRY * speed);

  // WristPitch ← Right Stick X
  if (fRX != 0.0f) moveServo(CH_WRIST_PITCH,  fRX * speed);

  // Gripper    ← L2 قفل / R2 فتح
  float gripDelta = (fR2 - fL2) * speed;
  if (abs(gripDelta) > 0.015f) moveServo(CH_GRIPPER, gripDelta);

  // Debug كل 500ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.printf(
      "[%s x%.2f] SHO:%.1f° ELB:%.1f° WP:%.1f° GRP:%.1f°\n",
      precisionMode ? "PREC" : "NORM", speedMultiplier,
      servos[CH_SHOULDER].currentAngle,
      servos[CH_ELBOW].currentAngle,
      servos[CH_WRIST_PITCH].currentAngle,
      servos[CH_GRIPPER].currentAngle
    );
  }

  delay(20);
}