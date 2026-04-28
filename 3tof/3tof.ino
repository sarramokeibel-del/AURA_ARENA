// ============================================================
//  ROBOT (Motors + Auto/Manual) + ARM (4 Servos via PCA9685)
//  -- Robot: X=Auto | O=Manual | DPAD=حركة
//  -- Arm:   Sticks=سيرفوهات | L2/R2=Gripper | △=Reset | □=Precision | R1/L1=Speed
//  -- الذراع شغالة في المانيوال بس
//  -- Wire  (26,27) → ToF Sensors (Front + Left + Right)
//  -- Wire1 (13,14) → PCA9685 Servo Driver
// ============================================================

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================
//  ROBOT - Motor Pins
// ============================================================
#define R1_IN1 23
#define R1_IN2 22
#define R2_IN1 21
#define R2_IN2 19
#define L1_IN1 18
#define L1_IN2 5
#define L2_IN1 17
#define L2_IN2 16
#define EN_RIGHT  4
#define EN_LEFT   15

// ============================================================
//  ROBOT - Tune Zone
// ============================================================
#define SPEED_MANUAL        200
#define SPEED_AUTO          100
#define DIST_STOP            15
#define DIST_WARN            28
#define DIST_CLEAR           25
#define WALL_THRESHOLD       20
#define DEAD_ZONE             3
#define CORRECTION_STRENGTH  0.6f
#define TURN_MS             800
#define REVERSE_MS          350

// ============================================================
//  ROBOT - Buttons
// ============================================================
#define BTN_AUTO   0x0002   // X  → Auto
#define BTN_MANUAL 0x0001   // O  → Manual

uint8_t  currentSpeed = SPEED_MANUAL;
uint8_t  robotMode    = 0;   // 0=Manual  1=Auto
uint16_t prevBtns     = 0;

void setSpeed(uint8_t spd) {
  ledcWrite(0, spd);
  ledcWrite(1, spd);
}

// ============================================================
//  ROBOT - VL53L1X (Wire على 26,27) — Front + Left + Right
// ============================================================
#define I2C_SDA      26
#define I2C_SCL      27
#define TOF_XSHUT_F  25   // ← front  (كان TRIG الالتراسونيك)
#define TOF_XSHUT_L  32
#define TOF_XSHUT_R  33
#define TOF_ADDR_F   0x32
#define TOF_ADDR_L   0x30
#define TOF_ADDR_R   0x31

Adafruit_VL53L1X tofFront(-1, -1);
Adafruit_VL53L1X tofLeft (-1, -1);
Adafruit_VL53L1X tofRight(-1, -1);
bool tofFrontOK = false;
bool tofLeftOK  = false;
bool tofRightOK = false;

float sensorFront = 999;
float sensorLeft  = 999;
float sensorRight = 999;

uint8_t       tofIdx    = 0;
unsigned long lastTofMs = 0;

// ============================================================
//  ROBOT - FSM
// ============================================================
uint8_t       fsmState    = 0;
unsigned long fsmTimer    = 0;
unsigned long fsmDuration = 0;

// ============================================================
//  ARM - PCA9685 على Wire1 (13,14)
// ============================================================
#define ARM_SDA_PIN 13
#define ARM_SCL_PIN 14

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire1);
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

#define ARM_DEADZONE     30
#define AXIS_MAX         512.0f
#define SPEED_NORMAL     1.2f
#define SPEED_PRECISION  0.35f
#define ARM_DEBOUNCE_MS  250

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

bool  precisionMode   = false;
float armSpeed        = SPEED_NORMAL;
float speedMultiplier = 1.0f;

unsigned long lastTriangle = 0;
unsigned long lastSquare   = 0;
unsigned long lastR1btn    = 0;
unsigned long lastL1btn    = 0;

// ============================================================
//  Bluepad32
// ============================================================
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// ============================================================
//  ROBOT - Motors
// ============================================================
void stopMotors() {
  setSpeed(0);
  digitalWrite(R1_IN1,LOW); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,LOW); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,LOW); digitalWrite(L2_IN2,LOW);
}
void moveForward() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}
void moveBackward() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,LOW); digitalWrite(R1_IN2,HIGH);
  digitalWrite(R2_IN1,LOW); digitalWrite(R2_IN2,HIGH);
  digitalWrite(L1_IN1,LOW); digitalWrite(L1_IN2,HIGH);
  digitalWrite(L2_IN1,LOW); digitalWrite(L2_IN2,HIGH);
}
void moveRight() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,LOW);  digitalWrite(R1_IN2,HIGH);
  digitalWrite(R2_IN1,LOW);  digitalWrite(R2_IN2,HIGH);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}
void moveLeft() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW);  digitalWrite(L1_IN2,HIGH);
  digitalWrite(L2_IN1,LOW);  digitalWrite(L2_IN2,HIGH);
}
void pivotLeft() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW);  digitalWrite(L1_IN2,HIGH);
  digitalWrite(L2_IN1,LOW);  digitalWrite(L2_IN2,HIGH);
}
void pivotRight() {
  setSpeed(currentSpeed);
  digitalWrite(R1_IN1,LOW);  digitalWrite(R1_IN2,HIGH);
  digitalWrite(R2_IN1,LOW);  digitalWrite(R2_IN2,HIGH);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}

// ============================================================
//  ROBOT - Wall Correction
// ============================================================
void driveWithCorrection() {
  bool leftValid  = (sensorLeft  < 400.0f);
  bool rightValid = (sensorRight < 400.0f);

  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);

  if (!leftValid && !rightValid) { setSpeed(currentSpeed); return; }
  if (!leftValid && rightValid) {
    if (sensorRight < WALL_THRESHOLD) { ledcWrite(0, currentSpeed); ledcWrite(1, (uint8_t)(currentSpeed * CORRECTION_STRENGTH)); }
    else setSpeed(currentSpeed);
    return;
  }
  if (leftValid && !rightValid) {
    if (sensorLeft < WALL_THRESHOLD) { ledcWrite(0, (uint8_t)(currentSpeed * CORRECTION_STRENGTH)); ledcWrite(1, currentSpeed); }
    else setSpeed(currentSpeed);
    return;
  }
  float error = sensorRight - sensorLeft;
  if (abs(error) <= DEAD_ZONE) {
    setSpeed(currentSpeed);
  } else if (error > 0) {
    float f = constrain(1.0f - (error / 30.0f), CORRECTION_STRENGTH, 1.0f);
    ledcWrite(0, currentSpeed); ledcWrite(1, (uint8_t)(currentSpeed * f));
  } else {
    float f = constrain(1.0f - (abs(error) / 30.0f), CORRECTION_STRENGTH, 1.0f);
    ledcWrite(0, (uint8_t)(currentSpeed * f)); ledcWrite(1, currentSpeed);
  }
}

// ============================================================
//  ROBOT - Sensors (Non-blocking) — 3 ToF sensors
// ============================================================
void updateSensors() {
  if (millis() - lastTofMs < 60) return;
  lastTofMs = millis();

  // idx 0 → Front, 1 → Left, 2 → Right
  if (tofIdx == 0 && tofFrontOK) {
    if (tofFront.dataReady()) {
      int16_t mm = tofFront.distance();
      tofFront.clearInterrupt(); tofFront.startRanging();
      if (mm > 0) sensorFront = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  if (tofIdx == 1 && tofLeftOK) {
    if (tofLeft.dataReady()) {
      int16_t mm = tofLeft.distance();
      tofLeft.clearInterrupt(); tofLeft.startRanging();
      if (mm > 0) sensorLeft = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  if (tofIdx == 2 && tofRightOK) {
    if (tofRight.dataReady()) {
      int16_t mm = tofRight.distance();
      tofRight.clearInterrupt(); tofRight.startRanging();
      if (mm > 0) sensorRight = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  tofIdx = (tofIdx + 1) % 3;  // ← دورة على 3 بدل 2
}

// ============================================================
//  ROBOT - Auto FSM
// ============================================================
void runAuto() {
  bool done = (millis() - fsmTimer >= fsmDuration);
  switch (fsmState) {
    case 0:
      driveWithCorrection();
      if (sensorFront <= DIST_STOP) {
        stopMotors(); delay(80);
        fsmState = 1; fsmTimer = millis(); fsmDuration = REVERSE_MS;
      } else if (sensorFront <= DIST_WARN) {
        stopMotors(); delay(80);
        if (sensorLeft >= sensorRight) { fsmState = 2; } else { fsmState = 3; }
        fsmTimer = millis(); fsmDuration = TURN_MS;
      }
      break;
    case 1:
      moveBackward();
      if (done) {
        stopMotors(); delay(80);
        if (sensorLeft >= sensorRight) { fsmState = 2; } else { fsmState = 3; }
        fsmTimer = millis(); fsmDuration = TURN_MS;
      }
      break;
    case 2:
      pivotLeft();
      if (done) {
        stopMotors(); delay(100); setSpeed(currentSpeed);
        fsmState = (sensorFront > DIST_CLEAR) ? 0 : 3;
        fsmTimer = millis(); fsmDuration = TURN_MS;
      }
      break;
    case 3:
      pivotRight();
      if (done) {
        stopMotors(); delay(100); setSpeed(currentSpeed);
        fsmState = (sensorFront > DIST_CLEAR) ? 0 : 2;
        fsmTimer = millis(); fsmDuration = TURN_MS;
      }
      break;
  }
  const char* names[] = {"FWD","REV","TURN_L","TURN_R"};
  Serial.printf("[AUTO] F=%.0f L=%.0f R=%.0f | %s\n",
    sensorFront, sensorLeft, sensorRight, names[fsmState]);
}

// ============================================================
//  ROBOT - Manual
// ============================================================
void runManual(ControllerPtr ctl) {
  uint8_t dpad = ctl->dpad();
  if      (dpad == DPAD_UP)    moveForward();
  else if (dpad == DPAD_DOWN)  moveBackward();
  else if (dpad == DPAD_RIGHT) moveRight();
  else if (dpad == DPAD_LEFT)  moveLeft();
  else                         stopMotors();
}

// ============================================================
//  ARM - Helper Functions
// ============================================================
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

void resetAllServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) setServoAngle(i, 90.0f);
  Serial.println("🔄 Arm Reset → 90°");
}

void updateTorqueManagement() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!servos[i].torqueActive) continue;
    unsigned long holdTime = (i == CH_GRIPPER) ? GRIPPER_HOLD_MS : TORQUE_HOLD_MS;
    if (now - servos[i].lastMoveTime > holdTime) {
      disableTorque(i);
      Serial.printf("💤 Ch%d OFF (%.1f°)\n", i, servos[i].currentAngle);
    }
  }
}

float applyArmDeadzone(int raw) {
  if (abs(raw) < ARM_DEADZONE) return 0.0f;
  float sign = (raw > 0) ? 1.0f : -1.0f;
  return sign * ((float)(abs(raw) - ARM_DEADZONE) / (AXIS_MAX - ARM_DEADZONE));
}

// ============================================================
//  ARM - Control
// ============================================================
void runArmControl(ControllerPtr ctl) {
  unsigned long now = millis();

  if (ctl->y() && (now - lastTriangle > ARM_DEBOUNCE_MS)) {
    lastTriangle = now;
    resetAllServos();
    ctl->setRumble(0x30, 0x30);
    delay(100);
    ctl->setRumble(0, 0);
  }

  if (ctl->x() && (now - lastSquare > ARM_DEBOUNCE_MS)) {
    lastSquare    = now;
    precisionMode = !precisionMode;
    armSpeed      = precisionMode ? SPEED_PRECISION : SPEED_NORMAL;
    Serial.printf("⚙️ Arm: %s Mode\n", precisionMode ? "PRECISION" : "NORMAL");
    ctl->setRumble(precisionMode ? 0x20 : 0x60, 0x00);
    delay(200);
    ctl->setRumble(0, 0);
  }

  if (ctl->r1() && (now - lastR1btn > ARM_DEBOUNCE_MS)) {
    lastR1btn       = now;
    speedMultiplier = constrain(speedMultiplier + 0.25f, 0.25f, 2.0f);
    Serial.printf("⚡ Arm Speed: %.2fx\n", speedMultiplier);
  }

  if (ctl->l1() && (now - lastL1btn > ARM_DEBOUNCE_MS)) {
    lastL1btn       = now;
    speedMultiplier = constrain(speedMultiplier - 0.25f, 0.25f, 2.0f);
    Serial.printf("🐢 Arm Speed: %.2fx\n", speedMultiplier);
  }

  float speed = armSpeed * speedMultiplier;

  float fLY = applyArmDeadzone(ctl->axisY());
  float fRY = applyArmDeadzone(ctl->axisRY());
  float fRX = applyArmDeadzone(ctl->axisRX());
  float fL2 = ctl->brake()    / 1023.0f;
  float fR2 = ctl->throttle() / 1023.0f;

  if (fLY != 0.0f) moveServo(CH_SHOULDER,    -fLY * speed);
  if (fRY != 0.0f) moveServo(CH_ELBOW,       -fRY * speed);
  if (fRX != 0.0f) moveServo(CH_WRIST_PITCH,  fRX * speed);

  float gripDelta = (fR2 - fL2) * speed;
  if (abs(gripDelta) > 0.015f) moveServo(CH_GRIPPER, gripDelta);
}

// ============================================================
//  Bluepad32 Callbacks
// ============================================================
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (!myControllers[i]) {
      myControllers[i] = ctl;
      Serial.printf("[BT] Controller #%d connected\n", i);
      return;
    }
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      stopMotors();
      Serial.printf("[BT] Controller #%d disconnected\n", i);
      return;
    }
  }
}

// ============================================================
//  ToF Init — 3 Sensors (Front + Left + Right)
// ============================================================
void initToF() {
  Serial.println("[TOF] Init 3 sensors...");
  pinMode(TOF_XSHUT_F, OUTPUT);
  pinMode(TOF_XSHUT_L, OUTPUT);
  pinMode(TOF_XSHUT_R, OUTPUT);

  // اطفي الكل الأول
  digitalWrite(TOF_XSHUT_F, LOW);
  digitalWrite(TOF_XSHUT_L, LOW);
  digitalWrite(TOF_XSHUT_R, LOW);
  delay(100);

  // Front أول
  digitalWrite(TOF_XSHUT_F, HIGH); delay(200);
  if (tofFront.begin(TOF_ADDR_F, &Wire)) {
    tofFront.startRanging(); tofFront.setTimingBudget(50);
    tofFrontOK = true;
    Serial.printf("[TOF] Front OK 0x%02X\n", TOF_ADDR_F);
  } else Serial.println("[TOF] Front FAIL");

  // Left
  digitalWrite(TOF_XSHUT_L, HIGH); delay(200);
  if (tofLeft.begin(TOF_ADDR_L, &Wire)) {
    tofLeft.startRanging(); tofLeft.setTimingBudget(50);
    tofLeftOK = true;
    Serial.printf("[TOF] Left  OK 0x%02X\n", TOF_ADDR_L);
  } else Serial.println("[TOF] Left  FAIL");

  // Right
  digitalWrite(TOF_XSHUT_R, HIGH); delay(200);
  if (tofRight.begin(TOF_ADDR_R, &Wire)) {
    tofRight.startRanging(); tofRight.setTimingBudget(50);
    tofRightOK = true;
    Serial.printf("[TOF] Right OK 0x%02X\n", TOF_ADDR_R);
  } else Serial.println("[TOF] Right FAIL");
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(921600);
  delay(300);
  Serial.println("=== ROBOT + ARM INIT ===");

  // LEDC للموتورات
  ledcSetup(0, 1000, 8); ledcAttachPin(EN_RIGHT, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(EN_LEFT,  1);

  // Motor Pins
  int motorPins[] = {R1_IN1,R1_IN2,R2_IN1,R2_IN2,
                     L1_IN1,L1_IN2,L2_IN1,L2_IN2};
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  stopMotors();

  // Wire → ToF (26,27) — 3 sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  initToF();

  // Wire1 → PCA9685 (13,14)
  Wire1.begin(ARM_SDA_PIN, ARM_SCL_PIN);
  Wire1.setClock(400000);
  pca.begin();
  pca.setOscillatorFrequency(PCA_OSCILLATOR);
  pca.setPWMFreq(SERVO_FREQ);
  delay(10);
  resetAllServos();
  Serial.println("[ARM] Servos ready at 90°");

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("[BT]  Ready");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  BP32.update();

  if (robotMode == 0) updateTorqueManagement();
  if (robotMode == 1) updateSensors();

  for (auto ctl : myControllers) {
    if (!ctl || !ctl->isConnected()) continue;

    uint16_t btns    = ctl->buttons();
    uint16_t pressed = btns & ~prevBtns;

    if (pressed) Serial.printf("[BTN] 0x%04X\n", pressed);

    if (pressed & BTN_AUTO) {
      robotMode = 1; currentSpeed = SPEED_AUTO;
      fsmState = 0; fsmTimer = millis(); fsmDuration = 0;
      stopMotors();
      Serial.println("[MODE] AUTO");
    }
    if (pressed & BTN_MANUAL) {
      robotMode = 0; currentSpeed = SPEED_MANUAL;
      stopMotors();
      Serial.println("[MODE] MANUAL");
    }

    prevBtns = btns;

    if (robotMode == 0) {
      runManual(ctl);
      runArmControl(ctl);
    }
  }

  if (robotMode == 1) runAuto();

  if (robotMode == 0) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      Serial.printf("[ARM][%s x%.2f] SHO:%.1f° ELB:%.1f° WP:%.1f° GRP:%.1f°\n",
        precisionMode ? "PREC" : "NORM", speedMultiplier,
        servos[0].currentAngle, servos[1].currentAngle,
        servos[2].currentAngle, servos[3].currentAngle);
    }
  }
}