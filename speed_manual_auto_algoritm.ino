// ============================================================
//  ROBOT - Manual (PS4) + Autonomous (HC-SR04 + VL53L1X ToF)
//  X = Auto  |  O = Manual
//  Libraries: Bluepad32 | Adafruit VL53L1X | Adafruit BusIO
//  HC-SR04: TRIG=25  ECHO=34 (voltage divider 1k+2.2k to GND)
//  ToF L: SDA=26 SCL=27 XSHUT=32
//  ToF R: SDA=26 SCL=27 XSHUT=33
// ============================================================

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

// -- Motor Pins
#define R1_IN1 23
#define R1_IN2 22
#define R2_IN1 21
#define R2_IN2 19
#define L1_IN1 18
#define L1_IN2 5
#define L2_IN1 17
#define L2_IN2 16

// -- Enable Pins
#define EN_RIGHT  4
#define EN_LEFT   15

// *** سرعة المنيوال والأوتوماتيك منفصلين - عدّلهم هنا ***
#define SPEED_MANUAL  200   // سرعة المنيوال  (0-255)
#define SPEED_AUTO    120   // سرعة الأوتوماتيك (0-255)

// المتغير الفعلي اللي بيتبعت للموتورات - بيتغير حسب المود
uint8_t currentSpeed = SPEED_MANUAL;

void setSpeed(uint8_t spd) {
  ledcWrite(0, spd);
  ledcWrite(1, spd);
}

// -- Ultrasonic ISR
#define FRONT_TRIG 25
#define FRONT_ECHO 34

volatile unsigned long usStart    = 0;
volatile unsigned long usDuration = 0;
volatile bool          usReady    = false;

void IRAM_ATTR echoISR() {
  if (digitalRead(FRONT_ECHO)) {
    usStart = micros();
  } else {
    unsigned long dur = micros() - usStart;
    if (dur > 150 && dur < 23200) {
      usDuration = dur;
      usReady    = true;
    }
  }
}

unsigned long lastTrigMs = 0;
void triggerUS() {
  if (millis() - lastTrigMs < 30) return;
  lastTrigMs = millis();
  digitalWrite(FRONT_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(FRONT_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(FRONT_TRIG, LOW);
}

// -- VL53L1X
#define I2C_SDA     26
#define I2C_SCL     27
#define TOF_XSHUT_L 32
#define TOF_XSHUT_R 33
#define TOF_ADDR_L  0x30
#define TOF_ADDR_R  0x31

Adafruit_VL53L1X tofLeft  = Adafruit_VL53L1X(TOF_XSHUT_L);
Adafruit_VL53L1X tofRight = Adafruit_VL53L1X(TOF_XSHUT_R);
bool tofLeftOK  = false;
bool tofRightOK = false;

// ============================================================
//  Thresholds - Tune Here
// ============================================================
#define DIST_STOP        15   // cm: عائق قريب جداً → رجوع
#define DIST_WARN        28   // cm: عائق أمامي → استدارة
#define DIST_CLEAR       25   // cm: تأكيد الطريق واضح بعد الاستدارة
#define WALL_THRESHOLD   15   // cm: قريب من جدار جانبي → تصحيح مسار
#define TURN_MS         800   // ms: مدة الاستدارة 90 درجة
#define REVERSE_MS      350   // ms: مدة الرجوع
#define CORRECT_MS      200   // ms: مدة تصحيح المسار الجانبي

// ============================================================
//  FSM States
//  0=FWD  1=REV  2=TURN_LEFT  3=TURN_RIGHT
//  4=CORRECT_LEFT  5=CORRECT_RIGHT
// ============================================================
uint8_t robotMode = 0;

uint8_t       fsmState    = 0;
unsigned long fsmTimer    = 0;
unsigned long fsmDuration = 0;

// -- Sensors
float sensorFront = 999;
float sensorLeft  = 999;
float sensorRight = 999;

// -- ToF round-robin
uint8_t       tofIdx    = 0;
unsigned long lastTofMs = 0;

// -- Controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
uint16_t prevBtns = 0;

// ============================================================
//  Motors
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

// *** دالتين جديدتين للتصحيح الناعم (بس في الأوتوماتيك) ***
void correctLeft() {
  // العجلة اليمين أسرع → الروبوت ينحرف يسار بدون توقف
  ledcWrite(0, currentSpeed);
  ledcWrite(1, currentSpeed / 2);
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}
void correctRight() {
  // العجلة اليسار أسرع → الروبوت ينحرف يمين بدون توقف
  ledcWrite(0, currentSpeed / 2);
  ledcWrite(1, currentSpeed);
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}

// ============================================================
//  Sensors - Non-blocking
// ============================================================
void updateSensors() {
  triggerUS();
  if (usReady) {
    usReady = false;
    float cm = (usDuration * 0.0343f) / 2.0f;
    sensorFront = (cm < 2.0f || cm > 400.0f) ? 400.0f : cm;
  }

  if (millis() - lastTofMs < 60) return;
  lastTofMs = millis();

  if (tofIdx == 0 && tofLeftOK) {
    if (tofLeft.dataReady()) {
      int16_t mm = tofLeft.distance();
      tofLeft.clearInterrupt();
      tofLeft.startRanging();
      if (mm > 0) sensorLeft = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  if (tofIdx == 1 && tofRightOK) {
    if (tofRight.dataReady()) {
      int16_t mm = tofRight.distance();
      tofRight.clearInterrupt();
      tofRight.startRanging();
      if (mm > 0) sensorRight = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  tofIdx = (tofIdx + 1) % 2;
}

// ============================================================
//  Auto FSM - Wall Following + Obstacle Avoidance
// ============================================================
void runAuto() {
  bool done = (millis() - fsmTimer >= fsmDuration);

  switch (fsmState) {

    // State 0: FWD
    case 0:
      moveForward();
      if (sensorFront <= DIST_STOP) {
        stopMotors(); delay(80);
        fsmState = 1; fsmTimer = millis(); fsmDuration = REVERSE_MS;
      }
      else if (sensorFront <= DIST_WARN) {
        stopMotors(); delay(80);
        if (sensorLeft >= sensorRight) {
          fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
        } else {
          fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      else if (sensorRight < WALL_THRESHOLD) {
        fsmState = 4; fsmTimer = millis(); fsmDuration = CORRECT_MS;
      }
      else if (sensorLeft < WALL_THRESHOLD) {
        fsmState = 5; fsmTimer = millis(); fsmDuration = CORRECT_MS;
      }
      break;

    // State 1: REV
    case 1:
      moveBackward();
      if (done) {
        stopMotors(); delay(80);
        if (sensorLeft >= sensorRight) {
          fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
        } else {
          fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      break;

    // State 2: TURN_LEFT
    case 2:
      pivotLeft();
      if (done) {
        stopMotors(); delay(100);
        if (sensorFront > DIST_CLEAR) {
          fsmState = 0;
        } else {
          fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      break;

    // State 3: TURN_RIGHT
    case 3:
      pivotRight();
      if (done) {
        stopMotors(); delay(100);
        if (sensorFront > DIST_CLEAR) {
          fsmState = 0;
        } else {
          fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      break;

    // State 4: CORRECT_LEFT (جدار أيمن قريب)
    case 4:
      correctLeft();
      if (done) {
        setSpeed(currentSpeed);   // رجع السرعة متساوية
        fsmState = 0;
      }
      break;

    // State 5: CORRECT_RIGHT (جدار أيسر قريب)
    case 5:
      correctRight();
      if (done) {
        setSpeed(currentSpeed);   // رجع السرعة متساوية
        fsmState = 0;
      }
      break;
  }

  const char* names[] = {"FWD","REV","TURN_L","TURN_R","CORR_L","CORR_R"};
  Serial.printf("[AUTO] F=%.0f L=%.0f R=%.0f | %s\n",
    sensorFront, sensorLeft, sensorRight, names[fsmState]);
}

// ============================================================
//  Manual Mode
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
//  ToF Init
// ============================================================
void initToF() {
  Serial.println("[TOF] Init...");
  pinMode(TOF_XSHUT_L, OUTPUT);
  pinMode(TOF_XSHUT_R, OUTPUT);
  digitalWrite(TOF_XSHUT_L, LOW);
  digitalWrite(TOF_XSHUT_R, LOW);
  delay(100);

  digitalWrite(TOF_XSHUT_L, HIGH); delay(200);
  if (tofLeft.begin(TOF_ADDR_L, &Wire)) {
    tofLeft.startRanging();
    tofLeft.setTimingBudget(50);
    tofLeftOK = true;
    Serial.printf("[TOF] Left  OK 0x%02X\n", TOF_ADDR_L);
  } else {
    Serial.println("[TOF] Left  FAIL");
  }

  digitalWrite(TOF_XSHUT_R, HIGH); delay(200);
  if (tofRight.begin(TOF_ADDR_R, &Wire)) {
    tofRight.startRanging();
    tofRight.setTimingBudget(50);
    tofRightOK = true;
    Serial.printf("[TOF] Right OK 0x%02X\n", TOF_ADDR_R);
  } else {
    Serial.println("[TOF] Right FAIL");
  }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(921600);
  delay(300);
  Serial.println("=== ROBOT STARTED ===");
  Serial.printf("MANUAL_SPEED=%d AUTO_SPEED=%d\n", SPEED_MANUAL, SPEED_AUTO);
  Serial.printf("STOP=%dcm WARN=%dcm WALL=%dcm TURN=%dms\n",
                DIST_STOP, DIST_WARN, WALL_THRESHOLD, TURN_MS);

  int motorPins[] = {R1_IN1,R1_IN2,R2_IN1,R2_IN2,
                     L1_IN1,L1_IN2,L2_IN1,L2_IN2};
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  stopMotors();

  ledcSetup(0, 1000, 8);
  ledcAttachPin(EN_RIGHT, 0);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(EN_LEFT, 1);

  pinMode(FRONT_TRIG, OUTPUT);
  digitalWrite(FRONT_TRIG, LOW);
  pinMode(FRONT_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_ECHO), echoISR, CHANGE);
  Serial.println("[US]  ISR ready on pin 34");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  initToF();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("[INFO] Ready! X=Auto | O=Manual");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  BP32.update();

  if (robotMode == 1) updateSensors();

  for (auto ctl : myControllers) {
    if (!ctl || !ctl->isConnected()) continue;

    uint16_t btns    = ctl->buttons();
    uint16_t pressed = btns & ~prevBtns;

    if (pressed & BUTTON_A) {
      robotMode     = 1;
      currentSpeed  = SPEED_AUTO;    // *** سرعة الأوتوماتيك ***
      fsmState      = 0;
      fsmTimer      = millis();
      fsmDuration   = 0;
      stopMotors();
      Serial.printf("[MODE] AUTO - Speed=%d\n", currentSpeed);
    }
    if (pressed & BUTTON_B) {
      robotMode     = 0;
      currentSpeed  = SPEED_MANUAL;  // *** سرعة المنيوال ***
      stopMotors();
      Serial.printf("[MODE] MANUAL - Speed=%d\n", currentSpeed);
    }

    prevBtns = btns;
    if (robotMode == 0) runManual(ctl);
  }

  if (robotMode == 1) runAuto();
}
