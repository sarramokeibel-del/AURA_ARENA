// ============================================================
//  ROBOT - Manual (PS4) + Autonomous (HC-SR04 + VL53L1X ToF)
//  X = Auto  |  O = Manual
//  Libraries: Bluepad32 | Adafruit VL53L1X | Adafruit BusIO
//  HC-SR04: TRIG=25  ECHO=34 (voltage divider 1k+2.2k to GND)
//  ToF L: SDA=26 SCL=27 XSHUT=33
//  ToF R: SDA=26 SCL=27 XSHUT=32
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

// ============================================================
//  *** TUNE ZONE ***
// ============================================================
#define SPEED_MANUAL        200
#define SPEED_AUTO          80
#define DIST_STOP            15   // cm: رجوع فوري
#define DIST_WARN            28   // cm: ابدأ استدارة
#define DIST_CLEAR           25   // cm: الطريق واضح بعد الاستدارة
#define WALL_THRESHOLD       20   // cm: ابدأ تصحيح جانبي
#define DEAD_ZONE             3   // cm: فرق مقبول بين L و R
#define CORRECTION_STRENGTH 0.6f  // قوة التصحيح (0.0 - 1.0)
#define TURN_MS             800   // ms: مدة الاستدارة
#define REVERSE_MS          350   // ms: مدة الرجوع
// ============================================================

// -- PS4 Buttons
// لو X و O مش شغالين، جرب تبدل القيمتين ببعض
#define BTN_AUTO   0x0002   // X  → Auto
#define BTN_MANUAL 0x0001   // O  → Manual

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

// *** تعديل: -1,-1 عشان الـ library متتحكمش في أي pin ***
Adafruit_VL53L1X tofLeft(-1, -1);
Adafruit_VL53L1X tofRight(-1, -1);
bool tofLeftOK  = false;
bool tofRightOK = false;

// -- FSM: 0=FWD  1=REV  2=TURN_LEFT  3=TURN_RIGHT
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

// ============================================================
//  Proportional Wall Correction
// ============================================================
void driveWithCorrection() {
  bool leftValid  = (sensorLeft  < 400.0f);
  bool rightValid = (sensorRight < 400.0f);

  // اتجاه الحركة للأمام دايماً
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);

  if (!leftValid && !rightValid) {
    setSpeed(currentSpeed);
    return;
  }

  if (!leftValid && rightValid) {
    if (sensorRight < WALL_THRESHOLD) {
      ledcWrite(0, currentSpeed);
      ledcWrite(1, (uint8_t)(currentSpeed * CORRECTION_STRENGTH));
    } else {
      setSpeed(currentSpeed);
    }
    return;
  }

  if (leftValid && !rightValid) {
    if (sensorLeft < WALL_THRESHOLD) {
      ledcWrite(0, (uint8_t)(currentSpeed * CORRECTION_STRENGTH));
      ledcWrite(1, currentSpeed);
    } else {
      setSpeed(currentSpeed);
    }
    return;
  }

  // جدارين من الجانبين → تمركز في النص
  float error = sensorRight - sensorLeft;  // موجب = أقرب لليمين

  if (abs(error) <= DEAD_ZONE) {
    setSpeed(currentSpeed);
  }
  else if (error > 0) {
    float factor = constrain(1.0f - (error / 30.0f), CORRECTION_STRENGTH, 1.0f);
    ledcWrite(0, currentSpeed);
    ledcWrite(1, (uint8_t)(currentSpeed * factor));
  }
  else {
    float factor = constrain(1.0f - (abs(error) / 30.0f), CORRECTION_STRENGTH, 1.0f);
    ledcWrite(0, (uint8_t)(currentSpeed * factor));
    ledcWrite(1, currentSpeed);
  }
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
//  Auto FSM
// ============================================================
void runAuto() {
  bool done = (millis() - fsmTimer >= fsmDuration);

  switch (fsmState) {

    case 0:
      driveWithCorrection();
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
      break;

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

    case 2:
      pivotLeft();
      if (done) {
        stopMotors(); delay(100);
        setSpeed(currentSpeed);
        if (sensorFront > DIST_CLEAR) {
          fsmState = 0;
        } else {
          fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      break;

    case 3:
      pivotRight();
      if (done) {
        stopMotors(); delay(100);
        setSpeed(currentSpeed);
        if (sensorFront > DIST_CLEAR) {
          fsmState = 0;
        } else {
          fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
        }
      }
      break;
  }

  const char* names[] = {"FWD","REV","TURN_L","TURN_R"};
  Serial.printf("[AUTO] F=%.0f L=%.0f R=%.0f ERR=%.0f | %s\n",
    sensorFront, sensorLeft, sensorRight,
    sensorLeft - sensorRight,
    names[fsmState]);
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
//  ToF Init - XSHUT يدوي بالكامل
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
  Serial.printf("MANUAL=%d AUTO=%d WALL=%dcm DEAD=%dcm STRENGTH=%.1f\n",
                SPEED_MANUAL, SPEED_AUTO, WALL_THRESHOLD, DEAD_ZONE, CORRECTION_STRENGTH);

  // *** أولاً: LEDC ***
  ledcSetup(0, 1000, 8);
  ledcAttachPin(EN_RIGHT, 0);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(EN_LEFT, 1);

  // *** تانياً: Motor pins ***
  int motorPins[] = {R1_IN1,R1_IN2,R2_IN1,R2_IN2,
                     L1_IN1,L1_IN2,L2_IN1,L2_IN2};
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  stopMotors();

  // *** تالثاً: Ultrasonic ***
  pinMode(FRONT_TRIG, OUTPUT);
  digitalWrite(FRONT_TRIG, LOW);
  pinMode(FRONT_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_ECHO), echoISR, CHANGE);
  Serial.println("[US]  ISR ready on pin 34");

  // *** رابعاً: I2C و ToF ***
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  initToF();

  // *** خامساً: Bluepad32 ***
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.printf("[INFO] Ready! BTN_AUTO=0x%04X BTN_MANUAL=0x%04X\n",
                BTN_AUTO, BTN_MANUAL);
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

    // *** Debug: طباعة قيمة الزرار لما تضغط أي حاجة ***
    if (pressed) {
      Serial.printf("[BTN] pressed=0x%04X\n", pressed);
    }

    if (pressed & BTN_AUTO) {
      robotMode    = 1;
      currentSpeed = SPEED_AUTO;
      fsmState     = 0;
      fsmTimer     = millis();
      fsmDuration  = 0;
      stopMotors();
      Serial.printf("[MODE] AUTO - Speed=%d\n", currentSpeed);
    }
    if (pressed & BTN_MANUAL) {
      robotMode    = 0;
      currentSpeed = SPEED_MANUAL;
      stopMotors();
      Serial.printf("[MODE] MANUAL - Speed=%d\n", currentSpeed);
    }

    prevBtns = btns;
    if (robotMode == 0) runManual(ctl);
  }

  if (robotMode == 1) runAuto();
}
