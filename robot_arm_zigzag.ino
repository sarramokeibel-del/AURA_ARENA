// ============================================================
//  ROBOT + ARM - Manual (PS4) + Autonomous Zigzag
//  X = Auto  |  O = Manual
//  In Manual: D-pad = robot | Sticks+triggers = arm
//
//  Libraries: Bluepad32 | Adafruit VL53L1X | Adafruit BusIO | ESP32Servo
//
//  HC-SR04 : TRIG=25  ECHO=34 (voltage divider 1k+2.2xddddddddddddddk)
//  ToF L   : SDA=26 SCL=27 XSHUT=32
//  ToF R   : SDA=26 SCL=27 XSHUT=33
//  Servo 1 (Shoulder MG996R) : pin 13
//  Servo 2 (Elbow    MG996R) : pin 12
//  Servo 3 (Wrist    SG90  ) : pin 14
//  Servo 4 (WristRoll SG90 ) : pin 15
//  Servo 5 (Gripper  SG90  ) : pin 4
// ============================================================

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <ESP32Servo.h>

// ============================================================
//  Motor Pins
// ============================================================
#define R1_IN1 23
#define R1_IN2 22
#define R2_IN1 21
#define R2_IN2 19
#define L1_IN1 18
#define L1_IN2 5
#define L2_IN1 17
#define L2_IN2 16

// ============================================================
//  Ultrasonic ISR (pin 34 supports interrupts)
// ============================================================
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
  if (millis() - lastTrigMs < 60) return;
  lastTrigMs = millis();
  digitalWrite(FRONT_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(FRONT_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(FRONT_TRIG, LOW);
}

// ============================================================
//  VL53L1X ToF
// ============================================================
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
//  Servo Pins & Objects
// ============================================================
#define PIN_SHOULDER  13
#define PIN_ELBOW     12
#define PIN_WRIST     14
#define PIN_WRISTROLL 15
#define PIN_GRIPPER    4

Servo servoShoulder;
Servo servoElbow;
Servo servoWrist;
Servo servoWristRoll;
Servo servoGripper;

// Angle limits (degrees)
#define SHOULDER_MIN  20
#define SHOULDER_MAX  160
#define ELBOW_MIN      0
#define ELBOW_MAX    180
#define WRIST_MIN      0
#define WRIST_MAX    180
#define WROLL_MIN      0
#define WROLL_MAX    180
#define GRIP_MIN       0   // fully open
#define GRIP_MAX      90   // fully closed

// Home position
#define SHOULDER_HOME  90
#define ELBOW_HOME     90
#define WRIST_HOME     90
#define WROLL_HOME     90
#define GRIP_HOME      20  // slightly open

// Current angles (float for smooth incremental)
float angShoulder  = SHOULDER_HOME;
float angElbow     = ELBOW_HOME;
float angWrist     = WRIST_HOME;
float angWristRoll = WROLL_HOME;
float angGripper   = GRIP_HOME;

// Max degrees per loop at full stick deflection
// Loop runs every ~10ms => 2.0 deg/loop = 200 deg/sec max
#define STICK_SPEED   2.0f
#define BTN_SPEED     1.5f  // degrees per loop for L1/R1/L2/R2
#define STICK_DEAD     30   // deadzone out of 512

// ============================================================
//  Zigzag Thresholds
// ============================================================
#define DIST_STOP   10
#define DIST_WARN   18
#define DIST_CLEAR  22
#define TURN_MS    800
#define REVERSE_MS 350

// ============================================================
//  Mode & FSM
// ============================================================
uint8_t robotMode = 0;  // 0=manual 1=auto

uint8_t       fsmState    = 0;
unsigned long fsmTimer    = 0;
unsigned long fsmDuration = 0;

float sensorFront = 999;
float sensorLeft  = 999;
float sensorRight = 999;

uint8_t       tofIdx    = 0;
unsigned long lastTofMs = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
uint16_t prevBtns = 0;

// ============================================================
//  Motors
// ============================================================
void stopMotors() {
  digitalWrite(R1_IN1,LOW); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,LOW); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,LOW); digitalWrite(L2_IN2,LOW);
}
void moveForward() {
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}
void moveBackward() {
  digitalWrite(R1_IN1,LOW); digitalWrite(R1_IN2,HIGH);
  digitalWrite(R2_IN1,LOW); digitalWrite(R2_IN2,HIGH);
  digitalWrite(L1_IN1,LOW); digitalWrite(L1_IN2,HIGH);
  digitalWrite(L2_IN1,LOW); digitalWrite(L2_IN2,HIGH);
}
void moveRight() {
  digitalWrite(R1_IN1,LOW);  digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,LOW);  digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}
void moveLeft() {
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW);  digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,LOW);  digitalWrite(L2_IN2,LOW);
}
void pivotLeft() {
  digitalWrite(R1_IN1,HIGH); digitalWrite(R1_IN2,LOW);
  digitalWrite(R2_IN1,HIGH); digitalWrite(R2_IN2,LOW);
  digitalWrite(L1_IN1,LOW);  digitalWrite(L1_IN2,HIGH);
  digitalWrite(L2_IN1,LOW);  digitalWrite(L2_IN2,HIGH);
}
void pivotRight() {
  digitalWrite(R1_IN1,LOW);  digitalWrite(R1_IN2,HIGH);
  digitalWrite(R2_IN1,LOW);  digitalWrite(R2_IN2,HIGH);
  digitalWrite(L1_IN1,HIGH); digitalWrite(L1_IN2,LOW);
  digitalWrite(L2_IN1,HIGH); digitalWrite(L2_IN2,LOW);
}

// ============================================================
//  Arm Helpers
// ============================================================
// Map stick axis (-512 to 511) to speed (-1.0 to 1.0) with deadzone
float stickToSpeed(int16_t val) {
  if (abs(val) < STICK_DEAD) return 0.0f;
  return (float)val / 512.0f;
}

// Move angle by delta, clamp to [minA, maxA], write servo
void moveServo(Servo &srv, float &angle, float delta, float minA, float maxA) {
  angle += delta;
  if (angle < minA) angle = minA;
  if (angle > maxA) angle = maxA;
  srv.write((int)angle);
}

void armHome() {
  angShoulder  = SHOULDER_HOME;
  angElbow     = ELBOW_HOME;
  angWrist     = WRIST_HOME;
  angWristRoll = WROLL_HOME;
  angGripper   = GRIP_HOME;
  servoShoulder.write(SHOULDER_HOME);
  servoElbow.write(ELBOW_HOME);
  servoWrist.write(WRIST_HOME);
  servoWristRoll.write(WROLL_HOME);
  servoGripper.write(GRIP_HOME);
  Serial.println("[ARM] Home position");
}

// ============================================================
//  Arm Control from PS4 (called in manual mode every loop)
//
//  Left  Stick Y  (axisY)  -> Shoulder  (push up = raise)
//  Right Stick Y  (axisRY) -> Elbow     (push up = raise)
//  Right Stick X  (axisRX) -> Wrist     (left/right)
//  L2 (brake)              -> WristRoll CCW
//  R2 (throttle)           -> WristRoll CW
//  L1 (held)               -> Gripper close
//  R1 (held)               -> Gripper open
//  Square (BUTTON_X)       -> Arm home position
// ============================================================
void updateArm(ControllerPtr ctl) {
  uint16_t btns     = ctl->buttons();
  uint16_t pressed  = btns & ~prevBtns;

  // Home position on Square press
  if (pressed & BUTTON_X) {
    armHome();
    return;
  }

  // -- Shoulder: left stick Y (inverted: up = positive = raise)
  float spShoulder = -stickToSpeed(ctl->axisY());
  moveServo(servoShoulder, angShoulder,
            spShoulder * STICK_SPEED,
            SHOULDER_MIN, SHOULDER_MAX);

  // -- Elbow: right stick Y (inverted)
  float spElbow = -stickToSpeed(ctl->axisRY());
  moveServo(servoElbow, angElbow,
            spElbow * STICK_SPEED,
            ELBOW_MIN, ELBOW_MAX);

  // -- Wrist: right stick X
  float spWrist = stickToSpeed(ctl->axisRX());
  moveServo(servoWrist, angWrist,
            spWrist * STICK_SPEED,
            WRIST_MIN, WRIST_MAX);

  // -- WristRoll: L2 = CCW, R2 = CW
  // brake() and throttle() return 0-1023
  float l2 = ctl->brake()    / 1023.0f;
  float r2 = ctl->throttle() / 1023.0f;
  float spRoll = r2 - l2;  // positive = CW
  if (abs(spRoll) > 0.05f) {
    moveServo(servoWristRoll, angWristRoll,
              spRoll * BTN_SPEED,
              WROLL_MIN, WROLL_MAX);
  }

  // -- Gripper: L1 = close, R1 = open
  if (btns & BUTTON_SHOULDER_L) {
    moveServo(servoGripper, angGripper,
               BTN_SPEED, GRIP_MIN, GRIP_MAX);
  }
  if (btns & BUTTON_SHOULDER_R) {
    moveServo(servoGripper, angGripper,
              -BTN_SPEED, GRIP_MIN, GRIP_MAX);
  }
}

// ============================================================
//  Sensors
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
    unsigned long t = millis();
    while (!tofLeft.dataReady() && millis() - t < 80);
    if (tofLeft.dataReady()) {
      int16_t mm = tofLeft.distance();
      tofLeft.clearInterrupt(); tofLeft.startRanging();
      if (mm > 0) sensorLeft = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  if (tofIdx == 1 && tofRightOK) {
    unsigned long t = millis();
    while (!tofRight.dataReady() && millis() - t < 80);
    if (tofRight.dataReady()) {
      int16_t mm = tofRight.distance();
      tofRight.clearInterrupt(); tofRight.startRanging();
      if (mm > 0) sensorRight = constrain(mm / 10.0f, 0.0f, 400.0f);
    }
  }
  tofIdx = (tofIdx + 1) % 2;
}

// ============================================================
//  Auto FSM - Zigzag
// ============================================================
void runAuto() {
  bool done = (millis() - fsmTimer >= fsmDuration);

  if (fsmState == 0) {
    moveForward();
    if (sensorFront <= DIST_STOP) {
      stopMotors(); delay(80);
      fsmState = 1; fsmTimer = millis(); fsmDuration = REVERSE_MS;
    } else if (sensorFront <= DIST_WARN) {
      stopMotors(); delay(80);
      if (sensorLeft >= sensorRight) {
        fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
      } else {
        fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
      }
    }
  }
  else if (fsmState == 1) {
    moveBackward();
    if (done) {
      stopMotors(); delay(80);
      if (sensorLeft >= sensorRight) {
        fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
      } else {
        fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
      }
    }
  }
  else if (fsmState == 2) {
    pivotLeft();
    if (done) {
      stopMotors(); delay(100);
      if (sensorFront > DIST_CLEAR) {
        fsmState = 0;
      } else {
        fsmState = 3; fsmTimer = millis(); fsmDuration = TURN_MS;
      }
    }
  }
  else if (fsmState == 3) {
    pivotRight();
    if (done) {
      stopMotors(); delay(100);
      if (sensorFront > DIST_CLEAR) {
        fsmState = 0;
      } else {
        fsmState = 2; fsmTimer = millis(); fsmDuration = TURN_MS;
      }
    }
  }

  const char* names[] = {"FWD","REV","LEFT","RIGHT"};
  Serial.printf("[AUTO] F=%.0f L=%.0f R=%.0f | %s\n",
    sensorFront, sensorLeft, sensorRight, names[fsmState]);
}

// ============================================================
//  Manual Mode (D-pad = robot movement)
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
  pinMode(TOF_XSHUT_L, OUTPUT); pinMode(TOF_XSHUT_R, OUTPUT);
  digitalWrite(TOF_XSHUT_L, LOW); digitalWrite(TOF_XSHUT_R, LOW);
  delay(100);

  digitalWrite(TOF_XSHUT_L, HIGH); delay(200);
  if (tofLeft.begin(TOF_ADDR_L, &Wire)) {
    tofLeft.startRanging(); tofLeft.setTimingBudget(50);
    tofLeftOK = true;
    Serial.printf("[TOF] Left  OK 0x%02X\n", TOF_ADDR_L);
  } else { Serial.println("[TOF] Left  FAIL"); }

  digitalWrite(TOF_XSHUT_R, HIGH); delay(200);
  if (tofRight.begin(TOF_ADDR_R, &Wire)) {
    tofRight.startRanging(); tofRight.setTimingBudget(50);
    tofRightOK = true;
    Serial.printf("[TOF] Right OK 0x%02X\n", TOF_ADDR_R);
  } else { Serial.println("[TOF] Right FAIL"); }
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== ROBOT + ARM STARTED ===");

  // Motors
  int motorPins[] = {R1_IN1,R1_IN2,R2_IN1,R2_IN2,
                     L1_IN1,L1_IN2,L2_IN1,L2_IN2};
  for (int p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  stopMotors();

  // Ultrasonic
  pinMode(FRONT_TRIG, OUTPUT); digitalWrite(FRONT_TRIG, LOW);
  pinMode(FRONT_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(FRONT_ECHO), echoISR, CHANGE);
  Serial.println("[US]  ISR ready pin 34");

  // I2C + ToF
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  initToF();

  // Servos - allocate timers before attach
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoWrist.setPeriodHertz(50);
  servoWristRoll.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  servoShoulder.attach(PIN_SHOULDER,  500, 2400);
  servoElbow.attach(PIN_ELBOW,        500, 2400);
  servoWrist.attach(PIN_WRIST,        500, 2400);
  servoWristRoll.attach(PIN_WRISTROLL,500, 2400);
  servoGripper.attach(PIN_GRIPPER,    500, 2400);

  armHome();
  Serial.println("[ARM] Servos ready - Home position set");

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("[INFO] Ready!  X=Auto | O=Manual");
  Serial.println("[ARM]  LS-Y=Shoulder | RS-Y=Elbow | RS-X=Wrist");
  Serial.println("[ARM]  L2/R2=WristRoll | L1=GripClose | R1=GripOpen | Sq=Home");
}

// ============================================================
//  Loop
// ============================================================
void loop() {
  BP32.update();
  updateSensors();

  for (auto ctl : myControllers) {
    if (!ctl || !ctl->isConnected()) continue;

    uint16_t btns    = ctl->buttons();
    uint16_t pressed = btns & ~prevBtns;

    // Mode switching
    if (pressed & BUTTON_A) {   // X button -> AUTO
      robotMode   = 1;
      fsmState    = 0;
      fsmTimer    = millis();
      fsmDuration = 0;
      stopMotors();
      armHome();  // park arm before auto mode
      Serial.println("[MODE] AUTO");
    }
    if (pressed & BUTTON_B) {   // O button -> MANUAL
      robotMode = 0;
      stopMotors();
      Serial.println("[MODE] MANUAL");
    }

    prevBtns = btns;

    if (robotMode == 0) {
      runManual(ctl);    // D-pad moves robot
      updateArm(ctl);    // sticks/triggers move arm
    }
  }

  if (robotMode == 1) runAuto();

  delay(10);
}
