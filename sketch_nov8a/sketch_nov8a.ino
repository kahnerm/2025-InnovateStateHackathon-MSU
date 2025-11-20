#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ps3Controller.h>
#include <math.h>

// =======================================================
// === SERVO SETUP ===
Servo servo1;
Servo servo2;

int servoPos = 90;            // Start centered
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int MOVE_SPEED = 5;
const int UPDATE_DELAY = 20;

// =======================================================
// === MOTOR SETUP ===
const int M1_IN1 = 25, M1_IN2 = 33, M1_EN = 32;
const int M2_IN1 = 19, M2_IN2 = 18, M2_EN = 13;
const int M3_IN1 = 27, M3_IN2 = 26, M3_EN = 14;
const int M4_IN1 = 23, M4_IN2 = 21, M4_EN = 22;

const int FREQ = 2000; // 2 kHz for motors
const int RES = 8;      // 8-bit PWM resolution
const int CH1 = 0, CH2 = 1, CH3 = 2, CH4 = 3;

static inline void pwmAttach(int pin, int freq, int res, int ch) {
  ledcAttach(pin, freq, res);
}

static inline void pwmWrite(int pin, int ch, int duty) {
  ledcWrite(ch, duty);
}

const int IN1[4] = { M1_IN1, M2_IN1, M3_IN1, M4_IN1 };
const int IN2[4] = { M1_IN2, M2_IN2, M3_IN2, M4_IN2 };
const int EN [4] = { M1_EN,  M2_EN,  M3_EN,  M4_EN  };
const int CH [4] = { CH1,    CH2,    CH3,    CH4    };
const int LEFT_IDX[2]  = {0, 2};
const int RIGHT_IDX[2] = {1, 3};

const int DEADZONE = 7;
const int STOP_THRESHOLD = 10;
const int HYSTERESIS = 6;

// =======================================================
// === MOTOR HELPERS ===
void motorDir(int i, bool forward) {
  digitalWrite(IN1[i], forward ? HIGH : LOW);
  digitalWrite(IN2[i], forward ? LOW  : HIGH);
}
void motorCoast(int i) { digitalWrite(IN1[i], LOW); digitalWrite(IN2[i], LOW); }
void motorBrake(int i) { digitalWrite(IN1[i], HIGH); digitalWrite(IN2[i], HIGH); }
void motorSpeed(int i, int duty) { pwmWrite(EN[i], CH[i], constrain(duty, 0, 255)); }

void setMotorSigned(int idx, int val) {
  val = constrain(val, -255, 255);
  if (val == 0) { motorCoast(idx); motorSpeed(idx, 0); return; }
  motorDir(idx, val > 0);
  motorSpeed(idx, abs(val));
}
void setPairSigned(const int idxs[2], int val) {
  setMotorSigned(idxs[0], val);
  setMotorSigned(idxs[1], val);
}
void allCoast() { for (int i = 0; i < 4; i++) { motorCoast(i); motorSpeed(i, 0); } }
void allBrake() { for (int i = 0; i < 4; i++) { motorBrake(i); motorSpeed(i, 0); } }

// =======================================================
// === STICK CURVE ===
int applyDeadzoneAndCurve(int8_t v, int deadzone = DEADZONE) {
  if (v >= -deadzone && v <= deadzone) return 0;
  int sign = (v < 0) ? -1 : 1;
  int mag = map(abs(v), deadzone, 127, 0, 255);
  float f = (float)mag / 255.0f;
  int scaled = (int)(sqrtf(f) * 255.0f + 0.5f);
  if (scaled <= STOP_THRESHOLD) return 0;
  return sign * scaled;
}

// =======================================================
// === PS3 CONNECTION ===
bool ps3_connected = false;
void onConnect() {
  ps3_connected = true;
  allCoast();
  Serial.println("PS3 connected");
}
void onDisconnect() { ps3_connected = false; allCoast(); Serial.println("PS3 disconnected"); }

// =======================================================
// === SETUP ===
void setup() {
  Serial.begin(115200);
  Serial.println("PS3 Tank Drive + Servo Sync Control (Servos Init First)");

  // --- Initialize Servos first ---
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  servo1.attach(17, 500, 2500);
  servo2.attach(5, 500, 2500);
  servo1.write(servoPos);
  servo2.write(servoPos);

  // --- Initialize Motors ---
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pwmAttach(EN[i], FREQ, RES, CH[i]);
    motorCoast(i);
    motorSpeed(i, 0);
  }

  // --- Initialize PS3 ---
  Ps3.begin("2c:81:58:40:9e:1d");
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
}

// =======================================================
// === LOOP ===
void loop() {
  if (!ps3_connected && !Ps3.isConnected()) {
    allCoast();
    delay(20);
    return;
  }

  // --- SERVO CONTROL ---
  bool moved = false;
  if (Ps3.data.analog.button.r1 > 0) {
    Serial.println("R1 -> Servo Up");
    servoPos = min(servoPos + MOVE_SPEED, SERVO_MAX);
    moved = true;
  }
  if (Ps3.data.analog.button.l1 > 0) {
    Serial.println("L1 -> Servo Down");
    servoPos = max(servoPos - MOVE_SPEED, SERVO_MIN);
    moved = true;
  }
  if (moved) {
    servo1.write(servoPos);
    servo2.write(servoPos);
    Serial.printf("Servo position: %d°\n", servoPos);
  }

  // --- MOTOR CONTROL ---
  int8_t ly = Ps3.data.analog.stick.ly;
  int8_t ry = Ps3.data.analog.stick.ry;
  int leftTarget  = applyDeadzoneAndCurve(-ly);
  int rightTarget = applyDeadzoneAndCurve(-ry);

  int r2 = Ps3.data.analog.button.r2;
  int l2 = Ps3.data.analog.button.l2;
  int maxOut = constrain(120 + (r2 - l2), 120, 255);
  leftTarget  = constrain(leftTarget,  -maxOut, maxOut);
  rightTarget = constrain(rightTarget, -maxOut, maxOut);

  static int prevLeft = 0;
  static int prevRight = 0;
  const int CHANGE_THRESHOLD = 3;

  if (abs(prevLeft - leftTarget) > CHANGE_THRESHOLD) {
    setPairSigned(LEFT_IDX, leftTarget);
    prevLeft = leftTarget;
  }
  if (abs(prevRight - rightTarget) > CHANGE_THRESHOLD) {
    setPairSigned(RIGHT_IDX, rightTarget);
    prevRight = rightTarget;
  }

  if (Ps3.data.button.triangle) { allBrake(); prevLeft = prevRight = 0; }
  if (Ps3.data.button.circle)   { allCoast(); prevLeft = prevRight = 0; }

  delay(UPDATE_DELAY);
}
