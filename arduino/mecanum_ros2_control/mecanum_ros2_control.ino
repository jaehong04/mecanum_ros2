#include <Wire.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "QGPMaker_MotorShield.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();

QGPMaker_DCMotor *motorFL = AFMS.getMotor(1);
QGPMaker_DCMotor *motorRL = AFMS.getMotor(2);
QGPMaker_DCMotor *motorRR = AFMS.getMotor(3);
QGPMaker_DCMotor *motorFR = AFMS.getMotor(4);

// 배열 순서: FL, FR, RL, RR
volatile long encoderCount[4] = {0, 0, 0, 0};
volatile byte lastState[4];

const int8_t quadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// 기존 실측 기준: 로봇 전진 시 양수 속도가 되도록 보정
const int8_t encoderSign[4] = {-1, -1, -1, 1};

const float COUNTS_PER_REV = 4300.0;
const float TWO_PI_F = 6.28318530718;

// 초기 안전 설정
const float MAX_TARGET_RAD_S = 12.0;
const int MAX_PWM = 200;

// 초기 속도 제어 계수: 추후 실측으로 조정
const float KFF = 25.0;
const float KP = 6.0;

const unsigned long CONTROL_PERIOD_US = 20000;  // 50Hz
const unsigned long FEEDBACK_PERIOD_MS = 50;    // 20Hz
const unsigned long COMMAND_TIMEOUT_MS = 300;

float targetVelocity[4] = {0, 0, 0, 0};
float measuredVelocity[4] = {0, 0, 0, 0};

long previousCount[4] = {0, 0, 0, 0};

unsigned long lastControlUs = 0;
unsigned long lastFeedbackMs = 0;
unsigned long lastCommandMs = 0;

char receiveBuffer[96];
byte receiveIndex = 0;

inline byte readFL() {
  byte b = PINB;
  return ((b & _BV(PB0)) ? 2 : 0) |
         ((b & _BV(PB1)) ? 1 : 0);  // D8, D9
}

inline byte readFR() {
  byte d = PIND;
  return ((d & _BV(PD4)) ? 2 : 0) |
         ((d & _BV(PD5)) ? 1 : 0);  // D4, D5
}

inline byte readRL() {
  byte d = PIND;
  return ((d & _BV(PD6)) ? 2 : 0) |
         ((d & _BV(PD7)) ? 1 : 0);  // D6, D7
}

inline byte readRR() {
  byte d = PIND;
  return ((d & _BV(PD3)) ? 2 : 0) |
         ((d & _BV(PD2)) ? 1 : 0);  // D3, D2
}

void updateEncoder(byte index, byte state) {
  byte tableIndex = (lastState[index] << 2) | state;
  encoderCount[index] += quadTable[tableIndex];
  lastState[index] = state;
}

ISR(PCINT0_vect) {
  updateEncoder(0, readFL());
}

ISR(PCINT2_vect) {
  updateEncoder(1, readFR());
  updateEncoder(2, readRL());
  updateEncoder(3, readRR());
}

void setupEncoders() {
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);

  noInterrupts();

  lastState[0] = readFL();
  lastState[1] = readFR();
  lastState[2] = readRL();
  lastState[3] = readRR();

  PCICR |= _BV(PCIE0) | _BV(PCIE2);

  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1);
  PCMSK2 |= _BV(PCINT18) | _BV(PCINT19) |
            _BV(PCINT20) | _BV(PCINT21) |
            _BV(PCINT22) | _BV(PCINT23);

  interrupts();
}

void copyEncoderCounts(long output[4]) {
  noInterrupts();

  for (byte i = 0; i < 4; i++) {
    output[i] = encoderCount[i];
  }

  interrupts();
}

void stopAll() {
  motorFL->run(RELEASE);
  motorFR->run(RELEASE);
  motorRL->run(RELEASE);
  motorRR->run(RELEASE);
}

void setMotorSigned(QGPMaker_DCMotor *motor, float pwm) {
  int magnitude = constrain((int)abs(pwm), 0, MAX_PWM);

  if (magnitude == 0) {
    motor->run(RELEASE);
    return;
  }

  // 기존 차량 기준 BACKWARD가 물리적 전진
  if (pwm > 0) {
    motor->run(BACKWARD);
  } else {
    motor->run(FORWARD);
  }

  motor->setSpeed(magnitude);
}

void applyWheelOutputs(float pwm[4]) {
  // 제어 배열 순서: FL, FR, RL, RR
  setMotorSigned(motorFL, pwm[0]);
  setMotorSigned(motorFR, pwm[1]);
  setMotorSigned(motorRL, pwm[2]);
  setMotorSigned(motorRR, pwm[3]);
}

void setTargetsToZero() {
  for (byte i = 0; i < 4; i++) {
    targetVelocity[i] = 0.0;
  }
}

void processCommand(char *line) {
  if (strcmp(line, "S") == 0) {
    setTargetsToZero();
    stopAll();
    lastCommandMs = millis();
    Serial.println("OK,S");
    return;
  }

  if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strncmp(line, "V,", 2) != 0) {
    Serial.println("ERR,UNKNOWN");
    return;
  }

  char *token = strtok(line, ",");

  for (byte i = 0; i < 4; i++) {
    token = strtok(NULL, ",");

    if (token == NULL) {
      Serial.println("ERR,FORMAT");
      return;
    }

    targetVelocity[i] = constrain(
      atof(token),
      -MAX_TARGET_RAD_S,
      MAX_TARGET_RAD_S
    );
  }

  lastCommandMs = millis();
  Serial.println("OK,V");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char value = Serial.read();

    if (value == '\n' || value == '\r') {
      if (receiveIndex > 0) {
        receiveBuffer[receiveIndex] = '\0';
        processCommand(receiveBuffer);
        receiveIndex = 0;
      }
    } else if (receiveIndex < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveIndex++] = value;
    } else {
      receiveIndex = 0;
      Serial.println("ERR,OVERFLOW");
    }
  }
}

void updateVelocityControl() {
  unsigned long nowUs = micros();
  unsigned long elapsedUs = nowUs - lastControlUs;

  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }

  lastControlUs = nowUs;

  float dt = elapsedUs / 1000000.0;
  long currentCount[4];

  copyEncoderCounts(currentCount);

  for (byte i = 0; i < 4; i++) {
    long delta = currentCount[i] - previousCount[i];
    previousCount[i] = currentCount[i];

    measuredVelocity[i] =
      encoderSign[i] *
      delta *
      TWO_PI_F /
      COUNTS_PER_REV /
      dt;
  }

  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    setTargetsToZero();
  }

  float pwm[4];

  for (byte i = 0; i < 4; i++) {
    if (abs(targetVelocity[i]) < 0.01) {
      pwm[i] = 0.0;
    } else {
      float error = targetVelocity[i] - measuredVelocity[i];

      pwm[i] =
        KFF * targetVelocity[i] +
        KP * error;

      pwm[i] = constrain(pwm[i], -MAX_PWM, MAX_PWM);
    }
  }

  applyWheelOutputs(pwm);
}

void sendEncoderFeedback() {
  unsigned long nowMs = millis();

  if (nowMs - lastFeedbackMs < FEEDBACK_PERIOD_MS) {
    return;
  }

  lastFeedbackMs = nowMs;

  Serial.print("E,");
  Serial.print(measuredVelocity[0], 4);
  Serial.print(",");
  Serial.print(measuredVelocity[1], 4);
  Serial.print(",");
  Serial.print(measuredVelocity[2], 4);
  Serial.print(",");
  Serial.println(measuredVelocity[3], 4);
}

void setup() {
  Serial.begin(115200);
  AFMS.begin();

  setupEncoders();
  stopAll();

  copyEncoderCounts(previousCount);

  lastControlUs = micros();
  lastFeedbackMs = millis();
  lastCommandMs = millis();

  Serial.println("READY,MECANUM_ROS2_CONTROL");
  Serial.println("FORMAT,V,FL,FR,RL,RR");
}

void loop() {
  readSerialCommands();
  updateVelocityControl();
  sendEncoderFeedback();
}
