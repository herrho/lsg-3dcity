#include "MotorControl.h"

MotorControl::MotorControl(int pin1, int pin2, int enablePin) {
  this->pin1 = pin1;
  this->pin2 = pin2;
  this->enablePin = enablePin;
  this->currentSpeed = 0;
}

void MotorControl::init() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  stop();
}

void MotorControl::forward(int speed) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(enablePin, speed);
  currentSpeed = speed;
}

void MotorControl::reverse(int speed) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(enablePin, speed);
  currentSpeed = speed;
}

void MotorControl::stop() {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  analogWrite(enablePin, 0);
  currentSpeed = 0;
}
