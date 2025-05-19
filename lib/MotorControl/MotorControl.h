#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
  private:
    int pin1;
    int pin2;
    int enablePin;
    int currentSpeed;
    
  public:
    MotorControl(int pin1, int pin2, int enablePin);
    void init();
    void forward(int speed);
    void reverse(int speed);
    void stop();
};

#endif // MOTOR_CONTROL_H
