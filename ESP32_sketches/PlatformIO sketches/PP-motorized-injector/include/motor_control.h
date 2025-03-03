#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <FastAccelStepper.h>

// Function declarations
void homeMove(int fastSpeed, int slowSpeed);
void programmedMotorMove(int speed, int distance, int accel = 0);
void continuousMotorMoveUp(int speed);
void continuousMotorMoveDown(int speed);
void compressionFunction(int initialSpeed, int minSpeed);

extern FastAccelStepper *stepper;
extern bool doMoveMotor;

#endif // MOTOR_CONTROL_H