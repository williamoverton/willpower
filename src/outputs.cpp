#include <Arduino.h>
#include <Servo.h>
#include "common.h"
#include "utils.h"
#include "state.h"
#include "pins.h"
#include "controller.h"
#include "stabilize.h"

// Servos
Servo leftMotor;
Servo rightMotor;
Servo leftAileron;
Servo rightAileron;

void positionServos()
{
    
    
    // ESCs expect a PWM signal between 1000-2000 microseconds
    leftMotor.writeMicroseconds(constrain(int(fmap(outputLeftMotor, 0.0, 1.0, 1000, 2000)), 1000, 2000));
    rightMotor.writeMicroseconds(constrain(int(fmap(outputRightMotor, 0, 1.0, 1000, 2000)), 1000, 2000));

    float maxServoAngle = 180.0 - 60;
    float minServoAngle = 60.0;

    float leftAileronTrim = 0.0;
    float rightAileronTrim = -10.0;

    leftAileron.write(constrain(int(leftAileronTrim + fmap(outputLeftavon, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    rightAileron.write(constrain(int(rightAileronTrim + fmap(outputRightavon, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
}

void setupServos()
{
    leftMotor.attach(LEFT_MOTOR_SERVO_PIN);
    rightMotor.attach(RIGHT_MOTOR_SERVO_PIN);
    leftAileron.attach(LEFTAVON_SERVO_PIN);
    rightAileron.attach(RIGHTAVON_SERVO_PIN);

    positionServos();
}