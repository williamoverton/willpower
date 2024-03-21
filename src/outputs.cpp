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
    const float maxServoAngle = 180.0;
    const float minServoAngle = 0.0;

    // Position servos based on output pitch, roll, and yaw
    // Remap output pitch, roll and yaw to 0-180 degrees for servo
    leftMotor.write(constrain(int(fmap(outputLeftMotor, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    rightMotor.write(constrain(int(fmap(outputRightMotor, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));

    leftAileron.write(constrain(int(fmap(outputLeftavon, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    rightAileron.write(constrain(int(fmap(outputRightavon, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
}

void setupServos()
{
    leftMotor.attach(PITCH_SERVO_PIN);
    rightMotor.attach(ROLL_SERVO_PIN);
    leftAileron.attach(YAW_SERVO_PIN);
    rightAileron.attach(THROTTLE_SERVO_PIN);

    positionServos();
}