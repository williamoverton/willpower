#include <Servo.h>
#include "common.h"
#include "utils.h"
#include "state.h"
#include "pins.h"
#include "controller.h"
#include "stabilize.h"

// Servos
Servo pitchServo;
Servo rollServo;
Servo yawServo;
Servo throttleOutput;
Servo aux1Servo;
Servo aux2Servo;

void positionServos()
{
    const float maxServoAngle = 180.0;
    const float minServoAngle = 0.0;

    // Position servos based on output pitch, roll, and yaw
    // Remap output pitch, roll and yaw to 0-180 degrees for servo
    pitchServo.write(constrain(int(fmap(outputPitch, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    rollServo.write(constrain(int(fmap(outputRoll, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    yawServo.write(constrain(int(fmap(outputYaw, -1.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));


    // Position throttle based on output throttle
    throttleOutput.write(constrain(int(fmap(outputThrottle, 0.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));

    // Position aux channels
    aux1Servo.write(constrain(int(fmap(outputAux1, 0.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
    aux2Servo.write(constrain(int(fmap(outputAux2, 0.0, 1.0, minServoAngle, maxServoAngle)), (int)minServoAngle, (int)maxServoAngle));
}


void setupServos()
{
    pitchServo.attach(PITCH_SERVO_PIN);
    rollServo.attach(ROLL_SERVO_PIN);
    yawServo.attach(YAW_SERVO_PIN);
    throttleOutput.attach(THROTTLE_SERVO_PIN);
    aux1Servo.attach(AUX1_SERVO_PIN);
    aux2Servo.attach(AUX2_SERVO_PIN);

    positionServos();
}