#include <Chassis.h>

/**
 * JUST A TEST FILE FOR THE MOMENT.
 * 
 * Don't think this is complete.
 */

Robot::Robot(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)
{
    
}

void Robot::loop(void)
{
    delay(100);

    Serial.print('\n');
    Serial.print(leftMotor.getCurrentDegrees());
    Serial.print('\t');
    Serial.print(leftMotor.getDegreesPerSecond());
    Serial.print('\t');

    delay(20);

    leftMotor.setTargetDegreesPerSecond(30);
}