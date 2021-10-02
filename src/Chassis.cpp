#include <Chassis.h>

Robot::Robot(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)

{
    
}

void Robot::loop(void)
{
    delay(10);
    Serial.println(leftMotor.getDegreesPerSecond());
}