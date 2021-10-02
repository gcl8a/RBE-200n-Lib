#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <RBE-200n-Lib.h>
#include <MotorSpeed.h>


class Robot
{
protected:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVING, ROBOT_WALL_FOLLOWING, ROBOT_READOUT_SENSORS};
    ROBOT_STATE robot_state = ROBOT_IDLE;

    MotorSpeedControl leftMotor;
    MotorSpeedControl rightMotor;

public:
    Robot(void);

    void loop(void);
};

#endif