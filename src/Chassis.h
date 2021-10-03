#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <RBE-200n-Lib.h>
#include <MotorEncoded.h>
#include <event_timer.h>
#include <button.h>

/**
 * WORK IN PROGRESS.
 * 
 * Nowhere near complete.
 */

class Robot
{
protected:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVING, ROBOT_WALL_FOLLOWING, ROBOT_READOUT_SENSORS};
    ROBOT_STATE robot_state = ROBOT_IDLE;

    MotorEncoded leftMotor;
    MotorEncoded rightMotor;

public:
    Robot(void);

    void loop(void);
};

#endif