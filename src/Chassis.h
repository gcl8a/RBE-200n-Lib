#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <RBE-200n-Lib.h>
#include <MotorEncoded.h>
#include <wall_follower.h>

#include <event_timer.h>
#include <button.h>

#include <maxbotix.h>
#include <IRdecoder.h>

#define IR_PIN 15

/**
 * WORK IN PROGRESS.
 */

class Robot
{
protected:
    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DEAD_RECKONING, ROBOT_DRIVING, ROBOT_WALL_FOLLOWING};
    ROBOT_STATE robotState = ROBOT_IDLE;

    MotorEncoded leftMotor;
    MotorEncoded rightMotor;

    bool timerAllocated = false;
    void allocateTimer(int PWMgenerationTimer);
    void MotorHandler(void);

   	friend void onMotorTimer(void* param);

    WallFollower wallFollower;

public:
    Robot(void);

    void init(void);
    void loop(void);

    void HandleIRPress(int16_t);
};

extern Robot robot;

#endif