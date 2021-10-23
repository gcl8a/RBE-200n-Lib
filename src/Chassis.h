#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <RBE-200n-Lib.h>
#include <MotorEncoded.h>

/**
 * WORK IN PROGRESS.
 */

class Chassis
{
protected:
    MotorEncoded leftMotor;
    MotorEncoded rightMotor;

    bool timerAllocated = false;
    void allocateTimer(int PWMgenerationTimer);
    void MotorHandler(void);

   	friend void onMotorTimer(void* param);

public:
    Chassis(void);

    void init(void);
    void loop(void);

};

extern Chassis chassis;

#endif