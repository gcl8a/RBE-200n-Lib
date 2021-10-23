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

    void stop(void) {setWheelSpeeds(0,0);}

    void setWheelSpeeds(float left, float right);
};

/*
 * We declare an extern chassis. This will let the compiler know that there exists
 * an object names chassis, which can now be addressed from global (static) constructs.
 * It's similar to declaring a static object.
 * 
 * Using an extern (or a static member, for that matter) avoids declaring static
 * member functions (note that there are no static functions in Chassis). In 
 * principle, then, you could declare another Chassis object, which is pointless
 * here, but useful for, say, multiple rangefinders and so forth.
 */

extern Chassis chassis;

#endif