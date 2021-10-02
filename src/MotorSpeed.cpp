/*
 * Motor.cpp
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#include <MotorSpeed.h>


MotorSpeedControl::MotorSpeedControl(int pwmPin, int dirPin, int encAPin, int encBPin)
	: MotorBase(pwmPin, dirPin), speedController(1)
{
	MotorEncAPin = encAPin;
	MotorEncBPin = encBPin;
}

MotorSpeedControl::~MotorSpeedControl()
{
	encoder.pauseCount();
}

void MotorSpeedControl::attach(void)
{
	MotorBase::attach();

	if(!encodersEnabled)
	{
		encodersEnabled = true;
		ESP32Encoder::useInternalWeakPullResistors = UP;
		encoder.attachFullQuad(MotorEncAPin, MotorEncBPin);
	}
}	

/**
 * setSpeed in degrees with time
 * Set the setpoint for the motor in degrees
 * This implements "Red Queen" mode running interpolation in the PID controller.

 "Now, here, you see, it takes all the running you can do, to keep in the same place.

 If you want to get somewhere else, you must run at least twice as fast as that!"

 — The Red Queen, Alice In Wonderland, Lewis Carroll

 * The way this velocity mode works is that the position target is moved forward every iteration of the PID
 * loop. The position runs away continuously, in order to keep the velocity stable.
 * A position increment is calculated, and added to the Position every 1ms of the loop()
 *
 * @param newDegreesPerSecond the new speed in degrees per second
 */
void MotorSpeedControl::setTargetDegreesPerSecond(float dps)
{
	targetTicksPerInterval = dps * processIntervalMS * 0.001 / TICKS_TO_DEGREES;

//	closedLoopControl = true;
}

/**
 * Loop function
 * this method is called by the timer to run the PID control of the motors and ensure strict timing
 *
 */
void MotorSpeedControl::process()
{
	if(++interruptCountForVelocity >= processIntervalMS)
	{
		interruptCountForVelocity = 0;

		nowEncoder = encoder.getCount();
		float currSpeed = nowEncoder - previousCount;
		previousCount = nowEncoder;

		float error = targetTicksPerInterval - currSpeed;
		float effort = speedController.ComputeEffort(error);

		setEffortLocal(effort);
	}
}
/**
 * getDegreesPerSecond
 *
 * This function returns the current speed of the motor
 *
 * @return the speed of the motor in degrees per second
 */
float MotorSpeedControl::getDegreesPerSecond()
{
	float tmp = -999;

	//tmp = cachedSpeed;

	return -tmp * TICKS_TO_DEGREES;
}
/**
 * getTicks
 *
 * This function returns the current count of encoders
 * @return count
 */
float MotorSpeedControl::getCurrentDegrees()
{
	float tmp = nowEncoder;
	return tmp * TICKS_TO_DEGREES;
}
