/*
 * Motor.cpp
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#include <MotorEncoded.h>

MotorEncoded::MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin)
	: MotorBase(pwmPin, dirPin), speedController(1)
{
	MotorEncAPin = encAPin;
	MotorEncBPin = encBPin;
}

MotorEncoded::~MotorEncoded()
{
	encoder.pauseCount();
}

bool MotorEncoded::attach(void)
{
	if(MotorBase::attach())
	{
		if(!encodersEnabled)
		{
			encodersEnabled = true;
			ESP32Encoder::useInternalWeakPullResistors = UP;
			encoder.attachFullQuad(MotorEncAPin, MotorEncBPin);
		}

		return true;
	}

	// motor is not attached
	return false;
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
void MotorEncoded::setTargetDegreesPerSecond(float dps)
{
	if(motorState != MOTOR_CLOSED_LOOP_CTRL)
	{
		resetEncoder(); //avoids jumps when engaging control algorithm
		motorState = MOTOR_CLOSED_LOOP_CTRL;
	}
	targetTicksPerInterval = dps * controlIntervalMS * 0.001 / TICKS_TO_DEGREES;

//	closedLoopControl = true;
}

/**
 * Loop function
 * this method is called by the timer to run the PID control of the motors and ensure strict timing
 *
 */
void MotorEncoded::process()
{
	//update the encoder regardless of whether or not we're going to perform control
	//this prevents jumps when engaging control algorithms
	currEncoder = encoder.getCount();

	if(motorState == MOTOR_CLOSED_LOOP_CTRL)
	{
		if(++velocityLoopCounter >= controlIntervalMS)
		{
			velocityLoopCounter = 0;

			float currSpeed = currEncoder - prevEncoder;
			prevEncoder = currEncoder;

			float error = targetTicksPerInterval - currSpeed;
			float effort = speedController.ComputeEffort(error);

			setEffortLocal(effort);
		}
	}
}
/**
 * getDegreesPerSecond
 *
 * This function returns the current speed of the motor
 *
 * @return the speed of the motor in degrees per second
 */
float MotorEncoded::getDegreesPerSecond()
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
float MotorEncoded::getCurrentDegrees()
{
	float tmp = currEncoder;
	return tmp * TICKS_TO_DEGREES;
}
