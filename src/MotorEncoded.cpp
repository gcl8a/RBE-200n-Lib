/*
 * Motor.cpp
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#include <MotorEncoded.h>

#define ENCODER_CPR 12.0f
#define GEAR_BOX_RATIO 120.0f

const float DEGREES_PER_TICK = 360.0 / (ENCODER_CPR * GEAR_BOX_RATIO);

MotorEncoded::MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin)
	: MotorBase(pwmPin, dirPin), speedController(0.1)
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
	attach();

	if(motorState != MOTOR_CLOSED_LOOP_CTRL)
	{
		Serial.println("Resetting encoder");
		resetEncoder(); //avoids jumps when engaging control algorithm
		motorState = MOTOR_CLOSED_LOOP_CTRL;
	}

	targetTicksPerInterval = dps * controlIntervalMS * 0.001 / DEGREES_PER_TICK;

	ctrlMode = CTRL_SPEED;
}

/**
 * Loop function
 * this method is called by the timer to run the PID control of the motors and ensure strict timing
 *
 */
void MotorEncoded::process()
{
	attach();
	//update the encoder regardless of whether or not we're going to perform control
	//this prevents jumps when engaging control algorithms
	currEncoder = encoder.getCount();

	if(motorState == MOTOR_CLOSED_LOOP_CTRL)
	{
		if(ctrlMode == CTRL_SPEED)
		{
			if(++velocityLoopCounter >= controlIntervalMS)
			{
				velocityLoopCounter = 0;

				currTicksPerInterval = currEncoder - prevEncoder;
				prevEncoder = currEncoder;

				float error = targetTicksPerInterval - currTicksPerInterval;
				float effort = speedController.ComputeEffort(error);

				// Serial.print('\n');
				// Serial.print(currEncoder);
				// Serial.print('\t');
				// Serial.print(prevEncoder);
				// Serial.print('\t');
				// Serial.print(targetTicksPerInterval);
				// Serial.print('\t');
				// Serial.print(currTicksPerInterval);
				// Serial.print('\t');
				// Serial.print(error);
				// Serial.print('\t');
				// Serial.print(effort);
				// Serial.print('\t');

				setTargetEffort(effort);
			}
		}
	}

	MotorBase::process();
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
	attach();
	
	float ticksPerInterval = currTicksPerInterval;

	return ticksPerInterval * DEGREES_PER_TICK * (1000.0 / controlIntervalMS);
}
/**
 * getTicks
 *
 * This function returns the current count of encoders
 * @return count
 */
float MotorEncoded::getCurrentDegrees()
{
	attach();

	float tmp = currEncoder;
	return tmp * DEGREES_PER_TICK;
}
