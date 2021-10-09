/*
 * MotorEncoded.cpp
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 * 
 * 		Reorganzed by: gcl8a
 */

#include <MotorEncoded.h>

// todo: make these member data
#define ENCODER_CPR 12.0f
#define GEAR_BOX_RATIO 120.0f

const float DEGREES_PER_TICK = 360.0 / (ENCODER_CPR * GEAR_BOX_RATIO);

/**
 * Constructor for an encoded motor that uses pwm and direction pins + quadrature encoder.
 * 
 * todo: create option for paired direction pins (eg, like the MC33926)
 */
MotorEncoded::MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin)
	: MotorBase(pwmPin, dirPin), speedController(0.02, 0.01, 0, 500)
{
	MotorEncAPin = encAPin;
	MotorEncBPin = encBPin;
}

/**
 * Destructor pauses the encoder before removing object.
 */
MotorEncoded::~MotorEncoded()
{
	encoder.pauseCount();
}

/**
 * Sets up the encoders and calls the base class attach (which registers the motor in the list)
 */
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
 * Sets the target speed in degrees/second.
 * 
 * If the motor is not currently speed controlled, it resets the encoder counts; otherwise
 * there can be a jump in the motor since the error can be artificially large.
 * 
 * @param dps the new speed in degrees per second
 */
void MotorEncoded::setTargetDegreesPerSecond(float dps)
{
	attach();

	if(ctrlMode != CTRL_SPEED)
	{
		Serial.println("Resetting encoder");
		resetEncoder(); //avoids jumps when engaging control algorithm
	}

	// convert dps to ticks per control interval to avoid all the maths each time through the loop
	targetTicksPerInterval = dps * controlIntervalMS * 0.001 / DEGREES_PER_TICK;

	ctrlMode = CTRL_SPEED;
}

/**
 * This method is called for each motor when it is time to perform control.
 * 
 * Currently, it only implements speed control.
 * 
 * todo: implement position control; implement speed control through position control (red queen)
 *
 */
void MotorEncoded::process()
{
	// ensure that the motor is attached
	attach();

	//update the encoder regardless of whether or not we're going to perform control
	//this prevents jumps when changing control algorithms
	currEncoder = encoder.getCount();
	if(isReversed) currEncoder *= -1;

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
	float ticksPerInterval = currTicksPerInterval;

	return ticksPerInterval * DEGREES_PER_TICK * (1000.0 / controlIntervalMS);
}

/**
 * getCurrentDegrees
 *
 * This function returns the current position in degrees (since last encoder reset)
 * @return count
 */
float MotorEncoded::getCurrentDegrees()
{
	float tmp = currEncoder;
	return tmp * DEGREES_PER_TICK;
}
