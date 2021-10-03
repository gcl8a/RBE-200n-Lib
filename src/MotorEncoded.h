/*
 * Motor.h
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#ifndef LIBRARIES_RBE200nLIB_SRC_MOTOR_ENCODED_H_
#define LIBRARIES_RBE200nLIB_SRC_MOTOR_ENCODED_H_

#include <MotorBase.h>
#include <PIDcontroller.h>

/** \brief A PID Motor class using FreeRTOS threads, ESP32Encoder and ESP32PWM
 *
 * This Motor class is intended to be used by RBE 1001 in the WPI Robotics Department.
 *
 * Motor objects can be instantiated statically. The attach method must be called before using the motor.
 *
 * The motor uses one timer for the ESP32PWM objects. That means the static method
 *
 * Motor::allocateTimer (int PWMgenerationTimer)
 *
 * must be called before any motor objects can be attached. This method will also start the PID thread.
 *
 */
class MotorEncoded : public MotorBase
{
protected:
	enum CTRL_MODE {CTRL_DIRECT, CTRL_SPEED, CTRL_POS, CTRL_SPD_WITH_POS};
	CTRL_MODE ctrlMode = CTRL_DIRECT;

private:
	/**
	 * GPIO pin number of the motor encoder A
	 */
	int MotorEncAPin = -1;
	/**
	 * GPIO pin number of the motor encoder B
	 */
	int MotorEncBPin = -1;

	bool encodersEnabled = false;

	/**
	 * ESP32Encoder object to keep track of the motors position
	 */
	ESP32Encoder encoder;
	/**
	 * an internal counter that counts iterations of the PID loop
	 * this is used to calculate N ms timing for calculation of the velocity
	 */
	uint32_t velocityLoopCounter = 0;
	/**
	 * loop rate for this motor
	 * timer loop is 1ms, so this value is in ms
	 */
	uint32_t controlIntervalMS = 50;
	/**
	 * Variable to store the latest encoder read from the encoder hardware as read by the PID thread.
	 * This variable is set inside the PID thread, and read outside.
	 */
	int64_t currEncoder = 0;
	/*
	 * this stores the previous count of the encoder last time the velocity was calculated
	 */
	int64_t prevEncoder = 0;
	/**
	 * PID controller setpoint in encoder ticks
	 */
	float targetTicksPerInterval = 0; //really needs to be float?

	int64_t currTicksPerInterval = 0;

	PIDController speedController;

public:
	MotorEncoded(int pwmPin, int dirPin, int encAPin, int encBPin);
	virtual ~MotorEncoded();

	void setTargetDegreesPerSecond(float dps);

	float getDegreesPerSecond();

	float getCurrentDegrees();
	
	int64_t resetEncoder(void) {return prevEncoder = currEncoder;}

	void setEffort(float effort)
	{
		// when setEffort is called, we stop closed-loop control
		ctrlMode = CTRL_DIRECT;

		MotorBase::setEffort(effort);
	}
	
	void setEffortPercent(float effort)
	{
		// when setEffort is called, we stop closed-loop control
		ctrlMode = CTRL_DIRECT;

		MotorBase::setEffortPercent(effort);
	}

protected:
	virtual bool attach(void);

	void process();
	/**
	 * SetSpeed in degrees with time
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
};

#endif 
