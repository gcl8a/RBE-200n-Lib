/*
 * Motor.h
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#ifndef LIBRARIES_RBE200nLIB_SRC_MOTOR_BASE_H_
#define LIBRARIES_RBE200nLIB_SRC_MOTOR_BASE_H_

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#define MAX_POSSIBLE_MOTORS 4

//used to reduce jerk; set to a large number to deactivate
const float DELTA_EFFORT = 1;

/** \brief A PID Motor class using FreeRTOS threads, with pwm controlled by an ESP32PWM object.
 *
 * The MotorBase class implements pwm for effort (including direction). Several methods are declared virtual
 * and overridden by derived classes (currently just MotorEncoded).
 *
 * The class uses one timer for all of the ESP32PWM objects, which is set up in allocateTimer().
 * 
 * When attached, motors are added to a list. loop() is called from the interrupt routine, which
 * calls process() for each motor. process() is virtual; derived classes can implement their specific functionality.
 */
class MotorBase
{
private:
	/**
	 * GPIO pin number of the motor PWM pin
	 */
	int MotorPWMPin = -1;

	/**
	 * GPIO pin number of the motor direction output flag
	 */
	int directionPin = -1;

	/**
	 * True if the motor has been attached
	 */
	bool isAttached = false;

	/**
	 * Hold the 'ideal' effort. The change in the actual effort (currentEffort) is limited to prevent jerk.
	 * Somewhat experiemental. Set DELTA_EFFORT to a large value to disable, as that is what limits the change.
	 * 
	 * todo: clean this up or eliminate to avoid unintended consequences when tuning.
	 */
	float targetEffort = 0;

	/**
	 * variable for caching the current effort being sent to the PWM/direction pins
	 */
	float currentEffort = 0;

	/**
	 * the object that produces PWM for motor speed
	 */
	ESP32PWM pwm;

public:
	MotorBase(int pwmPin, int dirPin);
	virtual ~MotorBase();

private:
	static bool timersAllocated;
	/**
	 * This is a list of all of the Motor objects that have been attached. As a motor is attached,
	 *  it adds itself to this list of Motor pointers. This list is read by the PID thread and each
	 *  object in the list has loop() called.
    */
    
	static MotorBase* motorList[MAX_POSSIBLE_MOTORS];

protected:
	/**
	 * \brief Setup of hardware and register the motor.
	 *
	 * This attaches the motors to the hardware ports that were saved in the constructor.
	 * @note this must only be called after timers are allocated via Motor::allocateTimers(int PWMgenerationTimer)
	 *
	 * todo: verify/eliminate the note above
	 */
	virtual bool attach(void);

public:
	/*
	 *  \brief effort of the motor, proportional to PWM
	 *
	 * @param effort a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 */
	virtual void setEffort(float effort);
	
	/*
	 * set the effort of the motor in percent
	 * @param percent a value from -100 to 100 representing effort
	 *        0 is brake
	 *        100 is full speed clockwise
	 *        -100 is full speed counter clockwise
	 */
	virtual void setEffortPercent(float percent)
	{
		setEffort(percent * 0.01);
	}

	/*
	 * effort of the motor
	 * @return a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 */
	float getEffort();

	/*
	 * effort of the motor
	 * @return a value from -100 to 100 representing effort
	 *        0 is brake
	 *        100 is full speed clockwise
	 *        -100 is full speed counter clockwise
	 */
	float getEffortPercent()
	{
		return getEffort() * 100;
	}

	// sets the motor so that positive is anti-clockwise
	bool setReverse(bool rev = true) {return isReversed = rev;}

protected:

	bool isReversed = false;

	/**
	 * Sets the desired effort. The change in actual effort is restricted to avoid jerk.
	 */
	void setTargetEffort(float effort)
	{
		targetEffort = effort;
	}

	/**
	 * Called from loop() for all motors in the motor list upon interupt (currently every ms).
	 * Overridden by derived classes to implement control methods.
	 */
	virtual void process(void);

private:
	/* Sets the nitty-gritty of the motor. Only called from process().

	 * effort of the motor
	 * @param a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 * @note this should only be called from the PID thread
	 */
	void setEffortLocal(float effort);

	/**
	 * Called upon interrupt. Cycles through motors in motorList and calls process() for each.
	 */
	static void loop();
	
	/**
	 * @param PWMgenerationTimer the timer to be used to generate the 20khz PWM
	 */
	static void allocateTimer(int PWMgenerationTimer);

	friend void onMotorTimer(void* param);
};

#endif 
