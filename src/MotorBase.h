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

const float DELTA_EFFORT = 0.0025;

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
class MotorBase
{
protected:
	enum MOTOR_STATE {MOTOR_UNATTACHED, MOTOR_DIRECT_CTRL, MOTOR_CLOSED_LOOP_CTRL};

private:
	/**
	 * the object that produces PWM for motor speed
	 */
	ESP32PWM pwm;
	/**
	 * True if the motor has been attached
	 */
	//bool isAttached = false;

protected:
	MOTOR_STATE motorState = MOTOR_UNATTACHED;

protected:
	/*
	 * effort of the motor
	 * @param a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 * @note this should only be called from the PID thread
	 */
	void setEffortLocal(float effort);

private:
	/**
	 * @param PWMgenerationTimer the timer to be used to generate the 20khz PWM
	 */
	static void allocateTimer(int PWMgenerationTimer);
	/**
	 * this is a flag to switch between using the PID controller, or allowing the user to set effort 'directly'
	 *
	 */

	float targetEffort = 0;
	/**
	 * variable for caching the current effort being sent to the PWM/direction pins
	 */
	float currentEffort = 0;

private:
	/**
	 * GPIO pin number of the motor PWM pin
	 */
	int MotorPWMPin = -1;
	/**
	 * GPIO pin number of the motor direction output flag
	 */
	int directionPin = -1;

	static bool timersAllocated;
	/**
	 * This is a list of all of the Motor objects that have been attached. As a motor is attached,
	 *  it adds itself to this list of Motor pointers. This list is read by the PID thread and each
	 *  object in the list has loop() called.
    */
    
	static MotorBase* motorList[MAX_POSSIBLE_MOTORS];

public:

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
	MotorBase(int pwmPin, int dirPin);
	virtual ~MotorBase();

	/**
	 * \brief Attach the motors hardware
	 *
	 * This attaches the motors to the hardware ports that were saved in the constructor
	 * @note this must only be called after timers are allocated via Motor::allocateTimers(int PWMgenerationTimer)
	 *
	 */

protected:
	virtual bool attach(void);
	/*
	 *  \brief effort of the motor, proportional to PWM
	 *
	 * @param effort a value from -1 to 1 representing effort
	 *        0 is brake
	 *        1 is full speed clockwise
	 *        -1 is full speed counter clockwise
	 */

public:
	void setEffort(float effort);
	/*
	 * effort of the motor
	 * @param percent a value from -100 to 100 representing effort
	 *        0 is brake
	 *        100 is full speed clockwise
	 *        -100 is full speed counter clockwise
	 */
	void setEffortPercent(float percent)
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

private:
	static void loop();
	virtual void process(void);

	friend void onMotorTimer(void* param);

};

#endif /* LIBRARIES_RBE1001LIB_SRC_MOTOR_H_ */
