/*
 * Motor.cpp
 *
 *  Created on: May 31, 2020
 *      Author: hephaestus
 */

#include <MotorSpeed.h>

bool MotorBase::timersAllocated = false;
MotorBase* MotorBase::motorList[MAX_POSSIBLE_MOTORS] = {
	NULL,
};
static TaskHandle_t complexHandlerTask;

void onMotorTimer(void* param)
{
	ESP_LOGI(TAG, "Starting the PID loop thread");
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = 10;
	while(true)
	{
		vTaskDelayUntil(&xLastWakeTime, xInterval);
		MotorBase::loop();
	}
	ESP_LOGE(TAG, "ERROR Pid thread died!");
}
/**
 * @param PWMgenerationTimer the timer to be used to generate the 20khx PWM
 * @param controllerTimer a timer for PID, velocity measurment, and trajectory planning
 */
void MotorBase::allocateTimer(int PWMgenerationTimer)
{
	if (!MotorBase::timersAllocated)
	{
		//ESP32PWM::allocateTimer(PWMgenerationTimer);
		xTaskCreatePinnedToCore(onMotorTimer, "PID loop Thread", 8192, NULL, 1,
								&complexHandlerTask, 0);
	}
	MotorBase::timersAllocated = true;
}

MotorBase::MotorBase(int pwmPin, int dirPin)
{
	MotorPWMPin = pwmPin;
	directionPin = dirPin;
}

MotorBase::~MotorBase()
{
	pwm.detachPin(MotorPWMPin);
}

/**
 * Loop function
 * this method is called by the timer to run the PID control of the motors and ensure strict timing
 *
 */
void MotorBase::loop()
{		
	for (int i = 0; i < MAX_POSSIBLE_MOTORS; i++)
	{
		if (MotorBase::motorList[i] != NULL)
		{
			MotorBase::motorList[i]->process();
		}
	}
}

void MotorBase::process(void)
{
	if (targetEffort > currentEffort + DELTA_EFFORT)
		currentEffort += DELTA_EFFORT;
	else if (targetEffort < currentEffort - DELTA_EFFORT)
		currentEffort -= DELTA_EFFORT;
	else
		currentEffort = targetEffort;

	// invert the effort so that the set speed and set effort match
	setEffortLocal(currentEffort);
}

void MotorBase::attach(void)
{
	if (isAttached)
		return;
	isAttached = true;
	// Motor timer must be allocated and the thread must be started before starting
	if (!MotorBase::timersAllocated)
	{
		MotorBase::allocateTimer(0); // used by the DC Motors
	}

	pwm.attachPin(MotorPWMPin, 20000, 12);
	pinMode(directionPin, OUTPUT);

	// add the motor to the list of timer based controls
	for (int i = 0; i < MAX_POSSIBLE_MOTORS; i++)
	{
		if (MotorBase::motorList[i] == NULL)
		{
			//			String message ="Allocating Motor " + String(i) + " on PWM "+ String(MotorPWMPin);
			//			ESP_LOGI(TAG,message.c_str());
			MotorBase::motorList[i] = this;
			return;
		}
	}
}

/*
 *  \brief effort of the motor, proportional to PWM
 *
 * @param effort a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
void MotorBase::setEffort(float effort)
{
	if (!isAttached)
		attach();

	if (effort > 1)
		effort = 1;
	if (effort < -1)
		effort = -1;
	//portENTER_CRITICAL(&mmux);
	targetEffort = effort;
	//portEXIT_CRITICAL(&mmux);
}
/*
 * effort of the motor
 * @return a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
float MotorBase::getEffort()
{
	if (!isAttached)
		attach();
	return currentEffort;
}
/*
 * effort of the motor
 * @param effort a value from -1 to 1 representing effort
 *        0 is brake
 *        1 is full speed clockwise
 *        -1 is full speed counter clockwise
 */
void MotorBase::setEffortLocal(float effort)
{
	if (!isAttached)
		attach();

	if (effort > 1)
		effort = 1;
	if (effort < -1)
		effort = -1;
	if (effort > 0)
		digitalWrite(directionPin, LOW);
	else
		digitalWrite(directionPin, HIGH);
	pwm.writeScaled(fabs(effort));
}
/**
 * getDegreesPerSecond
 *
 * This function returns the current speed of the motor
 *
 * @return the speed of the motor in degrees per second
 */
