#include "Chassis.h"

/**
 * WORK IN PROGRESS
 */

static TaskHandle_t complexHandlerTask;

void onMotorTimer(void* param)
{
	ESP_LOGI(TAG, "Starting the PID loop thread");
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = 1;
	while(true)
	{
		vTaskDelayUntil(&xLastWakeTime, xInterval);
		chassis.MotorHandler();
	}
	ESP_LOGE(TAG, "ERROR Pid thread died!");
}
/**
 * @param PWMgenerationTimer the timer to be used to generate the 20khx PWM
 * @param controllerTimer a timer for PID, velocity measurment, and trajectory planning
 */
void Chassis::allocateTimer(int PWMgenerationTimer)
{
	if (!timerAllocated)
	{
		//ESP32PWM::allocateTimer(PWMgenerationTimer);
		xTaskCreatePinnedToCore(onMotorTimer, "PID loop Thread", 8192, NULL, 1,
								&complexHandlerTask, 0);
	}
	timerAllocated = true;
}

void Chassis::MotorHandler(void)
{
    if(!timerAllocated) allocateTimer(0);

    leftMotor.process();
    rightMotor.process();
}


Chassis::Chassis(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)
{
    
}

void Chassis::init(void)
{
	allocateTimer(0); // used by the DC Motors
	leftMotor.attach();
    rightMotor.attach();
}

void Chassis::loop(void)
{
}
