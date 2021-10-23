#include "Chassis.h"
#include "ir_codes.h"

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
		robot.MotorHandler();
	}
	ESP_LOGE(TAG, "ERROR Pid thread died!");
}
/**
 * @param PWMgenerationTimer the timer to be used to generate the 20khx PWM
 * @param controllerTimer a timer for PID, velocity measurment, and trajectory planning
 */
void Robot::allocateTimer(int PWMgenerationTimer)
{
	if (!timerAllocated)
	{
		//ESP32PWM::allocateTimer(PWMgenerationTimer);
		xTaskCreatePinnedToCore(onMotorTimer, "PID loop Thread", 8192, NULL, 1,
								&complexHandlerTask, 0);
	}
	timerAllocated = true;
}

void Robot::MotorHandler(void)
{
    if(!timerAllocated) allocateTimer(0);

    leftMotor.process();
    rightMotor.process();
}


Robot::Robot(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)
{
    
}

void Robot::init(void)
{
	allocateTimer(0); // used by the DC Motors
	leftMotor.attach();
    rightMotor.attach();

    mb_ez1.Init(USE_ECHO);// | USE_CTRL_PIN);
    irDecoder.init(IR_PIN);
}

void Robot::loop(void)
{
    int16_t keyCode = irDecoder.getKeyCode();
    if(keyCode != -1) HandleIRPress(keyCode);

    //mb_ez1.CheckPingTimer();
    uint16_t pulseLen = mb_ez1.CheckEcho();
    if(pulseLen)
    {
        Serial.print(millis());
        Serial.print('\t');
        Serial.print(pulseLen);
        Serial.print('\t');
        Serial.print(pulseLen * 0.017);
        Serial.print('\t');
        
        wallFollower.ProcessDistanceReading(pulseLen * 0.017);

        Serial.print('\n');
    }
}

void Robot::HandleIRPress(int16_t key)
{
    Serial.println(key);
    if(key == INFO)
    {
        robotState = ROBOT_IDLE;
    }

    switch(robotState)
    {
        case ROBOT_IDLE:
            if(key == PREV)
            {
                robotState = ROBOT_WALL_FOLLOWING;
            }
            break;
        case ROBOT_DEAD_RECKONING:
            break;
        case ROBOT_WALL_FOLLOWING:
            wallFollower.HandleKeyPress(key);
            break;
        default:
            break;
    }
}