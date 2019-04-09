#ifndef  __FREERTOS_TASK_H
#define  __FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "imu.h"
#include "MotorCAN.h"
#include "event_groups.h"

struct IMU_EXT{
	int16_t Gyo_X;
	int16_t Gyo_Z;
	u8 state;
	float angle;
	uint32_t onlinecnt;
};
typedef enum
	{
      KEY_UP = 0,
	  KEY_DOWN ,
	  KEY_HOLE ,
	  KEY_CLICK,
	  KEY_ONE,
	  KEY_TWO,
	  KEY_LONG
}key_state;

typedef enum{
	  KEY_NO_CLICK = 0 ,
      KEY_ONE_CLICK    ,
	  KEY_DOUBLE_CLICK ,
	  KEY_LONG_CLICK   ,
}key_event;

typedef struct KEY_
{
    key_event Key_Event;
	key_state Key_State;
	int16_t cnt;
}Key_ide,*p_Key_ide;


void FreeRTOS_init(void);

extern  xSemaphoreHandle DbusParseSem;
extern  xSemaphoreHandle ImuDataSem;

extern  QueueHandle_t Can1ReceiveQueue;
extern  QueueHandle_t Can2ReceiveQueue;

extern IMUDATA  Mpu6500Data;

extern struct IMU_EXT Imu_ext;

extern Motor_SpeedLoopData_t CM_Motor[4];
extern Motor_Posi_A_Speed_LoopData_t YawMotor;
extern Motor_Posi_A_Speed_LoopData_t PitchMotor;
extern Motor_SpeedLoopData_t FireMotor;
extern EventGroupHandle_t CmGimbalOutputEnt;

static void Can1ReceiveTask(void *pvParameters);
void Key_Scan(uint16_t Key_Value,p_Key_ide key_id);
#endif

