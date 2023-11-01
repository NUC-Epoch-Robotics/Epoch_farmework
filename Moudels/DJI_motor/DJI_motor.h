/**
  ******************************************************************************
  * @file    DJI_motor.h
  * @author  Guoyinin
  * @version v1.0
  * @date    
  * @brief   This file contains the headers of DJI_motor.c
  ******************************************************************************
  * @attention
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DJI_MOTOR_h
#define __DJI_MOTOR_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "bsp_can.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} Motor_Measure_s;

typedef enum
{
  M3508,
  M2006,
  GM6020,
}Motor_Type_e;
typedef struct 
{
  CANInstance *motor_can_instance;
  Motor_Measure_s motor_measure;
  Motor_Type_e motor_type;
}DJIMotorInstance;

typedef struct
{
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define DJI_MOTOR_CNT 12
/* Exported functions ------------------------------------------------------- */

extern DJIMotorInstance *DJI_Motor_init(Motor_Init_Config_s *DJI_motor_config);

extern void Decode_Motor_Data(CANInstance *instance);

extern void CAN_cmd_chassis(DJIMotorInstance *motor_instance,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

