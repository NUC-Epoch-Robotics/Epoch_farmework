/**
  ******************************************************************************
  * @file     DJI_motor.c
  * @author   Guoyinin
  * @version  v1.0
  * @date     完成时间
  * @brief    简单介绍
  ******************************************************************************
  * @attention
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "DJI_motor.h"
#include "string.h"
#include "stdlib.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

//static uint8_t idx = 0;
//static DJIMotorInstance *DJI_motor_instance[DJI_MOTOR_CNT] = {NULL}; 
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/


/**
  * @brief  解析电机读到的数据，作为电机的can回调函数
  * @param instance can实例
  */
void Decode_Motor_Data(CANInstance *can_instance)
{
    uint8_t *rx_buff=can_instance->rx_buff;
    DJI_Motor_Instance *motor = (DJI_Motor_Instance *)can_instance->id;
    Motor_Measure_s *measure = &motor->motor_measure;

    measure->last_ecd = measure->ecd;                                   
    measure->ecd = (uint16_t)((rx_buff)[0] << 8 | (rx_buff)[1]);            
    measure->speed_rpm = (uint16_t)((rx_buff)[2] << 8 | (rx_buff)[3]);      
    measure->given_current = (uint16_t)((rx_buff)[4] << 8 | (rx_buff)[5]);  
    measure->temperate = (rx_buff)[6];                                   
    
}


/**
  * @brief  电机初始化
  * @param DJI_motor_config 初始化结构体
  * @retval 电机实例
  * @note 使用该函数应该提前给DJI_motor_config中motor_type以及can_handle、tx_id赋值
  */
DJI_Motor_Instance *DJI_Motor_init(Motor_Init_Config_s *DJI_motor_config)
{
  DJI_Motor_Instance *instance = (DJI_Motor_Instance *)malloc(sizeof(DJI_Motor_Instance));
  memset(instance, 0, sizeof(DJI_Motor_Instance));

  instance->motor_type = DJI_motor_config->motor_type; 

  DJI_motor_config->can_init_config.can_module_callback= Decode_Motor_Data;
  DJI_motor_config->can_init_config.id=instance;
  instance->motor_can_instance=CANRegister(&DJI_motor_config->can_init_config);

  //DJI_motor_instance[idx++]=instance;
  
  return instance;
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      hcan: 电机实例 
  * @param[in]      motor_type 电机类型
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(CAN_HandleTypeDef *hcan,Motor_Type_e motor_type,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    while ( !(HAL_CAN_GetTxMailboxesFreeLevel(hcan)) ){} //等待空邮箱
    
    uint32_t send_mail_box;
    uint8_t  chassis_can_send_data[8];
    CAN_TxHeaderTypeDef  chassis_tx_message;
    switch(motor_type)
    { 
      case M3508:
      case M2006:
        chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
        chassis_tx_message.IDE = CAN_ID_STD;
        chassis_tx_message.RTR = CAN_RTR_DATA;
        chassis_tx_message.DLC = 0x08;
        chassis_can_send_data[0] = motor1 >> 8;
        chassis_can_send_data[1] = motor1;
        chassis_can_send_data[2] = motor2 >> 8;
        chassis_can_send_data[3] = motor2;
        chassis_can_send_data[4] = motor3 >> 8;
        chassis_can_send_data[5] = motor3;
        chassis_can_send_data[6] = motor4 >> 8;
        chassis_can_send_data[7] = motor4;
        HAL_CAN_AddTxMessage(hcan, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
      break;

    default:
      break;  

  }
}            
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
