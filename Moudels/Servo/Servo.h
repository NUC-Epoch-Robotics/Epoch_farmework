/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of Servo.c
  ******************************************************************************
  * @attention
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_h
#define __SERVO_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "usart.h"
#include "tim.h"
#include <stdarg.h>
/* Exported types ------------------------------------------------------------*/

enum //舵机id枚举
{
	SERVO0=0,
  SERVO1=1,
  SERVO2=2,
  SERVO3=3
};

typedef enum //舵机转动的方向
{
  FORWARD,
  REVERSE
}servo_direction;

typedef struct //舵机结构体
{
  uint8_t id; //舵机的id
  uint16_t position;//舵机位置
  uint16_t angle;//舵机角度
  uint16_t time; //旋转的时间(单位ms)
  servo_direction direction;//舵机转动的方向
  
  TIM_HandleTypeDef htim;
  uint8_t tim_channel;
}servo_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/*******************************以下参数可更改**********************************/
#define SERVO_USART huart7 //舵机用的串口
#define SERVO_TIM htim2 //舵机用的定时器

#define SERVO_NUM 3 //舵机数量

#define SERVO_TIME 500 //舵机转动的时间（单位ms) 时间选择不对可能导致三个舵机之间有延迟

#define SERVO_CLOSE 0 //夹爪开关时舵机的角度
#define SERVO_OPEN 90 
/******************************************************************************/

#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06     //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07    //停止动作组指令
#define CMD_ACTION_GROUP_SPEED 0x0B   //设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取电池电压指令
/* Exported functions ------------------------------------------------------- */

extern void servo_init(servo_t *servo,uint8_t id,servo_direction direction,uint16_t time);
extern void move_servo(servo_t *servo,uint16_t angle);
extern void move_servos(uint8_t Num, uint16_t Time, ...);
extern uint16_t calculate_position(servo_t *servo);
extern void servo_init_pwm(servo_t *servo,TIM_HandleTypeDef htim,uint8_t tim_channel,servo_direction direction);
extern  void move_servo_pwm(servo_t *servo,uint16_t angle);

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

