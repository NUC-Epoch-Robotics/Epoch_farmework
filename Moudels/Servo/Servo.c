/**
  ******************************************************************************
  * @file     Servo.c
  * @author   
  * @version  
  * @date     
  * @brief    通过LSC控制板控制多路舵机
  ******************************************************************************
  * @attention
  * 更换端口要更改Servo.h中的宏定义
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "Servo.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/

//宏函数 获得A的低八位
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的高八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//限幅
#define LIMIT(val,min,max)   \
    {                        \
        if (val > max)       \
        {                    \
            val = max;       \
        }                    \
        else if (val < min)  \
        {                    \
            val = min;       \
        }                    \
    }
/* Private  variables ---------------------------------------------------------*/

uint8_t send_data[128];
//uint8_t receive_data[16];
servo_t servo[SERVO_NUM];

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/********************LSCk控制板控制多路舵机***************************************/

/**
  * @brief  舵机初始化
  */
void servo_init(servo_t *servo,uint8_t id,servo_direction direction,uint16_t time)
{
	servo->id = id;
	servo->direction = direction;
	servo->time = time;

//	move_servo(servo,SERVO_CLOSE);
}


/**
  * @brief  控制单个舵机
  * @param servo 舵机的结构体
  */
void move_servo(servo_t *servo,uint16_t angle)
{
	if (servo->id > 31 || !(servo->time > 0)) 
	{  
		//舵机ID不能打于31,可根据对应控制板修改
		return;
	}

	servo->angle=angle;

	calculate_position(servo);

	send_data[0] = send_data[1] = FRAME_HEADER;    //填充帧头
	send_data[2] = 8;
	send_data[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
	send_data[4] = 1;                        //要控制的舵机个数
	send_data[5] = GET_LOW_BYTE(servo->time);       //取得时间的低八位
	send_data[6] = GET_HIGH_BYTE(servo->time);      //取得时间的高八位
	send_data[7] = servo->id;                  //舵机ID
	send_data[8] = GET_LOW_BYTE(servo->position);   //取得目标位置的低八位
	send_data[9] = GET_HIGH_BYTE(servo->position);  //取得目标位置的高八位
	
	HAL_UART_Transmit(&SERVO_USART , send_data , 10, 0xFFFF);
}

/**
  * @brief  控制多个舵机
  * @param Num 舵机的id
  * @param Time 时间
  * @param 其余参数 舵机id、舵机角度...
  * @note 后续会考虑进行封装
  */
void move_servos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0,j=0;
	uint16_t temp;
	va_list arg_ptr;  

	va_start(arg_ptr, Time); //取得可变参数首地址
	if (Num < 1 || Num > 32) 
	{
		return;               //舵机数不能为零和大与32，时间不能小于0
	}
	send_data[0] = send_data[1] = FRAME_HEADER;      //填充帧头
	send_data[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
	send_data[3] = CMD_SERVO_MOVE;             //舵机移动指令
	send_data[4] = Num;                        //要控制舵机数
	send_data[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
	send_data[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位

	for (i = 0; i < Num; i++) 
	{
		//从可变参数中取得并循环填充舵机ID和对应目标位置
		temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
		send_data[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标角度
		for(j=0;j<SERVO_NUM;j++) //将角度转化为位置
		{
			if(servo[j].id == send_data[index-1])
			{
				servo[j].angle=temp;
				temp=calculate_position(&servo[j]);
			}
		}
		send_data[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
		send_data[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
	}

	va_end(arg_ptr);  //置空arg_ptr

	HAL_UART_Transmit(&SERVO_USART , send_data , send_data[2]+2, 0xFFFF);
}

/**
  * @brief  舵机位置解算,将旋转的角度转换为舵机位置
  */
uint16_t calculate_position(servo_t *servo)
{
	LIMIT(servo->angle,0,180);

	switch (servo->direction)
	{
		//舵机反转  0°位置为2500，180°位置为500
		case REVERSE:
			servo->position=2500-(float)servo->angle/180.0f*(2500-500);
			break;
		//舵机正转  0°位置为500，180°位置为2500	
		case FORWARD:
			servo->position=500+(float)servo->angle/180.0f*(2500-500);
			break;
		
		default:
			break;
	}

	return servo->position;

}

/***************************************pwm控制舵机***************************************/
/**
  * @brief  舵机初始化
  */
void servo_init_pwm(servo_t *servo,TIM_HandleTypeDef htim,uint8_t tim_channel,servo_direction direction)
{
	servo->htim=htim;
	servo->tim_channel=tim_channel;
	servo->direction=direction;
	
	HAL_TIM_PWM_Start(&servo->htim,servo->tim_channel);
}


/**
  * @brief  用pwm直接控制舵机
  */
 void move_servo_pwm(servo_t *servo,uint16_t angle)
 {
	servo->angle=angle;
	calculate_position(servo);
	__HAL_TIM_SET_COMPARE(&servo->htim,servo->tim_channel,servo->position);
 }
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
