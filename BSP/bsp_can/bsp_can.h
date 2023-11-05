/**
  ******************************************************************************
  * @file     bsp_can.h
  * @author   Junshuo
  * @version  V1.0
  * @date     Mar-28-2023
  * @brief    This file contains the headers of bsp_can.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "can.h"
//#include "CAN_defines.h"
/* Exported types ------------------------------------------------------------*/
typedef struct CAN_RX_Typedef
{
	int message_timestamp;
	int data_length;
	int data[8];
	int filter_index;
	int frame_type;
	int id_type;
	int32_t ID;
}CAN_RX_Typedef;

typedef struct CAN_TX_Typedef
{
	int id_type;
	int frame_type;
	int send_timestamp;
	int32_t ID;
	int data_length;
	uint8_t data[8];
}CAN_TX_Typedef;
typedef union
{
	
	uint8_t data8[8];	
	int16_t data16[4];	
	int data32[2];
	float dataf[2];
	
}UnionDataType;

typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint8_t rx_len;                // 接收长度,可能为0-8
    CAN_RxHeaderTypeDef rxconf;   //CAN报文接收配置
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // 处理接收数据的回调函数
    void *id;                      //区分不同的can实例，module层用不同id选择操控的电机
} CANInstance;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    uint32_t tx_id;                             // 发送id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id ;                       //区分不同的can实例，module层用不同id选择操控的电机
} CAN_Init_Config_s;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define DEVICE_CAN_CNT 16
/* Exported functions ------------------------------------------------------- */

CANInstance *CANRegister(CAN_Init_Config_s *init_config);

void CAN_Filter_Init(void);

void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox);
	
void CAN_Send_Packet(CANInstance *instance);


//void CAN_Get_Packet(CAN_HandleTypeDef *hcan, CAN_RX_Typedef *rx);


#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

