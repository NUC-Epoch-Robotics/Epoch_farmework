/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_h
#define __BSP_USART_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "usart.h"
/* Exported macro ------------------------------------------------------------*/

#define USART_RXBUFF_LIMIT 256 //缓冲区最大限制
#define DEVICE_USART_CNT 4     // A板至多分配4个串口
/* Exported types ------------------------------------------------------------*/
/*usart实例*/
typedef struct tmp_usart
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的串口句柄
    void (*usart_module_callback)(struct tmp_usart *);       // 解析收到的数据的回调函数
} USARTInstance;

/* 发送、接收模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    void (*usart_module_callback)(USARTInstance *);       // 解析收到的数据的回调函数
} USART_Init_Config_s;
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void USART_Send(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);

void USART_Receive(USARTInstance *_instance, uint8_t *receive_buf, uint16_t receive_size, USART_TRANSFER_MODE mode);

USARTInstance *USART_Register(USART_Init_Config_s *init_config);
#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

