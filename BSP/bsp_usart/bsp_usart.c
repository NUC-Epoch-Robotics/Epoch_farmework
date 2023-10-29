/**
  ******************************************************************************
  * @file     bsp_usart.c
  * @author   Guoyinin
  * @version  v1.0
  * @date     完成时间
  * @brief    简单介绍
  ******************************************************************************
  * @attention
  * 注意事项
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "string.h"
#include "stdlib.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

static uint8_t idx=0;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};
/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  注册一个usart实例
  */
USARTInstance *USART_Register(USART_Init_Config_s *init_config)
{
    if(idx > DEVICE_USART_CNT)//定义实例过多
    {
      return NULL;
    }

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->usart_module_callback = init_config->usart_module_callback;

    usart_instance[idx++]=instance;

    return instance;
}
/**
  * @brief  串口发送
  */
void USART_Send(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size,100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        break;
    }
}

/**
  * @brief  串口接收
  */
void USART_Receive(USARTInstance *_instance, uint8_t *receive_buf, uint16_t receive_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Receive(_instance->usart_handle, receive_buf, receive_size,100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Receive_IT(_instance->usart_handle, receive_buf, receive_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Receive_DMA(_instance->usart_handle, receive_buf, receive_size);
        break;
    default:
        break;
    }
}

/**
  * @brief  串口接收中断
  */
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {  
    USARTInstance *usart;
    for(uint8_t i=0;i<idx;i++)
    {
      usart=usart_instance[i];
      if(usart->usart_handle == huart && usart->usart_module_callback != NULL)
      {
        usart->usart_module_callback(usart);
        return ;
      }
    }
 }
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
