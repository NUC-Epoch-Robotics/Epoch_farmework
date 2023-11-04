/**
  ******************************************************************************
  * @file       bsp_gpio.h
  * @author     CoopLL
  * @version    V1.0
  * @date       Oct-26-2023
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_GPIO_h
#define __BSP_GPIO_h

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/**
 * @brief �����ж��ж���Դ,ע���CUBEMX������һ��
 *
 */
typedef enum
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIOʵ���ṹ�嶨��
 *
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // ����״̬,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // �ⲿ�ж�ģʽ not frequently used
    uint16_t GPIO_Pin;          // ���ź�,
    // ��Щ������stm32f4xx_hal_gpio.h�ж���ĺ�!!! һ��Ҫע��
    // ���ȡ�����ֵ���ʱ����
    void (*gpio_model_callback)(struct tmpgpio *); // exti�жϻص�����
    void *id;                                      // ���ֲ�ͬ��GPIOʵ��

} GPIOInstance;

typedef struct 
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // ����״̬,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // �ⲿ�ж�ģʽ not frequently used
    uint16_t GPIO_Pin;          // ���ź�,
    void (*gpio_model_callback)(struct tmpgpio *);
    void *id; 
}GPIO_Init_Config_s;
/* Exported constants --------------------------------------------------------*/
#define GPIO_MX_DEVICE_NUM 10
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

