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
 * @brief 用于判断中断来源,注意和CUBEMX中配置一致
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
 * @brief GPIO实例结构体定义
 *
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin;          // 引脚号,
    // 这些引脚是stm32f4xx_hal_gpio.h中定义的宏!!! 一定要注意
    // 随便取个名字当临时声明
    void (*gpio_model_callback)(struct tmpgpio *); // exti中断回调函数
    void *id;                                      // 区分不同的GPIO实例

} GPIOInstance;

typedef struct 
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin;          // 引脚号,
    void (*gpio_model_callback)(struct tmpgpio *);
    void *id; 
}GPIO_Init_Config_s;
/* Exported constants --------------------------------------------------------*/
#define GPIO_MX_DEVICE_NUM 10
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif

/****************** (C) COPYRIGHT 2023 EPOCH *****END OF FILE*************/

