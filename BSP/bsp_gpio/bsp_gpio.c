/**
  ******************************************************************************
  * @file     bsp_gpio.c
  * @author   CoopLL
  * @version  V1.0
  * @date     Oct-26-2023
  * @brief    �򵥽���
  ******************************************************************************
  * @attention
  * ע������
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "bsp_gpio.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

/* Extern   variables ---------------------------------------------------------*/
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief  
  * @param 
  * @param 
  * @retval 
  */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // ���б�Ҫ,���Ը���pinstate��HAL_GPIO_ReadPin���ж��������ػ����½���/rise&fall��
    GPIOInstance *gpio;
    for (size_t i = 0; i < idx; i++)
    {
        gpio = gpio_instance[i];
        if (gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
        {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}
                
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
