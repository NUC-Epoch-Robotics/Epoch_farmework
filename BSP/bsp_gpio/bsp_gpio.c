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
#include "stdlib.h"
/* Private  typedef -----------------------------------------------------------*/
static uint8_t idx;
static GPIOInstance *gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};
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

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
    GPIOInstance *ins = (GPIOInstance *)malloc(sizeof(GPIOInstance));
    memset(ins, 0, sizeof(GPIOInstance));

    ins->GPIOx = GPIO_config->GPIOx;
    ins->GPIO_Pin = GPIO_config->GPIO_Pin;
    ins->pin_state = GPIO_config->pin_state;
    ins->exti_mode = GPIO_config->exti_mode;
    ins->id = GPIO_config->id;
    ins->gpio_model_callback = GPIO_config->gpio_model_callback;
    gpio_instance[idx++] = ins;
    return ins;
}

// ----------------- GPIO API -----------------
// ���Ƕ�HAL����ʽ�ϵķ�װ,������������GPIO state����,����ֱ�Ӷ�ȡstate

void GPIOToggel(GPIOInstance *_instance)
{
    HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIOSet(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_SET);
}

void GPIOReset(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_RESET);
}

GPIO_PinState GPIORead(GPIOInstance *_instance)
{
    return HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
}
                
/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/
