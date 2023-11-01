/**
  ******************************************************************************
  * @file     bsp_can.c
  * @author   Junshuo
  * @version  V1.0
  * @date     Mar-28-2023
	* @brief      1. CAN初始化函数,使能CAN1和CAN2的中断，使能CAN1和CAN2的滤波器
	*             2. CAN发送函数
	*             3. CAN接收函数
  ******************************************************************************
  * @attention
  * 注意事项
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "bsp_can.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/

static uint8_t idx=0;
static CANInstance *can_instance[DEVICE_CAN_CNT] = {NULL};
/* Extern   variables ---------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Extern   function prototypes -----------------------------------------------*/
/* Private  function prototypes -----------------------------------------------*/
/* Private  functions ---------------------------------------------------------*/

/**
  * @brief          注册CAN实例
  * @param[in]      instance:CAN实例
  * @param[in]     	tx:发送数据
	* @note					  ODrive通信库里的屎山代码
  */
CANInstance *CANRegister(CAN_Init_Config_s *init_config)
{
    if(idx > DEVICE_CAN_CNT)//定义实例过多
    {
      return NULL;
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = init_config->can_handle;
    instance->tx_id=init_config->tx_id;
    instance->can_module_callback = init_config->can_module_callback;
    instance->id=init_config->id;

    can_instance[idx++]=instance;

    return instance;
}
/**
  * @brief          CAN初始化，在main函数中调用
  */
void CAN_Filter_Init(void)
{

	CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	  can_filter_st.FilterBank = 14;
	  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	  HAL_CAN_Start(&hcan2);
	  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  
}


/**
  * @brief          发送CAN数据包
  * @param[in]      instance:CAN实例
  */
void CAN_Send_Packet(CANInstance *instance)
{
	while ( !(HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle)) ){} //等待空邮箱

  //目前只用标准帧、数据帧、长度为8
  instance->txconf.StdId=instance->tx_id;
  instance->txconf.IDE=CAN_ID_STD;
  instance->txconf.RTR=CAN_RTR_DATA;
  instance->txconf.DLC=0x08;

	HAL_CAN_AddTxMessage(instance->can_handle, &instance->txconf, instance->tx_buff, &instance->tx_mailbox);
}

/**
  * @brief          fifo接收中断
  * @param[in]      _hcan CAN实例
  * @param[in]     	fifox fifo0 或 fifo1
  */
void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; 
    uint8_t can_rx_buff[8];

    HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
    for (uint8_t i = 0; i < idx; ++i)
    { 
      // 两者相等说明这是要找的实例
      if (_hcan == can_instance[i]->can_handle && can_instance[i]->can_module_callback != NULL)
      {
            can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
            memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
            can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
            can_instance[i]->rxconf=rxconf;
          return;
      }
    }
    
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANFIFOxCallback(hcan,CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CANFIFOxCallback(hcan,CAN_RX_FIFO1);
}
/**
  * @brief          [用于ODrive通信]获取CAN数据包，在can中断中调用
  * @param[in]      hcan:CAN句柄
  * @param[out]     rx:接收数据包
	* @note					  ODrive通信库里的屎山代码
  */
//void CAN_Get_Packet(CAN_HandleTypeDef *hcan, CAN_RX_Typedef *rx)
//{

//	int frame_type = 0;
//	int id_type = 0;


//	id_type =  (CAN1 -> sFIFOMailBox[0].RIR & CAN_RI0R_IDE_Msk) >> CAN_RI0R_IDE_Pos ;
//	frame_type = (CAN1 -> sFIFOMailBox[0].RIR & CAN_RI0R_RTR_Msk) >> CAN_RI0R_RTR_Pos ;

//	if(id_type)
//	{
//		//Extended ID
//		rx->id_type = CAN_ID_Extended;
//		rx->ID = (hcan -> Instance -> sFIFOMailBox[0].RIR & CAN_RI0R_EXID_Msk) >> CAN_RI0R_EXID_Pos;
//	}
//	else
//	{
//		//Standard ID
//		rx->id_type = CAN_ID_Standard;
//		rx->ID = (hcan -> Instance -> sFIFOMailBox[0].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
//	}

//	if(frame_type)
//	{
//		//RTR Frame
//		rx->frame_type = CAN_Frame_Remote;
//		rx->data_length = (hcan -> Instance -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
//		hcan -> Instance -> RF0R |= CAN_RF0R_RFOM0;
//		while((hcan -> Instance -> RF0R & CAN_RF0R_RFOM0)){}
//	}
//	else
//	{
//		//Data Frame
//		rx->frame_type = CAN_Frame_Data;
//		rx->data_length = (hcan -> Instance -> sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos;
//		for(int i = 0; i < rx->data_length; i++)
//		{
//			if(i < 4)
//			{
//				rx->data[i] =  (hcan -> Instance -> sFIFOMailBox[0].RDLR & ( 0xFF << (8*i))) >> (8*i);
//			}
//			else
//			{
//				rx->data[i] =  (hcan -> Instance -> sFIFOMailBox[0].RDHR & ( 0xFF << (8*(i-4)))) >> (8*(i-4));
//			}
//		}

//		hcan -> Instance -> RF0R |= CAN_RF0R_RFOM0;	}

//}


/************************ (C) COPYRIGHT 2023 EPOCH *****END OF FILE****/

