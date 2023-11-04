/**
****************************(C) COPYRIGHT 2023 EPOCH****************************
* @file       ops_receive.c/h
* @brief      接收并处理Action平面定位系统OPS通过串口发送的数据；
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Apr-06-2023     Junshuo         1. 移植。
*
****************************(C) COPYRIGHT 2023 EPOCH****************************
*/

#ifndef OPS_RECEIVE_H
#define OPS_RECEIVE_H

void ops_data_analyse(uint8_t rec);
void ops_reset(void);
// float GetX(chassis_move_t *chassis_ops);
// float GetY(chassis_move_t *chassis_ops);
void Update_X(float New_X);
void Update_Y(float New_Y);
void Update_A(float New_A);

#endif
