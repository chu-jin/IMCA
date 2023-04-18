#ifndef MF9025_H 
#define MF9025_H 
#include "main.h"
#define RS_ON 0

#define READ_PID     0X30
#define READ_ACCEL   0X33
#define READ_ENCODER 0X90
#define SET_MECHANICAL_ANGLE 0X95
#define READ_STATE1_FAULT_FLAG 0X9A
#define CLAER_FAULT_FLAG 0X9B
#define READ_STATE2_FLAG 0X9C
#define READ_STATE3_FLAG 0X9D
#define MOTOR_OFF  0X80
#define MOTOR_STOP 0X81
#define MOTOR_RUN  0X88

void MF_COMMAND(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, uint8_t command);//命令发送
void MF_speedControl(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, int32_t speedControl);//速度控制
void MF_Multi_angleControl_1(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, int32_t angleControl);//多圈角度控制
void MF_Multi_angleControl_2(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, uint16_t maxSpeed, int32_t angleControl);//多圈角度控制，但是可以限速
void MF_Single_loop_angleControl_1(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl);//单圈角度控制
void MF_Single_loop_angleControl_2(CAN_HandleTypeDef *hcan,uint8_t Motor_ID, uint8_t spinDirection, uint16_t maxSpeed, uint16_t angleControl);//单圈角度控制，但是可以限速
void MF_Motor_Off(CAN_HandleTypeDef *hcan,uint8_t Motor_ID);

#if RS_ON
void RS_angleControl_1(uint8_t Motor_ID, int64_t angleControl_1);
void RS_angleControl_2(uint8_t Motor_ID, int64_t angleControl_1, uint32_t maxSpeed);
void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, int16_t angleControl_1);
void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed);
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl);
void RS_Motor_Off(uint8_t Motor_ID);

void Mode_Select();
#endif

#endif 
