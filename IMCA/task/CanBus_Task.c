/**********************************************************************************************************************
 * @file  CanBus_Task.c
 * @brief CAN1滤波器的配置和CAN的开启，从CAN总线上接收报文和发送报文到CAN总线上
 *
**********************************************************************************************************************/
#include "CanBus_Task.h"
#include "main.h"
#include "RC_Task.h"
#include "balance_cal.h"
#include "user_lib.h"
#define LEFT  0
#define RIGHT 1
moto_measure_t  Gimbal_Motor[2];  //云台电机参数结构体
commucation_t Commucation_Data;

MFMOTOR_State_t CHASSIS_MOTOR[2];
MFMOTOR_State_t MFMOTOR_State_left;
MFMOTOR_State_t MFMOTOR_State_right;
uint16_t count_left  = 0;//堵转计数
uint16_t count_right = 0;//堵转计数
uint8_t closs_left = 0;//堵转标志位
uint8_t closs_right = 0;//堵转标志位
float Jscope_IMU_X;
float Jscope_IMU_Y;
float Jscope_IMU_Z;
float Jscope_Gimbal_w;
float Jscope_Gimbal_x; 

/**********************************************************************************************************************
  * @Func	 CANFilterInit
  * @Brief   CAN1滤波器配置
  * @Param	 CAN_HandleTypeDef* _hcan
  * @Retval	 None
 *********************************************************************************************************************/
void CANFilterInit(void)
{
	CAN_FilterTypeDef		CAN_FilterConfigStructure;

	/*filter config for can1*/
	CAN_FilterConfigStructure.FilterBank = 0;                      // filter 0
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;  // mask mode
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;            // set mask 0 to receive all can id
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
	CAN_FilterConfigStructure.FilterActivation = ENABLE;           // enable can filter
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;           // only meaningful in dual can mode

	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);       // init can filter
	HAL_CAN_Start(&hcan1);                                         // start can1
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);  // enable can1 rx interrupt

	/*filter config for can2*/ 
	/*can1(0-13)和can2(14-27)分别得到一半的filter*/
	CAN_FilterConfigStructure.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure);
	HAL_CAN_Start(&hcan2); 
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}


/**********************************************************************************************************************
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
 *********************************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    
    static uint16_t can_cnt;
    CAN_RxHeaderTypeDef   rx_header;
    uint8_t               rx_data[8];

    HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data); /*recive can data*/
  

    if(_hcan->Instance == CAN1)
    { 
			can_cnt++;
			switch(rx_header.StdId)
			{
				
				case MF_MOTOR_ID_LEFT:
				{
					MFMOTOR_State_left.lost_cnt=0;
					if(rx_data[0]==0x30){GetMotorPID(&CHASSIS_MOTOR[LEFT], rx_data);}
					
					if(rx_data[0]==0xA1){GetMotorstate(&CHASSIS_MOTOR[LEFT], rx_data);}//获取电机PID
				}break;
				case MF_MOTOR_ID_RIGHT:
				{
					MFMOTOR_State_right.lost_cnt=0;
					if(rx_data[0]==0x30){GetMotorPID(&CHASSIS_MOTOR[RIGHT], rx_data);}
					
					if(rx_data[0]==0xA1){GetMotorstate(&CHASSIS_MOTOR[RIGHT], rx_data);}//获取电机PID
				}break;
				case RM_MOTOR_ID_LEFT:
				{
					Gimbal_Motor[LEFT].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor[LEFT], rx_data) : GetMotorMeasure(&Gimbal_Motor[LEFT], rx_data);  /*读取yaw轴电机参数信息*/
				}break;
				case RM_MOTOR_ID_RIGHT:
				{
					Gimbal_Motor[RIGHT].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor[RIGHT], rx_data) : GetMotorMeasure(&Gimbal_Motor[RIGHT], rx_data);  /*读取yaw轴电机参数信息*/
				}break;
				#if JUDMENT_ONOFF
//				case CAN_JUDGMENT_ID1:
//				{
//					GetJudgment1(&Judgment_Limit,rx_data);  //接收裁判系统数据
//				}break;
//				case CAN_JUDGMENT_ID2:
//				{		
//					GetJudgment2(&Judgment_Limit,rx_data);
//				}break;
				#endif
			}  
    }
    else if(_hcan->Instance == CAN2)
    {
			switch(rx_header.StdId)
			{
//				case CAN2_GET_RC_ID:     //0x601 接收遥控器信息
//				{
////					 can_cnt++;
//					 Commucation_Data.lost_cnt = 0;
//					 GetCanCommunication(&remote_control, rx_data);
//				}break;
//				case CAN2_GET_RC2_ID:    //0x602 接收遥控器信息
//				{
//					 Commucation_Data.lost_cnt = 0;
//					 GetCanCommunication_two(&remote_control, rx_data);
//				}break;
				case CAN2_GIMBAL_MEASURE_ID:
				{
					 Commucation_Data.lost_cnt = 0;
		       GetGimbalMeasure(&Commucation_Data,rx_data);
					 Commucation_Data.lost_cnt=0;
				}break;
			}
        
    }
    if(can_cnt == 500)  /*用于指示CAN通信是否正常*/
    {
        can_cnt = 0;
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);  
    }
        
}

/*接收云台发送的底盘云台相对角度*/
void GetGimbalMeasure(commucation_t* data, uint8_t buff[])
{
  uint8_t yaw_relative_angle[4];
  uint8_t yaw_speed[4];
	
  yaw_relative_angle[0] = buff[0]; //底盘云台相对角度
  yaw_relative_angle[1] = buff[1];
  yaw_relative_angle[2] = buff[2];
  yaw_relative_angle[3] = buff[3];

  yaw_speed[0] = buff[4]; //底盘云台相对角度
  yaw_speed[1] = buff[5];
  yaw_speed[2] = buff[6];
  yaw_speed[3] = buff[7];
	
  CharToFloat(&(data->receive_yaw_relative_angle),yaw_relative_angle,4);
	CharToFloat(&(data->yaw_speed),yaw_speed,4);
}

/*************
新电机测试函数mf9025
************/
void SetMotorSpeed(CAN_HandleTypeDef *hcan,int32_t speedcontrol)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
      uint32_t              send_mail_box;
	tx_header.StdId = 0x143;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = 0xA2;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	tx_data[4] = *(uint8_t*)(&speedcontrol);
	tx_data[5] = *((uint8_t*)(&speedcontrol)+1);
	tx_data[6] = *((uint8_t*)(&speedcontrol)+2);
	tx_data[7] = *((uint8_t*)(&speedcontrol)+3);
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);
}	


/**********************************************************************************************************************
 * @Brief  	读取MF电机的PID
 * @Param   void
 * @Retval  void
 * @Other  
**********************************************************************************************************************/
void GetMotorPID(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[])
{
		data->angleKp = CAN_Data_rx[2];
		data->angleKi = CAN_Data_rx[3];
		data->SpeedKp = CAN_Data_rx[4];
		data->SpeedKi = CAN_Data_rx[5];
		data->iqKp 		= CAN_Data_rx[6];
		data->iqKi 		= CAN_Data_rx[7];
}	

/**********************************************************************************************************************
 * @Brief   MF电机转矩闭环控制
 * @Param   void
 * @Retval  void
 * @Other   iq = -2000~2000
**********************************************************************************************************************/
void SendMotorCommand(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int8_t command)
{
    
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];
	uint32_t              send_mail_box;
	tx_header.StdId = Motor_ID;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = command;
	tx_data[1] = 0;
	tx_data[2] = 0;
	tx_data[3] = 0;
	tx_data[4] = 0;
	tx_data[5] = 0;
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

/**********************************************************************************************************************
 * @Brief   MF电机转矩闭环控制																																											
 * @Param   void
 * @Retval  void
 * @Other   iq = -2000~2000		
**********************************************************************************************************************/
void SetMotoriq(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int16_t iq)
{
    
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];
	uint32_t              send_mail_box;
	tx_header.StdId = Motor_ID;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = 0xA1;
	tx_data[1] = 0;
	tx_data[2] = 0;
	tx_data[3] = 0;
	tx_data[4] = *(uint8_t*)(&iq);
	tx_data[5] = *((uint8_t*)(&iq)+1);
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

/**********************************************************************************************************************
 * @Brief   MF电机写入PID参数到RAM
 * @Param   void
 * @Retval  void
 * @Other  
**********************************************************************************************************************/
void SetMotorPid_RAM(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,uint8_t anglePidKp,uint8_t anglePidKi,uint8_t SpeedPidKp,uint8_t SpeedPidKi,uint8_t iqPidKp,uint8_t iqPidKi)
{
    
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];
	uint32_t              send_mail_box;
	tx_header.StdId = Motor_ID;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = 0x31;
	tx_data[1] = 0;
	tx_data[2] = anglePidKp;
	tx_data[3] = anglePidKi;
	tx_data[4] = SpeedPidKp;
	tx_data[5] = SpeedPidKi;
	tx_data[6] = iqPidKp;
	tx_data[7] = iqPidKi;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

/**********************************************************************************************************************
 * @Brief  	读取MF电机的状态
 * @Param   void
 * @Retval  void
 * @Other  
**********************************************************************************************************************/
void GetMotorstate(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[])
{
	if(CAN_Data_rx[0] == 0x9c||CAN_Data_rx[0] == 0xA1)
	{
		data->temperature = CAN_Data_rx[1];
		data->iq = ((CAN_Data_rx[3]<<8)|CAN_Data_rx[2]);
		data->speed = ((CAN_Data_rx[5]<<8)|CAN_Data_rx[4]);
		data->encoder = ((CAN_Data_rx[7]<<8)|CAN_Data_rx[6]);	
	}
}	

/**********************************************************************************************************************
 * @Brief   MF电机电流状态读取
 * @Param   void
 * @Retval  void
 * @Other  
**********************************************************************************************************************/
void ReadMotorState(CAN_HandleTypeDef *hcan,uint32_t Motor_ID)
{
    
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];
	uint32_t              send_mail_box;
	tx_header.StdId = Motor_ID;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x9c;
	tx_data[0] = 0;
	tx_data[1] = 0;
	tx_data[2] = 0;
	tx_data[3] = 0;
	tx_data[4] = 0;
	tx_data[5] = 0;
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

/**********************************************************************************************************************
* @brief    mf9025多电机电流发送 
 * @param	           
 * @retval	 None
**********************************************************************************************************************/
void SetMotorCurrent(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
      uint32_t              send_mail_box;
	tx_header.StdId = 0x280;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = *(uint8_t*)(&iq1);
	tx_data[1] = *((uint8_t*)(&iq1)+1);
	tx_data[2] = *(uint8_t*)(&iq2);
	tx_data[3] = *((uint8_t*)(&iq2)+1);
	tx_data[4] = *(uint8_t*)(&iq3);
	tx_data[5] = *((uint8_t*)(&iq3)+1);
	tx_data[6] = *((uint8_t*)(&iq4));
	tx_data[7] = *((uint8_t*)(&iq4)+1);
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

/**********************************************************************************************************************
 * @brief    接收3508电机通过CAN发过来的信息，2006电机也适用，不过2006电机没有温度值返来
 * @param	   moto_measure_t *ptr：电机参数结构体
 *           uint8_t can_rx_data[]：CAN接收数据缓存区
 * @retval	 None
**********************************************************************************************************************/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[])
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]);
	ptr->speed_rpm  = (int16_t)(can_rx_data[2]<<8 | can_rx_data[3]);
	ptr->real_current = (can_rx_data[4]<<8 | can_rx_data[5])*5.f/16384.f;
	ptr->hall = can_rx_data[6];
    
	/*编码器过零点处理*/
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
    
    /*得到转过的总角度，这个角度是相对于上电时的角度*/
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle; 
    
}



/*this function should be called after system+can init */
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[])        
{
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]);
	ptr->offset_angle = ptr->angle;
}



/**********************************************************************************************************************
 * @brief  send motor control message through can bus
 * @param  CAN_HandleTypeDef *hcan             CAN handle Structure
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  设置四个电机的电流值
 * @retval None
 * @others 标识符0x200对应前四个ID的电调，标识符0x1FF对应后四个ID的电调，一路CAN最多可以接八个电机，
 *         此函数是对应前4个ID的电调
**********************************************************************************************************************/
void SetChassisMotorCurrent(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
      uint32_t              send_mail_box;
	tx_header.StdId = 0x200;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
	tx_data[2] = (iq2 >> 8);
	tx_data[3] = iq2;
	tx_data[4] = (iq3 >> 8);
	tx_data[5] = iq3;
	tx_data[6] = (iq4 >> 8);
	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	


/**********************************************************************************************************************
 * @brief  send motor control message through can bus
 * @param  CAN_HandleTypeDef *hcan             CAN handle Structure
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  设置四个电机的电流值
 * @retval None
 * @others 标识符0x200对应前四个ID的电调，标识符0x1FF对应后四个ID的电调，一路CAN最多可以接八个电机，
 *         此函数是对应后4个ID的电调
**********************************************************************************************************************/
void SetGimbalMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
     uint32_t              send_mail_box;
	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
	tx_data[2] = (iq2 >> 8);
	tx_data[3] = iq2;
	tx_data[4] = (iq3 >> 8);
	tx_data[5] = iq3;
	tx_data[6] = (iq4 >> 8);
	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	


/*摩擦轮和拨弹轮电机的数据发送*/
void SetShootMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3)
{
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
    uint32_t              send_mail_box;
	tx_header.StdId = 0x200;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
	tx_data[2] = (iq2 >> 8);
	tx_data[3] = iq2;
	tx_data[4] = (iq3 >> 8);
	tx_data[5] = iq3;

	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);
    
}


/*云台yaw轴电机数据发送*/
void SetGimbalYawMotorValue(CAN_HandleTypeDef *hcan,s16 iq1)
{
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
    uint32_t              send_mail_box;
	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
//	tx_data[2] = (iq1 >> 8);
//	tx_data[3] = iq1;
//	tx_data[4] = (iq3 >> 8);
//	tx_data[5] = iq3;
//	tx_data[6] = (iq4 >> 8);
//	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);
    
    
}

/*云台pitch轴电机数据发送*/
void SetGimbalPitchMotorValue(CAN_HandleTypeDef *hcan,s16 iq1)
{
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
    uint32_t              send_mail_box;
	
	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
//	tx_data[2] = (iq1 >> 8);
//	tx_data[3] = iq1;
//	tx_data[4] = (iq3 >> 8);
//	tx_data[5] = iq3;
//	tx_data[6] = (iq4 >> 8);
//	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);
    
    
}

#if BOARD_COMUNICATE_ONOFF
/**********************************************************************************************************************
 * @brief  Dual master CAN communication transmission
 * @param  CAN_HandleTypeDef *hcan             CAN handle Structure
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  设置can通信发送的值
 * @retval None
 * @others 标识符0x200对应前四个ID的电调，标识符0x1FF对应后四个ID的电调，一路CAN最多可以接八个电机，
 *         此函数是对应后4个ID的电调
**********************************************************************************************************************/
void sen_can_comu_1(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
    uint32_t              send_mail_box;
	tx_header.StdId = 0x601;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
  tx_data[0] = iq1;
	tx_data[1] = iq2;
	tx_data[2] = iq3;
	tx_data[3] = iq4;
	tx_data[4] = iq5;
	tx_data[5] = iq6;
	tx_data[6] = iq7;
	tx_data[7] = iq8;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

void sen_can_comu_2(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];
    uint32_t              send_mail_box;
	tx_header.StdId = 0x602;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = iq1;
	tx_data[1] = iq2;
	tx_data[2] = iq3;
	tx_data[3] = iq4;
	tx_data[4] = iq5;
	tx_data[5] = iq6;
	tx_data[6] = iq7;
	tx_data[7] = iq8;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,&send_mail_box);

}	

void GetCanCommunication(RC_Type* rc, uint8_t buff[])
{
  /*遥控器解析*/
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
  
  rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
  
}
/**********************************************************************************************************************
 * @brief    接收CAN通信传输过来的信息
 * @param	 RC_Type* rc：遥控器参数结构体
 *            uint8_t buff[]：CAN接收数据缓存区
 * @petval	 None
**********************************************************************************************************************/
void GetCanCommunication_two(RC_Type* rc, uint8_t buff[])
{
  rc->mouse.y = buff[0] | (buff[1] << 8);
	rc->mouse.z = buff[2]| (buff[3] << 8);
	
	rc->mouse.press_left 	= buff[4];	// is pressed?
	rc->mouse.press_right = buff[5];
	
	rc->keyBoard.key_code = buff[6] | buff[7] << 8; //key borad code
}

void Get_Relative_Angle(connect_date* relative_angle, uint8_t buff[])
{
  uint8_t  angle[4];
  
  angle[0] = buff[0];
  angle[1] = buff[1];
  angle[2] = buff[2];
  angle[3] = buff[3];
 
  
  CharToFloat(&(relative_angle->relative_angle),angle,4);

  
}

///*接收裁判系统的信息
//参数：接收结构体，接收的Buff
//*/
//void GetJudgment1(Judgment_Measure* date,uint8_t buff[])
//{
//uint8_t speed[4],heat_lmit[4],heat[4];

//	speed[0]      =buff[0];
//	speed[1]      =buff[1];
//	heat_lmit[0]  =buff[2];
//	heat_lmit[1]  =buff[3];
//  heat[0]       =buff[4];
//	heat[1]       =buff[5];
//	
//	CharToUint16(&(date->Speed_Limit),speed,2);
//	CharToUint16(&(date->Heat_Limit),heat_lmit,2);
//	CharToUint16(&(date->Heat),heat,2);
//}

//void GetJudgment2(Judgment_Measure* date,uint8_t buff[])
//{
//uint8_t real_speed[4];

//  real_speed[0] = buff[0];
//	real_speed[1] = buff[1];
//	real_speed[2] = buff[2];
//	real_speed[3] = buff[3];
//	date->FireFeq = buff[4];
//	CharToFloat(&(date->Real_Speed),real_speed,4);

//}
#endif
