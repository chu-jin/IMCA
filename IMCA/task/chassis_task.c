#include "chassis_task.h"
#include "RC_Task.h"
#include "IMU_Task.h"
#include "balance_ctrl.h"

uint8_t Chassis_Ctrl = 0;
chassis_status_t chassis_status;
static void Interact_check(void);
static void chassis_state_update(void);
static void remote_control_current(void);


/**********************************************************************************************************************
 * @Brief   ������
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
void chassis_task(void)
{
	  if(Chassis_Ctrl)
		{	
			chassis_state_update();
			BALANCE_Contrl(); 
			remote_control_current();
			Chassis_Ctrl = 0;
		}
		
}


/**********************************************************************************************************************
 * @Brief   �������״̬����
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
static void chassis_state_update(void)
{
	Interact_check();//������ͨ��״̬
	GetImuAngle(&receive_imusol,&IMU_Yaw,&IMU_Pitch);//��ȡ����������
	chassis_status.speed_move = -(CHASSIS_MOTOR[0].speed- CHASSIS_MOTOR[1].speed) /57.29578f*0.205f/2.0f/2.0f +IMU_Pitch.speed_gyro* 0.03f;
	//�н��ٶ�
	chassis_status.location_move +=chassis_status.speed_move*0.001f;//���ֺ�
}



/**********************************************************************************************************************
 * @Brief   ͨ��״̬���
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
static void Interact_check(void)
{
		/*���ͨ�ŵ���ԽǶ�*/
		Commucation_Data.lost_cnt++;
		if(Commucation_Data.lost_cnt>200)
		{
			Commucation_Data.lost_cnt = 200;
			Commucation_Data.lost_flag = 1;
		}
		else
		{
			Commucation_Data.lost_flag = 0;
		}
		/*���̵��*/
			/*��*/
			MFMOTOR_State_left.lost_cnt++;
			if(MFMOTOR_State_left.lost_cnt>200)
			{
				MFMOTOR_State_left.lost_cnt=200;
				MFMOTOR_State_left.lost_flag = 1;
			}
			else
			{
				MFMOTOR_State_left.lost_flag = 0;
			}
			/*��*/
			MFMOTOR_State_right.lost_cnt++;
			if(MFMOTOR_State_right.lost_cnt>200)
			{
				MFMOTOR_State_right.lost_cnt=200;
				MFMOTOR_State_right.lost_flag = 1;
			}
			else
			{
				MFMOTOR_State_right.lost_flag = 0;
			}
}


/**********************************************************************************************************************
 * @Brief   ң�����ϵ�
 * @Param   void
 * @Retval  void
 * @Other   ����
**********************************************************************************************************************/
static void remote_control_current(void)
{
	if(RC_UPPER_LEFT_SW_DOWN)
	{
		SetMotoriq(&hcan1,MF_MOTOR_ID_LEFT,0);
		SetMotoriq(&hcan1,MF_MOTOR_ID_RIGHT,0);
		HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_SET);
	}
	else
	{
		SetMotoriq(&hcan1,MF_MOTOR_ID_LEFT,Iq_Left);
		SetMotoriq(&hcan1,MF_MOTOR_ID_RIGHT,Iq_Right);
		HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);
	}
}