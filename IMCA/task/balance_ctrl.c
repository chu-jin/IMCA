#include "balance_ctrl.h"
#include "balance_cal.h"
#include "RC_Task.h"
#include "IMU_Task.h"
#include "CanBus_Task.h"



int16_t Iq_Left      = 0;
int16_t Iq_Right     = 0;




/**********************************************************************************************************************
 * @Brief   ƽ����Ƹ�ֵ
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
CCMRAM void BALANCE_Contrl(void)
{
	chassis_balance_static();//����PID����
}


void remote_control_current(void)
{
	
}
//static void chassis_state_update(void)
//{
////	//���ӵ��ٶ�=���ת��* (����ֱ��*2��/2)/������ٱ�/60  ���ӵ��ٶȵ�λ��m/s ���ת�ٵ�λ��rpm  ����ֱ����λ��m
////	chassis_state.x_speed = -(MFMOTOR_State_left.speed - MFMOTOR_State_right.speed) /57.29578f*Diameter_weel/2.0f/2.0f + IMU_Row.speed_gyro * 0.03f;
////	chassis_state.x_potion  += chassis_state.x_speed*0.001f; 
//}
