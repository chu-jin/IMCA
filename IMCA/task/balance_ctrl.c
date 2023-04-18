#include "balance_ctrl.h"
#include "balance_cal.h"
#include "RC_Task.h"
#include "IMU_Task.h"
#include "CanBus_Task.h"



int16_t Iq_Left      = 0;
int16_t Iq_Right     = 0;




/**********************************************************************************************************************
 * @Brief   平衡控制赋值
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
CCMRAM void BALANCE_Contrl(void)
{
	chassis_balance_static();//测试PID方向
}


void remote_control_current(void)
{
	
}
//static void chassis_state_update(void)
//{
////	//轮子的速度=电机转速* (轮子直径*2Π/2)/电机减速比/60  轮子的速度单位是m/s 电机转速单位是rpm  轮子直径单位是m
////	chassis_state.x_speed = -(MFMOTOR_State_left.speed - MFMOTOR_State_right.speed) /57.29578f*Diameter_weel/2.0f/2.0f + IMU_Row.speed_gyro * 0.03f;
////	chassis_state.x_potion  += chassis_state.x_speed*0.001f; 
//}
