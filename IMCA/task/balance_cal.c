#include "balance_cal.h"
#include "balance_ctrl.h"
#include "IMU_Task.h"
#include "CanBus_Task.h"
#include "mytype.h"
#include "chassis_task.h"
#define LEFT  0
#define RIGHT 1
/* PID参数结构体 */
pid_t BALANCE_ANGLE[1];//速度环
pid_t BALANCE_GYRO[1];//直立环
pid_t OUT_INTERAL[1];//输出积分环
pid_t BALANCE_SPEED[1];//速度环
pid_t BALANCE_LOCATION[1];//速度环
pid_t derivative[1];//测试微分项
float K[2]={-19.8519   -3.1533};//Q=1,R=0.2， K= -19.8519   -3.1533；

/**********************************************************************************************************************
 * @Brief   PID参数初始化
 * @Param   void
 * @Retval  void
 * @Other  
**********************************************************************************************************************/
void PARAM_Pid_init(void)
{
	PID_struct_init(&BALANCE_GYRO[0],  		POSITION_PID, 10000, 0,  0,  2.1533,  0.0f,   0); //速度环//1500, 0,  3.1533, 0.0, 0
	PID_struct_init(&BALANCE_ANGLE[0], 		POSITION_PID, 10000, 0,  0,  19.8519, 0.0f,     0); //直立环//1500, 0, 19.8519, 0, 0
	PID_struct_init(&BALANCE_SPEED[0],    POSITION_PID, 250, 	 100, 0, 30,      0.010f, 0); //直立环//1500, 0, 19.8519, 0, 0
	PID_struct_init(&BALANCE_LOCATION[0], POSITION_PID, 120,   0,  0,  60,      0.0f,  10.0f); //直立环//1500, 0, 19.8519, 0, 0
	PID_struct_init(&OUT_INTERAL[0],   		POSITION_PID, 2000,  30, 0,  1,       0.0f,  0); //直立环//1500, 0, 19.8519, 0, 0
}

/**********************************************************************************************************************
 * @Brief   对角度进行软件限位，限制最大最小角度（1.0）
 * @Param   输入值，最大值，最小值
 * @Retval  返回角度
 * @Other  
**********************************************************************************************************************/
float ANGLE_Limit(float input, int32_t max_angle , int32_t min_angle )
{

	static float output;

	if ((input<=max_angle)&&(input>=min_angle))
	{
		output=input;
	}
	return output;
	
}

/**********************************************************************************************************************
 * @Brief   MF电机运动控制 角度环 
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
void chassis_balance_static(void)
{
	if(RC_UPPER_RIGHT_SW_MID||RC_UPPER_RIGHT_SW_UP)//遥控器控制断电（左侧按键）
	{		
		/*LQR */
//		pid_calc(&BALANCE_ANGLE[0] ,  IMU_Pitch.angle , -0.0f); //
//	//derivative[1].err[NOW]= 10*(IMU_Pitch.angle - IMU_Pitch.last_angle);//IMU_Pitch.angle_diff;
//		pid_calc(&BALANCE_GYRO[0]  , 	IMU_Pitch.speed_gyro , 0);
//		//调节速度使稳态误差为零
//		pid_calc(&BALANCE_SPEED[0] ,  chassis_status.speed_move , 0); 	
//		pid_calc(&BALANCE_LOCATION[0] ,  chassis_status.location_move,0); 		
//		pid_calc(&OUT_INTERAL[0], 0 , (-BALANCE_ANGLE[0].pos_out-BALANCE_GYRO[0].pos_out-BALANCE_SPEED[0].pos_out-BALANCE_LOCATION[0].pos_out)*iq_const);	
//		
		/*LQR__u=-kz*/
		
	OUT_INTERAL[0].pos_out=iq_const*(K[0]*(0-IMU_Pitch.angle)+K[1]*( 0-IMU_Pitch.speed_gyro));
		//把Xd作为新的平衡点输入
		//OUT_INTERAL[0].pos_out=iq_const*(-2+(K[0]*(0-IMU_Pitch.angle)+K[1]*( 0-IMU_Pitch.speed_gyro)));
		
		/*电流赋值*/	
		Iq_Left = -OUT_INTERAL[0].pos_out;
		Iq_Right = OUT_INTERAL[0].pos_out;
//		Iq_Left  =  (MOTOR_STD_ECO[LEFT].pos_out+MOTOR_SPD_ECO[LEFT].pos_out)*iq_const ;		
//		Iq_Left  =  ANGLE_Limit(Iq_Left,2000,-2000);
//		Iq_Right = 	-(MOTOR_STD_ECO[LEFT].pos_out+MOTOR_SPD_ECO[LEFT].pos_out)*iq_const;			
//		Iq_Right  =  ANGLE_Limit(Iq_Right,2000,-2000);
	}	
	else
	{
		Iq_Left  = 0;
		Iq_Right = 0;
	}
}

/**********************************************************************************************************************
 * @Brief   MF电机运动控制 角度闭环+角度闭环 
 * @Param   void
 * @Retval  void
 * @Other   
**********************************************************************************************************************/
void BALANCE_Speed_cal(float speed)
{
	
}

/**********************************************************************************************************************
 * @Brief   GM电机运动控制
 * @Param   void
 * @Retval  void
 * @Other   双环串级控制
**********************************************************************************************************************/
void BALANCE_Stand_cal2(float Pitch_Angle)
{
	
}
