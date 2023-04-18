#ifndef __CANBUS_TASK
#define __CANBUS_TASK


#include "mytype.h"
#include "can.h"
#include "pid.h"
#include "RC_Task.h"
#define FILTER_BUF_LEN		5
#define JUDMENT_ONOFF   1 //是否开启遥控器数据解析
#define BOARD_COMUNICATE_ONOFF   1 //是否开启板间通信
extern u8 temp;

/*接收到的新电机的参数结构体*/
typedef struct{
	int8_t	 	temperature;
	int16_t		iq;
	int16_t  	speed;
	uint16_t	encoder;
	uint8_t   angleKp;
	uint8_t   angleKi;
	uint8_t   SpeedKp;
	uint8_t   SpeedKi;
	uint8_t   iqKp;
	uint8_t   iqKi;
	u8			lost_cnt;//掉线计数
	u8			lost_flag;
	u8			online_flag;	
}MFMOTOR_State_t;

typedef enum
{
	MF_MOTOR_ID_LEFT    = 0x142,//MF ID2
	MF_MOTOR_ID_RIGHT   = 0x143,//MF ID3
	RM_MOTOR_ID_LEFT    = 0x205,//RM ID1
	RM_MOTOR_ID_RIGHT   = 0x206,//RM ID2
	CAN_JUDGMENT_ID1    = 0X604,
	CAN_JUDGMENT_ID2    = 0X605,
	
	CAN2_CAP_ID = 0x108, //超级电容
	CAN2_GET_RC_ID = 0x601,      
	CAN2_GET_RC2_ID = 0x602,
	CAN2_GIMBAL_MEASURE_ID = 0x603,
}CAN_Message_ID;


/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
	float  	    real_current;
	int16_t  	given_current;
	uint8_t  	hall;
	uint16_t 	angle;		//abs angle range:[0,8191]
	uint16_t 	last_angle;	  //abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

/*接收到的主控板相对角度*/
typedef struct{
	float receive_yaw_relative_angle;
	float yaw_speed;
	u8			lost_cnt;//掉线计数
	u8			lost_flag;
	u8			online_flag;	
}commucation_t;

/*需要传输的角度裁判系统等数据*/
typedef struct{
	float relative_angle;

}connect_date;

extern commucation_t Commucation_Data;
extern MFMOTOR_State_t CHASSIS_MOTOR[2];
extern MFMOTOR_State_t MFMOTOR_State_left;//MF电机状态存储结构体
extern MFMOTOR_State_t MFMOTOR_State_right;//MF电机状态存储结构体
extern moto_measure_t  Chassis_Motor[2];  //底盘电机参数结构体
extern moto_measure_t  Gimbal_Motor[2];  //云台电机参数结构体
extern moto_measure_t  Chassis_Motor[2];  //平衡步兵底盘结构体
extern moto_measure_t  Fluck_Motor[1];  //2006拨弹轮参数结构体
extern uint16_t count_left;
extern uint16_t count_right;
extern uint8_t closs_left;//堵转标志位
extern uint8_t closs_right;//堵转标志位
extern float Jscope_IMU_X;
extern float Jscope_IMU_Y; 
extern float Jscope_IMU_Z; 
extern float Jscope_Gimbal_w;
extern float Jscope_Gimbal_x; 


/*CAN1过滤器的配置和CAN的开启*/
void CANFilterInit(void);
/*获得电机的机械角度*/
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*接收3508电机通过CAN发过来的信息*/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*发送电机的信息到CAN总线上，此函数用于一路CAN的前4个电机的控制*/
void SetChassisMotorCurrent(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*发送电机的信息到CAN总线上，此函数用于一路CAN的后4个电机的控制*/
void SetGimbalMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*摩擦轮和拨弹轮，CAN2*/
void SetShootMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3);
/*拨弹轮电机*/

/*遥控数据传输*/
void sen_can_comu_1(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8);
void sen_can_comu_2(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8);
/*mf电机*/
void SetMotoriq(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int16_t iq);
void SetMotorPid_RAM(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,uint8_t anglePidKp,uint8_t anglePidKi,uint8_t SpeedPidKp,uint8_t SpeedPidKi,uint8_t iqPidKp,uint8_t iqPidKi);//写入PID到RAM
void SetMotorSpeed(CAN_HandleTypeDef *hcan,int32_t speedcontrol);
void SetMotorCurrent(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);//设置多个MF电机电流
void ReadMotorState(CAN_HandleTypeDef *hcan,uint32_t Motor_ID);//读取MF电机状态
void GetMotorstate(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[]);//解析MF电机状态
void SendMotorCommand(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int8_t command);//发送MF电机命令
void GetMotorPID(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[]);
/*云台pitch轴电机*/
void SetGimbalPitchMotorValue(CAN_HandleTypeDef *hcan,s16 iq1);
/*云台yaw轴电机*/
void SetGimbalYawMotorValue(CAN_HandleTypeDef *hcan,s16 iq1);

void GetGimbalMeasure(commucation_t* data, uint8_t buff[]);
#if JUDMENT_ONOFF//测试阶段还用不上
///*接收裁判系统的信息*/
//void GetJudgment1(Judgment_Measure* date,uint8_t buff[]);
//void GetJudgment2(Judgment_Measure* date,uint8_t buff[]);
/*接收遥控器的信息*/
void GetCanCommunication(RC_Type* rc, uint8_t buff[]);
void GetCanCommunication_two(RC_Type* rc, uint8_t buff[]);
void Get_Relative_Angle(connect_date* relative_angle, uint8_t buff[]);
void Send_RG(CAN_HandleTypeDef *hcan,RC_Type*date);
#endif

#endif

