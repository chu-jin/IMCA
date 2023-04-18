#ifndef __CANBUS_TASK
#define __CANBUS_TASK


#include "mytype.h"
#include "can.h"
#include "pid.h"
#include "RC_Task.h"
#define FILTER_BUF_LEN		5
#define JUDMENT_ONOFF   1 //�Ƿ���ң�������ݽ���
#define BOARD_COMUNICATE_ONOFF   1 //�Ƿ������ͨ��
extern u8 temp;

/*���յ����µ���Ĳ����ṹ��*/
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
	u8			lost_cnt;//���߼���
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
	
	CAN2_CAP_ID = 0x108, //��������
	CAN2_GET_RC_ID = 0x601,      
	CAN2_GET_RC2_ID = 0x602,
	CAN2_GIMBAL_MEASURE_ID = 0x603,
}CAN_Message_ID;


/*���յ�����̨����Ĳ����ṹ��*/
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

/*���յ������ذ���ԽǶ�*/
typedef struct{
	float receive_yaw_relative_angle;
	float yaw_speed;
	u8			lost_cnt;//���߼���
	u8			lost_flag;
	u8			online_flag;	
}commucation_t;

/*��Ҫ����ĽǶȲ���ϵͳ������*/
typedef struct{
	float relative_angle;

}connect_date;

extern commucation_t Commucation_Data;
extern MFMOTOR_State_t CHASSIS_MOTOR[2];
extern MFMOTOR_State_t MFMOTOR_State_left;//MF���״̬�洢�ṹ��
extern MFMOTOR_State_t MFMOTOR_State_right;//MF���״̬�洢�ṹ��
extern moto_measure_t  Chassis_Motor[2];  //���̵�������ṹ��
extern moto_measure_t  Gimbal_Motor[2];  //��̨��������ṹ��
extern moto_measure_t  Chassis_Motor[2];  //ƽ�ⲽ�����̽ṹ��
extern moto_measure_t  Fluck_Motor[1];  //2006�����ֲ����ṹ��
extern uint16_t count_left;
extern uint16_t count_right;
extern uint8_t closs_left;//��ת��־λ
extern uint8_t closs_right;//��ת��־λ
extern float Jscope_IMU_X;
extern float Jscope_IMU_Y; 
extern float Jscope_IMU_Z; 
extern float Jscope_Gimbal_w;
extern float Jscope_Gimbal_x; 


/*CAN1�����������ú�CAN�Ŀ���*/
void CANFilterInit(void);
/*��õ���Ļ�е�Ƕ�*/
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*����3508���ͨ��CAN����������Ϣ*/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN��ǰ4������Ŀ���*/
void SetChassisMotorCurrent(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN�ĺ�4������Ŀ���*/
void SetGimbalMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*Ħ���ֺͲ����֣�CAN2*/
void SetShootMotorValue(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3);
/*�����ֵ��*/

/*ң�����ݴ���*/
void sen_can_comu_1(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8);
void sen_can_comu_2(CAN_HandleTypeDef *hcan,u8 iq1, u8 iq2, u8 iq3, u8 iq4, u8 iq5, u8 iq6, u8 iq7, u8 iq8);
/*mf���*/
void SetMotoriq(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int16_t iq);
void SetMotorPid_RAM(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,uint8_t anglePidKp,uint8_t anglePidKi,uint8_t SpeedPidKp,uint8_t SpeedPidKi,uint8_t iqPidKp,uint8_t iqPidKi);//д��PID��RAM
void SetMotorSpeed(CAN_HandleTypeDef *hcan,int32_t speedcontrol);
void SetMotorCurrent(CAN_HandleTypeDef *hcan,int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4);//���ö��MF�������
void ReadMotorState(CAN_HandleTypeDef *hcan,uint32_t Motor_ID);//��ȡMF���״̬
void GetMotorstate(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[]);//����MF���״̬
void SendMotorCommand(CAN_HandleTypeDef *hcan,uint32_t Motor_ID,int8_t command);//����MF�������
void GetMotorPID(MFMOTOR_State_t*data, uint8_t CAN_Data_rx[]);
/*��̨pitch����*/
void SetGimbalPitchMotorValue(CAN_HandleTypeDef *hcan,s16 iq1);
/*��̨yaw����*/
void SetGimbalYawMotorValue(CAN_HandleTypeDef *hcan,s16 iq1);

void GetGimbalMeasure(commucation_t* data, uint8_t buff[]);
#if JUDMENT_ONOFF//���Խ׶λ��ò���
///*���ղ���ϵͳ����Ϣ*/
//void GetJudgment1(Judgment_Measure* date,uint8_t buff[]);
//void GetJudgment2(Judgment_Measure* date,uint8_t buff[]);
/*����ң��������Ϣ*/
void GetCanCommunication(RC_Type* rc, uint8_t buff[]);
void GetCanCommunication_two(RC_Type* rc, uint8_t buff[]);
void Get_Relative_Angle(connect_date* relative_angle, uint8_t buff[]);
void Send_RG(CAN_HandleTypeDef *hcan,RC_Type*date);
#endif

#endif

