#include "DataScope_DP.h"
#include "CanBus_Task.h"
#include "balance_cal.h"
#include "IMU_Task.h"
#include "balance_ctrl.h"
#include "chassis_task.h"
unsigned char DataScope_OutPut_Buffer[42] = {0};	   //���ڷ��ͻ�����

/*--------------------------------------------------------------------------------------------------
    ����λ��ͨ�ŵ�����֡���ȹ̶�Ϊ4N*1��֡�����ֽ�֮�䴫����ʱ���ɳ���1ms��������Ϊ��ǰ֡������
    UIˢ��û��������ʾ��ʱ
----------------------------------------------------------------------------------------------------*/

//����˵�����������ȸ�������ת��4�ֽ����ݲ�����ָ����ַ
//����˵�����û�����ֱ�Ӳ����˺���
//target:Ŀ�굥��������
//buf:��д������
//beg:ָ��������ڼ���Ԫ�ؿ�ʼд��
//�����޷���ֵ

uint8_t Data_Scope_DP_Send;
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//����˵������������ͨ���ĵ����ȸ�������д�뷢�ͻ�����
//Data:ͨ������
//Channel:ѡ��ͨ��(1-10)
//�����޷��� 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //ͨ����������10���ߵ���0��ֱ����������
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//����˵��������DataScopeV1.0����ȷʶ���֡��ʽ
//Channel_Number,��Ҫ���͵�ͨ������
//���ط��ͻ��������ݵĸ���
//����0��ʾ֡��ʽ����ʧ��
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //ͨ����������10���ߵ���0��ֱ����������
  else
  {	
	 DataScope_OutPut_Buffer[0] = '$';  //֡ͷ
		
	 switch(Channel_Number)   
   { 
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
   }	 
  }
	return 0;
}

//-------------------����������-------------------------------------
void DataScope(void)
{
  unsigned char i;          //��������
  unsigned char Send_Count; //������Ҫ���͵����ݸ���
  
        DataScope_Get_Channel_Data(CHASSIS_MOTOR[0].speed /10.0f, 1 );//ң����
        DataScope_Get_Channel_Data( chassis_status.speed_move /10.0f, 2 );
        DataScope_Get_Channel_Data(BALANCE_SPEED[0].pos_out /10.0f, 3 ); //������
        DataScope_Get_Channel_Data(BALANCE_ANGLE[0].pos_out/10.0f, 4 );
        DataScope_Get_Channel_Data(IMU_Pitch.speed_gyro , 5 );//����ϵͳ    BALANCE_GYRO[0].pos_out/1.0f
        DataScope_Get_Channel_Data(IMU_Pitch.total_angle/1.0f , 6);
        DataScope_Get_Channel_Data(BALANCE_ANGLE[0].pos_out/1.0f, 7 );
        DataScope_Get_Channel_Data(Iq_Left/1.0f, 9 ); //Ħ����
		//DataScope_Get_Channel_Data(MFMOTOR_State_left.iq /1.0f, 9 );  //YAW���
		DataScope_Get_Channel_Data(300*(IMU_Pitch.angle - IMU_Pitch.last_angle)/1.0f, 8 ); //�۲�΢�����Ƿ������ֵһ��
        DataScope_Get_Channel_Data(IMU_Pitch.angle/1.0f, 10);
        Send_Count = DataScope_Data_Generate(10);
        for( i = 0 ; i < Send_Count; i++) 
        {
        while((USART1->SR&0X40)==0);  
        USART1->DR = DataScope_OutPut_Buffer[i]; 
        }

		
}

//------------------------------------------------------------------------------

