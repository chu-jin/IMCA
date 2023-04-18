#ifndef __RC_TASK
#define __RC_TASK

#include "stm32f4xx_hal.h"

#define RC_CONTROL_ENABLE   /*遥控器接收使能宏定义*/
#define RC_Frame_Lentgh		18

#define KEY_W 				((uint16_t)1 << 0)&remote_control.keyBoard.key_code
#define KEY_S 				((uint16_t)1 << 1)&remote_control.keyBoard.key_code 
#define KEY_A 				((uint16_t)1 << 2)&remote_control.keyBoard.key_code
#define KEY_D 				((uint16_t)1 << 3)&remote_control.keyBoard.key_code
#define KEY_SHIFT 		((uint16_t)1 << 4)&remote_control.keyBoard.key_code
#define KEY_CTRL 			((uint16_t)1 << 5)&remote_control.keyBoard.key_code
#define KEY_Q 				((uint16_t)1 << 6)&remote_control.keyBoard.key_code
#define KEY_E 				((uint16_t)1 << 7)&remote_control.keyBoard.key_code
#define KEY_R 				((uint16_t)1 << 8)&remote_control.keyBoard.key_code
#define KEY_F 				((uint16_t)1 << 9)&remote_control.keyBoard.key_code
#define KEY_G 				((uint16_t)1 << 10)&remote_control.keyBoard.key_code
#define KEY_Z 				((uint16_t)1 << 11)&remote_control.keyBoard.key_code
#define KEY_X 				((uint16_t)1 << 12)&remote_control.keyBoard.key_code
#define KEY_C 				((uint16_t)1 << 13)&remote_control.keyBoard.key_code
#define KEY_V 				((uint16_t)1 << 14)&remote_control.keyBoard.key_code
#define KEY_B 				((uint16_t)1 << 15)&remote_control.keyBoard.key_code



/*遥控器左上方和右上方的两个开关状态*/
#define  RC_UPPER_LEFT_SW_UP     (remote_control.switch_left == Switch_Up)
#define  RC_UPPER_LEFT_SW_MID    (remote_control.switch_left == Switch_Middle)     
#define  RC_UPPER_LEFT_SW_DOWN   (remote_control.switch_left == Switch_Down)
#define  RC_UPPER_RIGHT_SW_UP    (remote_control.switch_right == Switch_Up)
#define  RC_UPPER_RIGHT_SW_MID   (remote_control.switch_right == Switch_Middle)
#define  RC_UPPER_RIGHT_SW_DOWN  (remote_control.switch_right == Switch_Down)


//#define  RC_CH1_OFFSET   (remote_control.ch1)
//#define  RC_CH2_OFFSET   (remote_control.ch2)
//#define  RC_CH3_OFFSET   (remote_control.ch3)
//#define  RC_CH4_OFFSET   (remote_control.ch4)

typedef struct {
	int16_t ch1;	//each ch value from -364 -- +364
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
  int16_t ch5;
	
	uint8_t switch_left;	//3 value
	uint8_t switch_right;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
	
		uint8_t press_left;
		uint8_t press_right;
	}mouse;
	
	struct {
		uint16_t key_code;
/**********************************************************************************
   * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/

	}keyBoard;
	

}RC_Type;


enum{
	Switch_Up = 1,
	Switch_Middle = 3,
	Switch_Down = 2,
};

enum{
	Key_W,
	Key_S,
	//...
};

/*遥控或PC控制模式*/
enum{
    RC_CONTROL_MODE = 0,
    PC_CONTROL_MODE = 1,
};

extern RC_Type remote_control;
extern uint8_t UART_Buffer[100];


/*遥控接收数据解析*/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff);
/*配置串口没有DMA中断的接收函数*/
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
/*启动串口空闲中断*/
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
/*串口空闲中断，在USART2中断中调用*/
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
/*遥控器死区限制*/
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine);


#endif

