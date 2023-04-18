/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RC_Task.h"
#include "DataScope_DP.h"
#include "balance_ctrl.h"
#include "balance_cal.h"
#include "imu_data_decode.h"
#include "IMU_Task.h"
#include "CanBus_Task.h"
#include "oled_iic.h"
#include "chassis_task.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEFT  0
#define RIGHT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer6[1];

uint8_t OLED_Flag;//屏幕刷新任务时间标志位


#if 1 //R标
/****************************************R标志************************************/
 unsigned char R[] = 
{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF7,0xE7,0xC7,
0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x0F,0x1F,0x1F,
0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x0C,0x1C,
0x3C,0x7C,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xF8,0x78,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x3F,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0xC0,0xC0,
0xF0,0xF8,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x1F,0x03,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFD,0xF9,0xF1,0xC1,0x81,0x01,0x01,0x01,0x01,0x01,0x01,0x03,0x07,0x0F,0x1F,0x7F,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xEF,0xE1,0xE0,0xE0,
0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xF0,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xFC,
0xFC,0xF8,0xF8,0xF0,0xF0,0xE0,0xE0,0xC0,0xC0,0x80,0x80,0x00,0x08,0x38,0x78,0xF8,
0xF8,0xF9,0xFB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,
0x7F,0x1F,0x1F,0x1F,0x7F,0x7F,0x7F,0x3F,0x3F,0x9F,0xBF,0x9F,0x1F,0x1F,0x1F,0xFF,
0x1F,0x1F,0x1F,0x9F,0x9F,0x1F,0x1F,0x1F,0x1F,0x9F,0x9F,0x9F,0x1F,0x1F,0x1F,0xFF,
0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x1F,0x1F,0x1F,0x3F,0x7F,0x7F,0x7F,0x7E,0x7E,0x7C,
0x3D,0x3B,0x3F,0x1F,0x1F,0x1F,0xFF,0x9F,0x9F,0x9F,0x1F,0x1F,0x1F,0x9F,0x9F,0x9F,
0xBF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x07,0x61,
0x03,0x00,0x00,0x18,0x01,0x0F,0x00,0x00,0xE0,0xFF,0xFF,0x03,0x00,0x00,0xF8,0xC1,
0xC0,0xC0,0xC0,0xD9,0xD9,0x00,0x00,0x00,0x00,0xD9,0xD9,0xD9,0xC0,0xC0,0xC0,0xFF,
0x7C,0x3C,0x1C,0x04,0x00,0xC0,0xE0,0xF8,0xF0,0xE0,0x80,0x04,0x0C,0x1C,0x3C,0x7C,
0xFE,0x00,0x00,0x1F,0x80,0x00,0x00,0x3C,0x00,0x00,0x0E,0xE0,0x00,0x00,0x0E,0xE0,
0x00,0x03,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xEE,0xF0,
0xF8,0xF8,0xFE,0xCE,0xCB,0xEA,0xEA,0xCA,0xD3,0xE7,0xC3,0xDA,0xDA,0xE2,0xDE,0xC3,
0xD3,0xD3,0xC3,0xE3,0xE7,0xC2,0xDA,0xDA,0xE2,0xDF,0xE3,0xE3,0xCF,0xE7,0xF3,0xC2,
0xFE,0xDE,0xE6,0xD2,0xD3,0xC7,0xDF,0xFF,0xD3,0xD3,0xDB,0xDB,0xEA,0xFA,0xC2,0xF2,
0xFA,0xDB,0xD3,0xD2,0xD3,0xD2,0xFE,0xCA,0xEB,0xEA,0xCB,0xD3,0xFC,0xF8,0xF0,0xF0,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* (128 X 64 ) */
#endif
static void Oled_flush(void)
{
	if(OLED_Flag)
	{
		/*显示角度*/
		OLED_ShowFloatNumber(75,2,IMU_Pitch.angle,1);	
		/*板间 现在判断的是相对角度的发送*/
		if(Commucation_Data.lost_flag)
		{
			oled_show_string(75,4,"NO",1);
		}
		else
		{
			oled_show_string(75,4,"OK",1);
		}
		/*电机*/
		/*右*/
		if(MFMOTOR_State_right.lost_flag)
		{
			oled_show_string(75+(0*15),6,"NO",1);
		}
		else
		{
			oled_show_string(75+(0*15),6,"OK",1);
		}
		/*左*/
		if(MFMOTOR_State_left.lost_flag)
		{
			oled_show_string(75+(1*15),6,"NO",1);
		}
		else
		{
			oled_show_string(75+(1*15),6,"OK",1);
		}
		OLED_Flag=0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
 	MX_I2C3_Init();
	oled_init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);  //使能TIM3中断，溢出时间为1ms
	HAL_UART_Receive_IT_IDLE(&huart2,UART_Buffer,100);//启动串口空闲中断，用于接收遥控器发来的数据
	imu_data_decode_init(); //映射解析函数（数值接收解析初始化）
	HAL_UART_Receive_DMA(&huart6,aRxBuffer6,sizeof(aRxBuffer6));//陀螺仪USART接收
	CANFilterInit();
	PARAM_Pid_init();//PID参数初始化
	SetMotorPid_RAM(&hcan1,MF_MOTOR_ID_LEFT,0,0,0,0,50,50);//ID1
	SetMotorPid_RAM(&hcan1,MF_MOTOR_ID_RIGHT,0,0,0,0,50,50);//ID2
	
	HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);
	oled_drawBMP(0,0,128,8,R);//绘制R标
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_SET);
	oled_cls();
	oled_show_string(20,3,"Init finish",2);
	HAL_Delay(500);
	oled_cls();
	oled_show_string(0,0,	"INTERACT",1);
	oled_show_string(0,2,	"Angle",1);
	oled_show_string(0,4,	"Board",1);
	oled_show_string(0,6,	"Motor",1);
	oled_show_string(75,0,"STATE",1);
	oled_show_string(75,2,"?",1);
	oled_show_string(75,4,"?",1);
	oled_show_string(75,6,"?",1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		chassis_task();
		if(Data_Scope_DP_Send)//波形读取
    {
			Oled_flush();
		  Data_Scope_DP_Send = 0;
		  DataScope();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t  ms = 0;
	static uint16_t ms50 = 0;
	if(htim->Instance == TIM3)
	{
		if(ms++>=1)
		{
			ms = 0;
			/*读取电机状态*/
//			ReadMotorState(&hcan1,MF_MOTOR_ID_LEFT); //发送读取电机状态得命令
//			ReadMotorState(&hcan1,MF_MOTOR_ID_RIGHT);//发送读取电机状态得命令
//			SendMotorCommand(&hcan1,MF_MOTOR_ID_LEFT,0x30);
//			SendMotorCommand(&hcan1,MF_MOTOR_ID_RIGHT,0x30);
			/*检测堵转*/
			if(Gimbal_Motor[LEFT].speed_rpm<=10||Gimbal_Motor[LEFT].speed_rpm>=-10)  count_left++;else closs_left =0;
			if(count_left>=500)closs_left=1;
			if(Gimbal_Motor[RIGHT].speed_rpm<=10||Gimbal_Motor[RIGHT].speed_rpm>=-10)count_right++;else closs_right =0;
			if(count_right>=500)closs_right=1;
			/*标志位置1*/
			Chassis_Ctrl = 1;	
		}
		if(ms50++>=50)
		{
			OLED_Flag = 1;
			ms50 = 0;
			Data_Scope_DP_Send=1;			
		}
	}
}

/* 陀螺仪串口数据处理 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)	// 判断是由哪个串口触发的中断
	{    
		packet_decode(aRxBuffer6[0]);//解析数据
		HAL_UART_Receive_DMA(&huart6,aRxBuffer6,sizeof(aRxBuffer6));		// 重新使能串口6接收中断
  }


}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)         //丢失后重新开启
{
	if(huart == &huart6)
	{
		__HAL_UNLOCK(huart);
		packet_decode(aRxBuffer6[0]);//解析数据
		HAL_UART_Receive_DMA(&huart6,aRxBuffer6,sizeof(aRxBuffer6));		// 重新使能串口6接收中断
	}					  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
