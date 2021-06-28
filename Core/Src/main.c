/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "SERVO/bsp_servo.h"
#include "MATRIXKEY/bsp_matrixkey.h"
#include "LASERDISTANCE/bsp_laserdistance.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265
//#define V_0 6.3837
#define V_0 6.442
#define g 9.7803
#define INVALID_VAL 999
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
enum InstructionSource{
  my_MatrixKey=0,
  my_USART1=1,
  my_USART2=2,
  my_USART3=3
};



volatile uint8_t rx1_len=0;
volatile uint8_t rx2_len=0;
volatile uint8_t rx3_len=0;
volatile uint8_t recv1_end_flag=0;
volatile uint8_t recv2_end_flag=0;
volatile uint8_t recv3_end_flag=0;
volatile uint8_t laserdistance_recv=0;
uint8_t rx1_buffer[BUFFER_SIZE];
uint8_t rx2_buffer[BUFFER_SIZE];
uint8_t rx3_buffer[BUFFER_SIZE];

char matrixkey_temp = 0;
MatrixKey_Buffer matrixkey_buffer;

volatile int pitch    = 0;
volatile int yaw      = 30;
volatile int distance = 0;    // 非负整数，单位cm
char pitch_buffer[5];
char yaw_buffer[5];
char distance_buffer[5];
volatile uint8_t mode  = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 处理串口数据
  * @param
  * @retval
  */
void Handle_UART_Data(UART_HandleTypeDef* huart);


/**
  * @brief 处理矩阵按键数据
  * @param
  * @retval
  */
void ProcessMatrixKeyData(void);


/**
  * @brief 分析指令
  *    数据格式：yaw*pitch*distance，999为非法值
  * @param
  * @retval
  */
void AnalyzeInstruction(char* instr, enum InstructionSource s);

void AnalyzeInstruction_MatrixKey(char* instr);
void AnalyzeInstruction_USART1(char* instr);
void AnalyzeInstruction_USART2(char* instr);
void AnalyzeInstruction_USART3(char instr[]);

int CalPitchAngle(void);


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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  //使能idle中断和打开串口DMA接收语句
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);          //使能idle中断
	HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE); //打开DMA接收，数据存入rx_buffer数组中。	
  
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);         
	HAL_UART_Receive_DMA(&huart2,rx2_buffer,BUFFER_SIZE); 
  
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);         
	HAL_UART_Receive_DMA(&huart3,rx3_buffer,BUFFER_SIZE); 
  
  
  // tim3用于产生pwm，tim4用于定时打印log
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 输出pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 输出pwm
  
  HAL_TIM_Base_Start_IT(&htim3);            // 启用中断
  HAL_TIM_Base_Start_IT(&htim4);            // 启用中断
  
//  int i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // 接收串口信息
    if(recv1_end_flag || recv2_end_flag || recv3_end_flag){
      Handle_UART_Data(&huart1);
      Handle_UART_Data(&huart2);
      Handle_UART_Data(&huart3);
    }
    ProcessMatrixKeyData();

   


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void Handle_UART_Data(UART_HandleTypeDef* huart)
{
  if(huart == &huart1)
  {
    if(recv1_end_flag ==1)
		{
			printf("rx_len=%d\n",rx1_len);                        //打印接收长度
			HAL_UART_Transmit(&huart1,rx1_buffer, rx1_len,200);   // 接收数据打印出来
      
      //分析指令
      AnalyzeInstruction((char*)rx1_buffer, my_USART1);

      
			for(uint8_t i=0;i<rx1_len;i++)
      {
				rx1_buffer[i]=0;                                    //清接收缓存
			}
			rx1_len=0;                                            //清除计数
			recv1_end_flag=0;                                     //清除接收结束标志位
		}
		HAL_UART_Receive_DMA(&huart1,rx1_buffer,BUFFER_SIZE);   //重新打开DMA接收
  } else if(huart == &huart2)
  {
    if(recv2_end_flag ==1)
		{
			printf("rx_len=%d\n",rx2_len);                        //打印接收长度
			HAL_UART_Transmit(&huart1,rx2_buffer, rx2_len,200);    // 接收数据打印出来
			for(uint8_t i=0;i<rx2_len;i++)
			{
					rx2_buffer[i]=0;                                  //清接收缓存
			}
			rx2_len=0;                                            //清除计数
			recv2_end_flag=0;                                     //清除接收结束标志位
		}
		HAL_UART_Receive_DMA(&huart2,rx2_buffer,BUFFER_SIZE);   //重新打开DMA接收
  } else if(huart == &huart3)
  {
    if(recv3_end_flag ==1)
		{
			printf("rx_len=%d\n",rx3_len);                        //打印接收长度
			HAL_UART_Transmit(&huart1,rx3_buffer, rx3_len,200);    // 接收数据打印出来
      
      // 分析指令
      AnalyzeInstruction((char*)rx3_buffer, my_USART3);
      // usart3的数据分析函数写在Get_Distance中。这样可以确保每次都使用的是新数据
      
			for(uint8_t i=0;i<rx3_len;i++)
			{
					rx3_buffer[i]=0;                                  //清接收缓存
			}
			rx3_len=0;                                            //清除计数
			recv3_end_flag=0;                                     //清除接收结束标志位
		}
		HAL_UART_Receive_DMA(&huart3,rx3_buffer,BUFFER_SIZE);   //重新打开DMA接收
  }
  
}

/** 注意：
 *       pitch舵机接PA6
 *       yaw舵机接PA7
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3){
    TIM3->CCR1 = Servo_Control(Get_Pitch_ServoAngle(pitch));
    TIM3->CCR2 = Servo_Control(Get_Yaw_ServoAngle(yaw));
  } 
  
  if(htim->Instance == TIM4){
    printf("[data:%s ", matrixkey_buffer.data);
    printf("size:%d]\n", matrixkey_buffer.size);
    printf("[yaw=%d pitch=%d d=%d]", yaw, pitch, distance);
  }
}



void ProcessMatrixKeyData(void){
  // 扫描按键
  matrixkey_temp = Key_ScanChar();
  switch(matrixkey_temp){
    case 0:
      break;
    case '#':
      AnalyzeInstruction(matrixkey_buffer.data, my_MatrixKey);
      matrixkey_buffer.size = 0;
      memset(matrixkey_buffer.data, 0, sizeof(matrixkey_buffer.data));//清空缓冲区
      
      //充电
      Relay2_Open;  // 断开
      Relay1_Close; // 闭合
      HAL_Delay(5000);
      // 发射
      Relay1_Open;  // 断开
      HAL_Delay(1000);
      Relay2_Close; // 闭合
      break;
    case 'D':  // 删除一个字符
      if(matrixkey_buffer.size != 0){
        matrixkey_buffer.size-- ;
        matrixkey_buffer.data[matrixkey_buffer.size] = 0;
      }
      break;
    case 'C':  // 清空缓冲区
      memset(matrixkey_buffer.data, 0, sizeof(matrixkey_buffer.data));
      matrixkey_buffer.size = 0;
      break;
    default:
      matrixkey_buffer.data[matrixkey_buffer.size] = matrixkey_temp;
      matrixkey_buffer.size++;
  }
}


void AnalyzeInstruction(char* instr, enum InstructionSource s)
{
  switch(s){
    case my_MatrixKey:
      AnalyzeInstruction_MatrixKey(instr);
      break;
    case my_USART1:
      AnalyzeInstruction_USART1(instr);
      break;
    case my_USART2:
      break;
    case my_USART3:
      AnalyzeInstruction_USART3(instr);
      break;
    default:
      return;
  }
}

void AnalyzeInstruction_MatrixKey(char* instr){
  //数据格式：yaw*pitch*distance，999为非法值
  char *delim = "*", *p;
  volatile int temp_pitch = 0;
  uint8_t index = 0;
  sprintf(yaw_buffer, "%s", strtok(instr, delim));
  while((p = strtok(NULL, delim)) && index<2){
    if(index == 0){
      sprintf(pitch_buffer, "%s", p);
    } else if (index == 1){
      sprintf(distance_buffer, "%s", p);
    }
    index ++;
  }
  
  yaw      = atoi((const char*)yaw_buffer);
  //pitch    = atoi((const char*)pitch_buffer);
  distance = atoi((const char*)distance_buffer);
  
  temp_pitch = atoi((const char*)pitch_buffer);
  if(temp_pitch == INVALID_VAL){
    pitch  = CalPitchAngle();
  } else {
    pitch = temp_pitch;
  }
}
void AnalyzeInstruction_USART1(char* instr){
  switch(*instr){
    case 'G':
      if(pitch < 90)
        pitch += 10;
      break;
    case 'K':
      if(pitch > -10)
        pitch -= 10;
      break;
    case 'H':
      if(yaw < 120)
        yaw += 10;
      break;
    case 'J':
      if(yaw > -60)
      yaw -= 10;
      break;
    case 'I':
      pitch = 0;
      yaw   = 30;
      break;
    case 'A':
      // 开始充电：先断开放电回路，再连接充电回路
      Relay2_Open;  // 断开
      Relay1_Close; // 闭合
      break;
    case 'B':
      // 开始放电：先断开充电回路，再连接放电回路
      Relay1_Open;  // 断开
      HAL_Delay(1000);
      Relay2_Close; // 闭合
      break;
    case 'C':
      // 同时断开充放电回路
      Relay1_Open;
      Relay2_Open;    
      break;
    case 'D':
      // 充电发射
      
      //充电
      Relay2_Open;  // 断开
      Relay1_Close; // 闭合
      HAL_Delay(5000);
      // 发射
      Relay1_Open;  // 断开
      HAL_Delay(1000);
      Relay2_Close; // 闭合
      break;
    case 'E':
      Get_Distance();
      break;
      
    default:
      AnalyzeInstruction_MatrixKey(instr);
  }
}
void AnalyzeInstruction_USART2(char* instr);
void AnalyzeInstruction_USART3(char instr[])
{
  // 处理接收到的数据
  if(instr[0]=='E')
  {
    return;
  } else if (instr[0]=='D'){
    
    double temp;  // 暂时存储d的中间值，单位m
    char *delim = "m";
    instr[0] = '0';
    instr[1] = '0';
    sprintf(distance_buffer, "%s", strtok((char*)instr, delim));   // 只提取切割后的第一位
    temp = atof((const char*)distance_buffer);
    distance = (int)(temp * 1000 + 0.5);   // 四舍五入
  }
  return;
}



int CalPitchAngle(void)
{
//  double temp_d = 0.001 * distance;
//  double angle = 0;
//  int temp_pitch;
//  double error_distance = 0;
//  //angle = 0.5 * asin(temp_d * g / ( V_0 * V_0)) * 180.0 / PI ;
//  int i = 0;
//  angle = 0.5 * asin(temp_d * g / ( V_0 * V_0)) * 180.0 / PI ;

//  temp_pitch = (int) (angle + 0.5);
//  return temp_pitch;
  
  
//  double temp_d = 0.001 * distance;
//  double angle = 0;
//  int temp_pitch;
//  double error_distance = 0;
//  //angle = 0.5 * asin(temp_d * g / ( V_0 * V_0)) * 180.0 / PI ;
//  int i = 0;
//  
//  for (;i<10;i++){
//    //angle = 0.5 * asin(temp_d * g / ( V_0 * V_0)) * 180.0 / PI ;
//    rad = 0.5 * asin(temp_d * g / ( V_0 * V_0)) * 180.0 / PI ;
//    
//  }
//  temp_pitch = (int) (angle + 0.5);
//  return temp_pitch;
//  

  
  double temp_d = 0.001 * distance;
  double angle = 0;
  int temp_pitch;
  angle = 10 * temp_d - 8 ;
  temp_pitch = (int) (angle + 0.5);
  return temp_pitch;

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
