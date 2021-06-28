/**
  * @file      bsp_laserdistance.h
  * @author    yann
  * @version   0.1
  * @date      2021/05/20
  * @brief     激光测距驱动
  * @attention 
  *      1. 注意波特率
  */ 

#ifndef __BSP_LASERDISTANCE_H_
#define __BSP_LASERDISTANCE_H_

#include "stm32f1xx.h"
#include "string.h"


#define LASERDISTANCE_HUART huart3          // 和模块通信的串口
#define RECEIVE_FLAG laserdistance_recv         // 单片机确实获取到测距模块数据的标志，该标志在中断中更新

#ifndef BUFFER_SIZE
  #define BUFFER_SIZE 200
#endif

extern UART_HandleTypeDef LASERDISTANCE_HUART;
extern volatile uint8_t RECEIVE_FLAG; 
extern volatile int distance;               // 存储数据的变量
extern char distance_buffer[5];
extern uint8_t cmd_buffer[BUFFER_SIZE];



/**
  * @brief
  * @param
  * @retval
  */
void Get_Distance(void);





#endif

