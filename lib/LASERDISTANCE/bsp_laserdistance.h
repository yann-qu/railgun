/**
  * @file      bsp_laserdistance.h
  * @author    yann
  * @version   0.1
  * @date      2021/05/20
  * @brief     ����������
  * @attention 
  *      1. ע�Ⲩ����
  */ 

#ifndef __BSP_LASERDISTANCE_H_
#define __BSP_LASERDISTANCE_H_

#include "stm32f1xx.h"
#include "string.h"


#define LASERDISTANCE_HUART huart3          // ��ģ��ͨ�ŵĴ���
#define RECEIVE_FLAG laserdistance_recv         // ��Ƭ��ȷʵ��ȡ�����ģ�����ݵı�־���ñ�־���ж��и���

#ifndef BUFFER_SIZE
  #define BUFFER_SIZE 200
#endif

extern UART_HandleTypeDef LASERDISTANCE_HUART;
extern volatile uint8_t RECEIVE_FLAG; 
extern volatile int distance;               // �洢���ݵı���
extern char distance_buffer[5];
extern uint8_t cmd_buffer[BUFFER_SIZE];



/**
  * @brief
  * @param
  * @retval
  */
void Get_Distance(void);





#endif

