/**
  * @file      bsp_matrixkey.h
  * @author    yann
  * @version   0.1
  * @date      2021/05/13
  * @brief     �����������
  * @attention 
  *      1. ע���޸����Ŷ���
  *      2. �������������롣
  *      3. ���������������������Ҳ��������Ĭ������ߵ�ƽ
  */ 

#ifndef __BSP_MATRIXKEY_H_
#define __BSP_MATRIXKEY_H_

#include "stm32f1xx.h"


// �����ţ��������
#define KEY_Col0_GPIO GPIOB
#define KEY_Col1_GPIO GPIOB
#define KEY_Col2_GPIO GPIOB
#define KEY_Col3_GPIO GPIOB
#define KEY_Col0_Pin GPIO_PIN_15
#define KEY_Col1_Pin GPIO_PIN_14
#define KEY_Col2_Pin GPIO_PIN_13
#define KEY_Col3_Pin GPIO_PIN_12

// �����ţ���������
#define KEY_Row0_GPIO GPIOB
#define KEY_Row1_GPIO GPIOB
#define KEY_Row2_GPIO GPIOB
#define KEY_Row3_GPIO GPIOB
#define KEY_Row0_Pin GPIO_PIN_7
#define KEY_Row1_Pin GPIO_PIN_6
#define KEY_Row2_Pin GPIO_PIN_5
#define KEY_Row3_Pin GPIO_PIN_4



#define KEY_CLO0_OUT_LOW  HAL_GPIO_WritePin(KEY_Col0_GPIO,KEY_Col0_Pin,GPIO_PIN_RESET) 
#define KEY_CLO1_OUT_LOW  HAL_GPIO_WritePin(KEY_Col1_GPIO,KEY_Col1_Pin,GPIO_PIN_RESET)
#define KEY_CLO2_OUT_LOW  HAL_GPIO_WritePin(KEY_Col2_GPIO,KEY_Col2_Pin,GPIO_PIN_RESET)
#define KEY_CLO3_OUT_LOW  HAL_GPIO_WritePin(KEY_Col3_GPIO,KEY_Col3_Pin,GPIO_PIN_RESET)

#define KEY_CLO0_OUT_HIGH  HAL_GPIO_WritePin(KEY_Col0_GPIO,KEY_Col0_Pin,GPIO_PIN_SET) 
#define KEY_CLO1_OUT_HIGH  HAL_GPIO_WritePin(KEY_Col1_GPIO,KEY_Col1_Pin,GPIO_PIN_SET)
#define KEY_CLO2_OUT_HIGH  HAL_GPIO_WritePin(KEY_Col2_GPIO,KEY_Col2_Pin,GPIO_PIN_SET)
#define KEY_CLO3_OUT_HIGH  HAL_GPIO_WritePin(KEY_Col3_GPIO,KEY_Col3_Pin,GPIO_PIN_SET)

#define MATRIXKEY_BUFFER 200

typedef struct {
  char data[MATRIXKEY_BUFFER];
  uint8_t size;
} MatrixKey_Buffer;

/**
  * @brief
  * @param   4*4����ɨ��
  * @retval  0~16����Ӧ16������
  *
  *        1(1��)     2(5��)     3(9 ��)     A(13��)
  *        4(2��)     5(6��)     6(10��)     B(14��)
  *        7(3��)     8(7��)     9(11��)     C(15��)
  *        *(4��)     0(8��)     #(12��)     D(16��)
  */
char Key_Scan(void);

/**
  * @brief
  * @param   4*4����ɨ��
  * @retval  0~9,a,b,c,d,*,#
  *
  *        1(1��)     2(5��)     3(9 ��)     A(13��)
  *        4(2��)     5(6��)     6(10��)     B(14��)
  *        7(3��)     8(7��)     9(11��)     C(15��)
  *        *(4��)     0(8��)     #(12��)     D(16��)
  */
char Key_ScanChar(void);

/**
  * @brief  ������ɨ��
  * @param
  * @retval 1~4����Ӧ1~4�а���λ��
  */
char Key_Row_Scan(void);







#endif

