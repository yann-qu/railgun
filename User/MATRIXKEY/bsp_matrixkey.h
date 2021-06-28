/**
  * @file      bsp_matrixkey.h
  * @author    yann
  * @version   0.1
  * @date      2021/05/13
  * @brief     矩阵键盘驱动
  * @attention 
  *      1. 注意修改引脚定义
  *      2. 行引脚上拉输入。
  *      3. 列引脚推挽输出、不上拉也不下拉、默认输出高电平
  */ 

#ifndef __BSP_MATRIXKEY_H_
#define __BSP_MATRIXKEY_H_

#include "stm32f1xx.h"


// 列引脚，推挽输出
#define KEY_Col0_GPIO GPIOB
#define KEY_Col1_GPIO GPIOB
#define KEY_Col2_GPIO GPIOB
#define KEY_Col3_GPIO GPIOB
#define KEY_Col0_Pin GPIO_PIN_15
#define KEY_Col1_Pin GPIO_PIN_14
#define KEY_Col2_Pin GPIO_PIN_13
#define KEY_Col3_Pin GPIO_PIN_12

// 行引脚，上拉输入
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
  * @param   4*4按键扫描
  * @retval  0~16，对应16个按键
  *
  *        1(1号)     2(5号)     3(9 号)     A(13号)
  *        4(2号)     5(6号)     6(10号)     B(14号)
  *        7(3号)     8(7号)     9(11号)     C(15号)
  *        *(4号)     0(8号)     #(12号)     D(16号)
  */
char Key_Scan(void);

/**
  * @brief
  * @param   4*4按键扫描
  * @retval  0~9,a,b,c,d,*,#
  *
  *        1(1号)     2(5号)     3(9 号)     A(13号)
  *        4(2号)     5(6号)     6(10号)     B(14号)
  *        7(3号)     8(7号)     9(11号)     C(15号)
  *        *(4号)     0(8号)     #(12号)     D(16号)
  */
char Key_ScanChar(void);

/**
  * @brief  按键行扫描
  * @param
  * @retval 1~4，对应1~4行按键位置
  */
char Key_Row_Scan(void);







#endif

