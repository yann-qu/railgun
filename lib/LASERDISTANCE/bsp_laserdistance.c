#include "bsp_laserdistance.h"
#include "stdio.h"
#include "stdlib.h"

uint8_t cmd_buffer[BUFFER_SIZE];

void Get_Distance()
{
  strcpy((char*)cmd_buffer, "iSM");
  HAL_UART_Transmit(&LASERDISTANCE_HUART, cmd_buffer, 3, 200);
  // while(!RECEIVE_FLAG);    // 确保数据已经获取，已经在中断中更新数据
}
