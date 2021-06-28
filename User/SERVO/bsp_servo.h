
#ifndef __BSP_SERVO_H_
#define __BSP_SERVO_H_

#include "stm32f1xx.h"

uint16_t Get_Pitch_ServoAngle(uint16_t PitchAngle);
uint16_t Get_Yaw_ServoAngle(uint16_t YawAngle);
uint16_t Servo_Control(uint16_t angle);

#endif

