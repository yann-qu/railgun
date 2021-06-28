
#include "bsp_servo.h"

extern TIM_HandleTypeDef htim3;


/**
  * @brief 将pitch习惯角(0~90)转换为舵机绝对角(0~180)
  * @param pitch习惯角
  * @retval 舵机绝对角(0-180)
  */
uint16_t Get_Pitch_ServoAngle(uint16_t PitchAngle)
{
  return -PitchAngle + 90;
}

/**
  * @brief 将yaw习惯角(0~60)转换为舵机绝对角(0~180)
  * @param yaw习惯角
  * @retval舵机绝对角
  */
uint16_t Get_Yaw_ServoAngle(uint16_t YawAngle)
{
  return YawAngle + 60;
}

/**
  * @brief  72M-72-20000  将舵机绝对角转换为定时器计数值
  * @param angle:角度值，0~180
  * @retval
  */
uint16_t Servo_Control(uint16_t angle)
{
  uint16_t val;
  float temp;
  temp =(100.0 / 9.0) * angle + 500.0; // 0~180度对应计数器值500-2500(us)
  val = (uint16_t) temp;
  return val;   // 返回定时器装载值
}
