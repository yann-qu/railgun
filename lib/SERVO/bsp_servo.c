
#include "bsp_servo.h"

extern TIM_HandleTypeDef htim3;


/**
  * @brief ��pitchϰ�߽�(0~90)ת��Ϊ������Խ�(0~180)
  * @param pitchϰ�߽�
  * @retval ������Խ�(0-180)
  */
uint16_t Get_Pitch_ServoAngle(uint16_t PitchAngle)
{
  return -PitchAngle + 90;
}

/**
  * @brief ��yawϰ�߽�(0~60)ת��Ϊ������Խ�(0~180)
  * @param yawϰ�߽�
  * @retval������Խ�
  */
uint16_t Get_Yaw_ServoAngle(uint16_t YawAngle)
{
  return YawAngle + 60;
}

/**
  * @brief  72M-72-20000  ��������Խ�ת��Ϊ��ʱ������ֵ
  * @param angle:�Ƕ�ֵ��0~180
  * @retval
  */
uint16_t Servo_Control(uint16_t angle)
{
  uint16_t val;
  float temp;
  temp =(100.0 / 9.0) * angle + 500.0; // 0~180�ȶ�Ӧ������ֵ500-2500(us)
  val = (uint16_t) temp;
  return val;   // ���ض�ʱ��װ��ֵ
}
