#include "./MATRIXKEY/bsp_matrixkey.h"



char Key_Row_Scan(void)
{
  uint8_t Key_row[1]={0xff};   //���水����ɨ�������״̬����
  //������ɨ��״̬
  Key_row[0] = HAL_GPIO_ReadPin(KEY_Row0_GPIO,KEY_Row0_Pin)<<3;
  Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(KEY_Row1_GPIO,KEY_Row1_Pin)<<2);
  Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(KEY_Row2_GPIO,KEY_Row2_Pin)<<1);
  Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(KEY_Row3_GPIO,KEY_Row3_Pin));
  
  if(Key_row[0] != 0x0f)         //��ɨ���б仯���жϸ����а�������
  {
    HAL_Delay(10);                    //����
    if(Key_row[0] != 0x0f)
    {   
      //printf("Key_Row_DATA = 0x%x\r\n",Key_row[0]);
      switch(Key_row[0])
      {
        case 0x07:         //0111 �ж�Ϊ���е�1�еİ�������
          return 1;
        case 0x0b:         //1011 �ж�Ϊ���е�2�еİ�������
          return 2;
        case 0x0d:         //1101 �ж�Ϊ���е�3�еİ�������
          return 3;
        case 0x0e:         //1110 �ж�Ϊ���е�4�еİ�������
          return 4;
        default :
          return 0;
        }
    }
    else return 0;
  }
  else return 0;
}


char Key_Scan(void)
{    
  char Key_Num = 0;          //1-16��Ӧ�İ�����
  char key_row_num = 0;      //��ɨ������¼
  
  KEY_CLO0_OUT_LOW;        
  if( (key_row_num=Key_Row_Scan()) != 0 )
  { 
    while(Key_Row_Scan() != 0);  //����
    Key_Num = 0 + key_row_num;
  }
  KEY_CLO0_OUT_HIGH;
    
  KEY_CLO1_OUT_LOW;        
  if( (key_row_num=Key_Row_Scan()) != 0 )
  { 
    while(Key_Row_Scan() != 0);
    Key_Num = 4 + key_row_num;
  }
  KEY_CLO1_OUT_HIGH;
   
  KEY_CLO2_OUT_LOW;    
  if( (key_row_num=Key_Row_Scan()) != 0 )
  { 
    while(Key_Row_Scan() != 0);
    Key_Num = 8 + key_row_num;
  }
  KEY_CLO2_OUT_HIGH;
    
  KEY_CLO3_OUT_LOW;    
  if( (key_row_num=Key_Row_Scan()) != 0 )
  {
    while(Key_Row_Scan() != 0);
    Key_Num = 12 + key_row_num;
  }
  KEY_CLO3_OUT_HIGH;
    
  return Key_Num;
}

char Key_ScanChar(void){
  char keyNum = Key_Scan();
  switch(keyNum){
    case 1:
      return '1';
    case 2:
      return '4';
    case 3:
      return '7';
    case 4:
      return '*';
    case 5:
      return '2';
    case 6:
      return '5';
    case 7:
      return '8';
    case 8:
      return '0';
    case 9:
      return '3';
    case 10:
      return '6';
    case 11:
      return '9';
    case 12:
      return '#';
    case 13:
      return 'A';
    case 14:
      return 'B';
    case 15:
      return 'C';
    case 16:
      return 'D';
    default:
      return 0;
  }
}
