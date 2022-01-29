#include "gpio.h"
#include "fan.h"
#include <stdbool.h>


static uint8_t FanStaus = false;

//**********************************//
//
//�������ƣ�   FanOn
//
//����������   ��������
//
//����������   ��
//
//����ֵ��     ��
//
//*******************************//

void FanOn(void)
{
    HAL_GPIO_WritePin( FAN_GPIO_PORT, FAN_PIN, FAN_ON );
    FanStaus = true;
}

//**********************************//
//
//�������ƣ�   FanOff
//
//����������   �رշ���
//
//����������   ��
//
//����ֵ��     ��
//
//*******************************//


void FanOff(void)
{
    HAL_GPIO_WritePin( FAN_GPIO_PORT, FAN_PIN, FAN_OFF ); 
    FanStaus = false;
}


//**********************************//
//
//�������ƣ�   FanReadStaus
//
//����������   ��ȡ����״̬
//
//����������   ��
//
//����ֵ��     ��
//
//*******************************//


uint8_t FanReadStaus( void )
{
    return FanStaus;
}


