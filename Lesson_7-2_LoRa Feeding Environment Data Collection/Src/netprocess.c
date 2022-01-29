#include "netprocess.h"
#include "dataprocess.h"
#include "tim.h"
#include "rtc.h"
#include "adc.h"
#include "protocol.h"

#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
//sx1278
#include "platform.h"
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"

//���нڵ�ĸ�������(��Time���ϴ���������) ��λMs
volatile uint32_t DataUpTimePeriod = 1000 *  60 * 1;	//1����

volatile static uint32_t currentTime = 0;
//��ǰ��������ĸ���
volatile  uint16_t currentDeviceNumber = 0;
//���浱ǰ����ڵ�
volatile  uint16_t oldNodeNumber = 0;
//�ڵ�ʱ��Ƭ
volatile uint32_t DataUpTime = 0;

//�ڵ�����״̬
volatile DeviceJionFlag JionNodeTimeOutFlag = No_Node_Jion_Flag;

uint8_t startUpTimeHours = 0;
uint8_t startUpTimeMinute = 0;
uint8_t startUpTimeSeconds = 0;
uint32_t startUpTimeSubSeconds = 0;


uint8_t DataUpTimeHours = 0;
uint8_t DataUpTimeMinute = 0;
uint8_t DataUpTimeSeconds = 0;
uint32_t DataUpTimeSubSeconds = 0;


//ʱ��ͬ��
SlaveRtcSync rtcSync_t;


//��ʼ������״̬
volatile DeviceJionStatus NetStatus = NO_JION;

extern tRadioDriver *Radio;


//**************��**************//
//**************��**************//


//**********************************//
//
//�������ƣ�   RandomNumber
//
//����������   ���������
//
//����������   ��
//
//����ֵ��     �����
//
//*******************************//

uint16_t RandomNumber(void)
{

  
    uint16_t randNumber = 0;
    float adcValue = 0;
    uint32_t u32adcValue = 0;
    
    //����DMAת��ADC
    HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_DMA_Value, ADC_NUM);
    
    HAL_Delay(100);
//    printf("ADC_DMA_Value[0] = %d\n",ADC_DMA_Value[0]);
//    printf("ADC_DMA_Value[1] = %d\n",ADC_DMA_Value[1]);
//    printf("ADC_DMA_Value[2] = %d\n",ADC_DMA_Value[2]);
//    printf("ADC_DMA_Value[3] = %d\n",ADC_DMA_Value[3]);
//    printf("ADC_DMA_Value[4] = %d\n",ADC_DMA_Value[4]);
    //ת��Ϊmvֵ
    adcValue = ADC_DMA_Value[ADC_IN5];
    adcValue = (adcValue * 3300) / 4096;  
    
    printf("adcValue = %f\n",adcValue);
    
    u32adcValue = (uint32_t)((adcValue-floor(adcValue))*1000000);
    printf("u32adcValue = %d\n",u32adcValue);
    //��ȡ�����
    srand(u32adcValue);
    for(int i = 0;i< 10;i++)
    randNumber += (uint8_t)rand();
    return randNumber;




}

//**********************************//
//
//�������ƣ�   SlaveJionNetFuction
//
//����������   �ӻ���������
//
//����������   ��
//
//����ֵ��     ����״̬
//
//*******************************//

uint8_t SlaveJionNetFuction(void)
{

  switch(NetStatus)
  {
    case NO_JION:
          SendJionNetPacke();
          //if(Radio->Process( ) == RF_TX_DONE)
          NetStatus = JIONING;
          currentTime = HAL_GetTick();
    break;
    case JIONING:
          if(Sx127xDataGet() == 0xFF)
          {
          
            NetStatus = JIONDONE;
            printf("Slave_JIONDONE\n");
          
          
          }
          else
          {
            if ((HAL_GetTick() - currentTime) > 6000)
            NetStatus = JIONTIMEOUT;
            
          }
        
    break;
    case JIONTIMEOUT:
        NetStatus = NO_JION;
    break;
    case JIONDONE:
          Radio->StartRx();
          return 0;
    break;
    default:
    break;
  }
return 1;

}
//**********************************//
//
//�������ƣ�   SlaveGetSendTime
//
//����������   �ڵ��ȡʱ��Ƭ
//
//����������   ��
//      
//����ֵ��     ��     
//
//*******************************//

void SlaveGetSendTime(void)
{
  float TransTimeUP = 0;		//���ݴ���ʱ��
  TransTimeUP = SX1276LoRaGetTransferTime();
  DataUpTime  = Sx127xGetSendTime(NodeNumber,TransTimeUP, DataUpTimePeriod);
  printf("DataUpTime = %d\n",DataUpTime);
  if (DataUpTime == 0)
  {
    startUpTimeHours = startUpTimeMinute = 0;
    startUpTimeSeconds = startUpTimeSubSeconds = 0;
  
  
  
  
  }
  else
  {
  
    GetTimeHMS(DataUpTime, &startUpTimeHours, &startUpTimeMinute, &startUpTimeSeconds, &startUpTimeSubSeconds);
    printf("DataUpTime->H:%d,M:%d,S:%d,SUB:%d\n", startUpTimeHours, startUpTimeMinute, startUpTimeSeconds, startUpTimeSubSeconds);
  
  
  
  }
  GetTimeHMS(DataUpTimePeriod, &DataUpTimeHours, &DataUpTimeMinute, &DataUpTimeSeconds, &DataUpTimeSubSeconds);
  printf("DataUpTimePeriod->H:%d,M:%d,S:%d,SUB:%d\n", DataUpTimeHours, DataUpTimeMinute, DataUpTimeSeconds, DataUpTimeSubSeconds);

}






//**************��**************//
//**************��**************//


//**********************************//
//
//�������ƣ�  WaiitJionNetFinish 
//
//����������  �ȴ�������� 
//
//����������  ��ʱʱ��
//
//����ֵ��     ��
//
//*******************************//

DeviceJionFlag WaitJionNetFinish(uint8_t timout)
{

  JionNodeTimeCount = 0;
  while(1)
  {
    Sx127xDataGet();
    if (JionNodeTimeCount > timout)
    {

      if (oldNodeNumber == currentDeviceNumber)
      {
          printf("���½ڵ����\r\n");
          //���½ڵ����
          JionNodeTimeOutFlag = Node_Jion_Finish_Flag;
          //ֹͣ��ʱ��
          HAL_TIM_Base_Stop_IT(&htim2);
          return JionNodeTimeOutFlag;
      
      
      }
      else
      {
          //���½ڵ����
          printf("���½ڵ����\r\n");
          JionNodeTimeOutFlag = Node_Jion_No_Finish_Flag;
          //���浱ǰ�ڵ�����
          oldNodeNumber = currentDeviceNumber;
      
      
      
      }

    }//�ȴ���������
    
    
    
    
    }
  

  
  }
//**********************************//
//
//�������ƣ�   MasterSendClockData
//
//����������   ��������ͬ��ʱ��
//
//����������   ��
//
//����ֵ��     ��
//
//*******************************//

void MasterSendClockData(void)
{
  RTC_TimeTypeDef thisTime;
  
  
  rtcSync_t.msgHead = JIONREQUEST;
  rtcSync_t.dataLength = 0x09;
  rtcSync_t.netType = 'T';
  rtcSync_t.netPanid[0] = HI_UINT16(PAN_ID);
  rtcSync_t.netPanid[1] = LO_UINT16(PAN_ID);
  
  //��ȡ��ǰʱ��
  HAL_RTC_GetTime(&hrtc, &thisTime, RTC_FORMAT_BIN);
  
  rtcSync_t.timeData[0] = thisTime.Hours;
  rtcSync_t.timeData[1] = thisTime.Minutes;
  rtcSync_t.timeData[2] = thisTime.Seconds;
  rtcSync_t.timeData[3] = (thisTime.SubSeconds >> 8) & 0xFF;
  rtcSync_t.timeData[4] = thisTime.SubSeconds & 0xFF;
  //����У����
  rtcSync_t.crcCheck = crc8((uint8_t *)&rtcSync_t, rtcSync_t.dataLength + 1);
  //�������ݰ�
  Radio->SetTxPacket((uint8_t *)&rtcSync_t, rtcSync_t.dataLength + 2);

}
