#include "dataprocess.h"
#include "usart.h"
#include "led.h"
#include "protocol.h"
#include "rtc.h"



#include "string.h"
#include "stdio.h"
//sx1278
#include "platform.h"
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"




extern uint16_t BufferSize;
extern uint8_t Buffer[BUFFER_SIZE];

#if defined(MASTER)
extern uint8_t EnableMaster;

#elif defined(SLAVE)

extern uint8_t EnableMaster;

#endif


extern tRadioDriver *Radio;

extern uint32_t Master_RxNumber;
extern uint32_t Master_TxNumber;

extern uint32_t Slave_RxNumber;
extern uint32_t Slave_TxNumber;

extern volatile uint8_t SendDataOkFlag;


uint8_t startUpDateHours = 0;
uint8_t startUpDateMinute = 0;
uint8_t startUpDateSeconds = 0;
uint16_t startUpDateSubSeconds = 0;


//Master存储入网的设备信息
SlaveInfo slaveNetInfo_t[NodeNumber];

//Salve入网信息包
SlaveJionNet jionPacke_t;

//Salve保存自己的地址
SlaveInfo slaveNativeInfo_t;
//节点数据
SlaveDataNet DataPacke_t;



//**********************************//
//
//函数名称：UartDmaGet   
//
//函数描述：串口数据获取   
//
//函数参数：   无
//
//返回值：     无
//
//*******************************//

void UartDmaGet(void)
{
  if(UsartType1.receive_flag == 1)//如果过新的数据
  {

    //串口接收到的数据原封发给SX1278
    Radio->SetTxPacket(UsartType1.usartDMA_rxBuf, UsartType1.Usart_rx_len);
    
    memset(UsartType1.usartDMA_rxBuf,0,UsartType1.Usart_rx_len);
    UsartType1.receive_flag = 0; //接收数据标志清零，
  }
}

//**********************************//
//
//函数名称：  RxDataPacketNum 
//
//函数描述：  接收数据包计数 
//
//函数参数：   无
//
//返回值：     无
//
//*******************************//

void RxDataPacketNum(void)
{
  if(EnableMaster == true)
    Master_RxNumber++;
  else
    Slave_RxNumber++;
}
//**********************************//
//
//函数名称：   TxDataPacketNum
//
//函数描述：   发送数据包计数
//
//函数参数：  无 
//
//返回值：     无
//
//*******************************//

void TxDataPacketNum(void)
{
  if(EnableMaster == true)
    Master_TxNumber++;
  else
    Slave_TxNumber++;
}
//**********************************//
//
//函数名称：  Sx127xDataGet 
//
//函数描述：   读取sx127x射频射频数据
//
//函数参数：   无
//
//返回值：     无
//
//*******************************//

uint8_t Sx127xDataGet(void)
{
  uint8_t status = 0;
  switch( Radio->Process( ) )
  {
  case RF_RX_TIMEOUT:
    printf("RF_RX_TIMEOUT\n");
    break;
  case RF_RX_DONE:
    Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
    if(EnableMaster == true)
      printf("master Rx Len = %d\n",BufferSize);
    else
      printf("slave Rx Len = %d\n",BufferSize);      
    if( BufferSize > 0 )//&& (BufferSize == strlen((char*)Buffer)))
    {
      //接收数据闪烁
      LedBlink( LED_RX );
      //计算接收数据的个数
      RxDataPacketNum();

      //清空sx127x接收缓冲区
#ifdef MASTER
      status = MasterProtocolAnalysis(Buffer,BufferSize);
#else      
      
      status = SlaveProtocolAnalysis(Buffer, BufferSize);
#endif
      memset(Buffer,0,BufferSize);
    }            
    break;
  case RF_TX_DONE:
    //发送闪烁
    LedBlink( LED_TX );
    //计算发送数据的个数
    TxDataPacketNum();
    Radio->StartRx( );
    SendDataOkFlag = 1;
    break;
  case RF_TX_TIMEOUT:
    printf("RF_TX_TIMEOUT\n");
    break; 
  default:
    break;
  }
  return status;
}


//**************从**************//
//**************机**************//



//**********************************//
//
//函数名称：   SendJionNetPacke
//
//函数描述：   从机入网数据发送
//
//函数参数：   无
//
//返回值：     无
//
//*******************************//

void SendJionNetPacke(void)
{
  uint16_t addr = ADDR;
    jionPacke_t.msgHead = 0x3C;
    jionPacke_t.dataLength = 0x06;
    jionPacke_t.netType = 'J';
    jionPacke_t.netPanid[0] = HI_UINT16(PAN_ID);
    jionPacke_t.netPanid[1] = LO_UINT16(PAN_ID);
    jionPacke_t.deviceAddr[0] = HI_UINT16(ADDR);
    jionPacke_t.deviceAddr[1] = LO_UINT16(ADDR);
    //校验码
    jionPacke_t.crcCheck = crc8((uint8_t *)&jionPacke_t,jionPacke_t.dataLength + 1);
    
    printf("SendJionNetPacke addr = %d\n",addr);
    //发送数据包
    Radio->SetTxPacket((uint8_t *)&jionPacke_t, jionPacke_t.dataLength + 2);
  
}

//**********************************//
//
//函数名称：   SlaveProtocolAnalysis
//
//函数描述：   从机协议解析
//
//函数参数：   uint8_t *buff,uint8_t len
//
//返回值：     uint8_t
//
//*******************************//

uint8_t SlaveProtocolAnalysis(uint8_t *buff,uint8_t len)
{
  uint8_t Crc8Data;
  
  printf("SlaveProtocolAnalysis\n");
  for (int i = 0; i < len; i++)
  {
    printf("0x%x  ",buff[i]);
    
 
  }
  printf("\n");
  
  
  if (buff[0] == NETDATA)
  {
    if (buff[1] == HI_UINT16(PAN_ID) && buff[2] == LO_UINT16(PAN_ID))
    {
      Crc8Data = crc8(&buff[3], len - 4);
      
      if (Crc8Data != buff[len - 1])
      {
      
        memset(buff, 0, len);
        return 0;
      
      }
      if (buff[3] == 0x21)
      {
      
      
        printf("Slave_NETDATA\n");
      
      
      
      }
      return 0;
      
    
    
    
    
    }
  
  
  
  
  
  
  }
  else if((buff[0] == 0x3C) && (buff[2] == 'A'))
  {
  
    if (DataCrcVerify(buff, len) == 0)
    {
      return 0;
    }
    if (buff[3] == HI_UINT16(PAN_ID) && buff[4] == LO_UINT16(PAN_ID))
    {
      if (buff[5] == jionPacke_t.deviceAddr[0] && buff[6] == jionPacke_t.deviceAddr[1])
      {
        slaveNativeInfo_t.deviceId = buff[7];
        printf("Slave_ACK\n");
        return 0xFF;
      
      
      
      }
    
    
    
    }
  
  
  
  
  }
  else if((buff[0] == 0x3C) && (buff[2] == 'T'))
  {
    if (DataCrcVerify(buff, len) == 0)
    {
      return 0;
    }
    if (buff[3] == HI_UINT16(PAN_ID) && buff[4] == LO_UINT16(PAN_ID))
    {
      uint32_t alarmTime = 0;
      startUpTimeHours = buff[5];
      startUpTimeMinute = buff[6];
      startUpTimeSeconds = buff[7];
      startUpTimeSubSeconds = buff[8] <<8 | buff[9];
      printf("Slave_CLOCK\n");
      printf("H:%d,M:%d,S:%d,SUB:%d\n", startUpTimeHours, startUpTimeMinute, startUpTimeSeconds, startUpTimeSubSeconds);
      alarmTime = ((DataUpTimeHours * 60 + DataUpTimeMinute) * 60 
                   + DataUpTimeSeconds) * 1000 + (DataUpTimeSubSeconds / 2) + DataUpTime;
      GetTimeHMS(alarmTime, &DataUpTimeHours, &DataUpTimeMinute, &DataUpTimeSeconds, &DataUpTimeSubSeconds);
      printf("DataUpTime->H:%d,M:%d,S:%d,SUB:%d\n", DataUpTimeHours, DataUpTimeMinute, DataUpTimeSeconds, DataUpTimeSubSeconds);
      //使能RTC
      MX_RTC_Init();
     return 0xFF;
    
    
    }
  
  
  
  
  
  }


  return 1;


}

//**********************************//
//
//函数名称：   SendSensorDataUP
//
//函数描述：   上传节点传感器数据
//
//函数参数：   无
//
//返回值：     无
//
//*******************************//

void SendSensorDataUP(void)
{
    
  printf("SendSensorDataUP\n");
  DataPacke_t.netmsgHead = 'N';
  DataPacke_t.netPanid[0] = HI_UINT16(PAN_ID);
  DataPacke_t.netPanid[1] = LO_UINT16(PAN_ID);
  DataPacke_t.msgHead = 0x21;
  DataPacke_t.dataLength = 0x09;
  DataPacke_t.dataType = 0;
  DataPacke_t.deviceAddr[0] = HI_UINT16(ADDR);
  DataPacke_t.deviceAddr[1] = LO_UINT16(ADDR);
  DataPacke_t.sensorType  = 0x1;
  DataPacke_t.buff[0]  = 0x1;
  DataPacke_t.buff[1]  = 0x2;
  DataPacke_t.buff[2]  = 0x3;
  DataPacke_t.buff[3]  = 0x4;
    
  //校验码
  DataPacke_t.crcCheck = crc8((uint8_t *)&DataPacke_t,DataPacke_t.dataLength + 4);
  //发送数据包
  Radio->SetTxPacket((uint8_t *)&DataPacke_t, DataPacke_t.dataLength + 5);
}


//**************主**************//
//**************机**************//






//**********************************//
//
//函数名称：   MasterProtocolAnalysis
//
//函数描述：   主机协议解析
//
//函数参数：   uint8_t *buff,uint8_t len
//
//返回值：     uint8_t
//
//*******************************//

uint8_t MasterProtocolAnalysis(uint8_t *buff,uint8_t len)
{
  
  uint8_t Crc8Data,deviceID;
  
  uint8_t SendAck[12];
  
  printf("MasterProtocolAnalysis\n");
  for (int i = 0; i < len; i++)
  {
    printf("0x%x  ",buff[i]);
    
 
  }
  printf("\n");
  
  if(buff[0] == NETDATA)
  {
    if((buff[1] == HI_UINT16(PAN_ID))&&(buff[2] == LO_UINT16(PAN_ID)))
    {
      Crc8Data = crc8(&buff[0], len - 1); //减去校验
      if(Crc8Data != buff[len - 1])
      {
        memset(buff,0,len);//清空缓存区
        
        return 0;
      
      
      }
      
      if(buff[3] == DATAHEAD)
      {
      
        NetDataProtocolAnalysis(&buff[3], len - 3);
        
      
      }
    
    
    
    
    
    }
    else
      return 0;
  
  
  }
  else if(buff[0] == JIONREQUEST)
  {
  
      deviceID = JionNetProtocolAnalysis(buff, len);
      printf("deviceID = %d\n",deviceID);
      
      if(deviceID >= 0)
      {
        SendAck[0] = JIONREQUEST;
        SendAck[1] = 1;
        SendAck[2] = 'A';
        SendAck[3] = HI_UINT16(PAN_ID);
        SendAck[4] = LO_UINT16(PAN_ID);
        SendAck[5] = slaveNetInfo_t[deviceID].deviceAddr[0];
        SendAck[6] = slaveNetInfo_t[deviceID].deviceAddr[1];
        SendAck[7] = deviceID;
        SendAck[8] = crc8(SendAck, 8);
        Radio->SetTxPacket(SendAck, 9);
        printf("MasterAck\n");
        for (int i = 0; i < 9; i++)
        {
          printf("0x%x  ",SendAck[i]);
          
       
        }
        printf("\n");
      }
  }
  
  return 1;

}



//**********************************//
//
//函数名称：   JionNetProtocolAnalysis
//
//函数描述：   入网协议解析
//
//函数参数：   uint8_t *buff,uint8_t len
//
//返回值：     uint8_t
//
//*******************************//

uint8_t JionNetProtocolAnalysis(uint8_t *buff,uint8_t len)
{
  uint8_t i = 0, dataLen = 0;
  uint8_t status = 0, lenOld = len;
  
  printf("JionNetProtocolAnalysis\n");
  for (int i = 0; i < len; i++)
  {
    printf("0x%x  ",buff[i]);
    
 
  }
  printf("\n");
  while(len--)
  {
    switch(status)
    {
      case JION_HEADER:
        if (buff[status] == JIONREQUEST)
        {
          status = JION_LENGHT;
        
        } 
        else
        {
          goto ERR;
        }
      break;
      case JION_LENGHT:
       if (buff[status] == 0x06)
        {
          status = JION_TYPE;
        
        } 
        else
        {
          goto ERR;
        }
      break;
      case JION_TYPE:
        if (buff[status] == 'J')
        {
          status = JION_PANID;
        
        } 
        else
        {
          goto ERR;
        }
      break;
      case JION_PANID:
        if (buff[status] == HI_UINT16(PAN_ID) && buff[status + 1] == LO_UINT16(PAN_ID))
        {
          status = JION_ADDR;
        
        } 
        else
        {
          goto ERR;
        }
      break;
      case JION_ADDR:
      //旧节点加入
        for (i = 0; i < currentDeviceNumber; i++)
        {
          if ((slaveNetInfo_t[i].deviceAddr[0] == buff[status + 1]) &&
                (slaveNetInfo_t[i].deviceAddr[1] == buff[status + 2]))
          {
            slaveNetInfo_t[i].deviceNetStatus = AGAIN_JION_NET;
            status = JION_CRC;  
            printf("AGAIN_JION_NET i = %d\n",i);
            printf("deviceId=%x\n",slaveNetInfo_t[i].deviceId);
            printf("deviceAddr[0]=%x\n",slaveNetInfo_t[i].deviceAddr[0]);
            printf("deviceAddr[1]=%x\n",slaveNetInfo_t[i].deviceAddr[1]);
            break;
          
          }
        
        }
        //新节点加入
        if(i == currentDeviceNumber)
        {
          currentDeviceNumber++;//新增加入节点
          slaveNetInfo_t[i].deviceId = i;
          slaveNetInfo_t[i].deviceAddr[0] = buff[status + 1];
          slaveNetInfo_t[i].deviceAddr[1] = buff[status + 2];
          status = JION_CRC;
          printf("CURRENT_JION_NET i = %d\n",i);
          printf("deviceId=%x\n",slaveNetInfo_t[i].deviceId);
          printf("deviceAddr[0]=%x\n",slaveNetInfo_t[i].deviceAddr[0]);
          printf("deviceAddr[1]=%x\n",slaveNetInfo_t[i].deviceAddr[1]);

        }
      break;
      case JION_CRC:
      //更新节点入网状态
            if (slaveNetInfo_t[i].deviceNetStatus != AGAIN_JION_NET)  
            {
              slaveNetInfo_t[i].deviceNetStatus = JIONDONE;
              status = JION_HEADER;
              printf("JIONDONE i = %d\n",i);
              printf("deviceId=%x\n",slaveNetInfo_t[i].deviceId);
              printf("deviceAddr[0]=%x\n",slaveNetInfo_t[i].deviceAddr[0]);
              printf("deviceAddr[1]=%x\n",slaveNetInfo_t[i].deviceAddr[1]);
            
            }
      break;
      default:
      break;
    
    
    }
  
  
  }
  return i;
  


ERR:
  memset(buff, 0, lenOld);
  status = JION_HEADER;
  return -1;


}

//**********************************//
//
//函数名称：   NetDataProtocolAnalysis
//
//函数描述：   网络数据包解析
//
//函数参数：   uint8_t *buff,uint8_t len
//
//返回值：   无  
//
//*******************************//

void NetDataProtocolAnalysis(uint8_t *buff,uint8_t len)
{
  printf("NetDataProtocolAnalysis\n");
  for (int i = 0; i < len; i++)
  {
    printf("0x%x  ",buff[i]);
    
 
  }
  printf("\n");
}