#ifndef _NETPROCESS_H
#define _NETPROCESS_H


#include "stm32f0xx.h"
#include "stdbool.h"

#define NodeNumber	20

extern volatile  uint16_t currentDeviceNumber;

extern volatile  uint16_t oldNodeNumber;

extern volatile uint32_t DataUpTime;

extern uint8_t startUpTimeHours;
extern uint8_t startUpTimeMinute;
extern uint8_t startUpTimeSeconds;
extern uint32_t startUpTimeSubSeconds;


extern uint8_t DataUpTimeHours;
extern uint8_t DataUpTimeMinute;
extern uint8_t DataUpTimeSeconds;
extern uint32_t DataUpTimeSubSeconds;
/************************************************************************/
/* 定义设备入网时的状态                                                 */
/************************************************************************/
typedef enum
{
	NO_JION = 0,    //未加入网络
	JIONING,	  //正在加入网络
	JIONTIMEOUT,  //入网超时
	JIONDONE,      //入网完成
	AGAIN_JION_NET
}DeviceJionStatus;


/************************************************************************/
/* 入网协议分析状态                                                                 */
/************************************************************************/
typedef enum 
{
	JION_HEADER = 0,
	JION_LENGHT,
	JION_TYPE,
	JION_PANID,
	JION_ADDR,
	JION_CRC
}JionProtocol;

/************************************************************************/
/* 定义设备节点加入标志                                                 */
/************************************************************************/
typedef enum
{
	No_Node_Jion_Flag = 0,
	Node_Jion_Finish_Flag,
	Node_Jion_No_Finish_Flag,
	New_Node_Jion_Flag
}DeviceJionFlag;

/************************************************************************/
/* 设备入网，发送的消息体                                               */
/************************************************************************/
typedef struct 
{
	uint8_t	msgHead;	//入网消息头0x3C
	uint8_t dataLength;	//数据长度 type~crc
	uint8_t netType;	//模块的网络类型
	uint8_t netPanid[2];	//设备的PANID
	uint8_t deviceAddr[2];	//模块的设备地址
	uint8_t crcCheck;	//数据校验（整个数据包，除校验位）
}SlaveJionNet;

/************************************************************************/
/* 设备数据，发送的消息体                                               */
/************************************************************************/
typedef struct 
{
	uint8_t	netmsgHead;	//入网消息头0x3C
        uint8_t netPanid[2];	//设备的PANID
        uint8_t	msgHead;	//数据消息头0x21
	uint8_t dataLength;	//数据长度 type~crc
        uint8_t dataType;	//模块的数据类型
	uint8_t deviceAddr[2];	//模块的设备地址
        uint8_t sensorType;	//模块的传感器类型
        uint8_t buff[4];        //每种传感器数值用两个字节标识，比如温湿度占四个字节
	uint8_t crcCheck;	//数据校验（整个数据包，除校验位）
}SlaveDataNet;

/************************************************************************/
/* 更新RTC，发送的消息体                                               */
/************************************************************************/
typedef struct
{
	uint8_t	msgHead;	//入网消息头0x3C
	uint8_t dataLength;	//数据长度 type~crc
	uint8_t netType;	//模块的网络类型
	uint8_t netPanid[2];	//设备的PANID
	uint8_t timeData[5];	//模块的设备地址
	uint8_t crcCheck;	//数据校验（整个数据包，除校验位）
}SlaveRtcSync;

/************************************************************************/
/* 设备信息                                                             */
/************************************************************************/
typedef struct  
{
	uint8_t deviceType;	//模块的设备类型
	DeviceJionStatus deviceNetStatus;	//设备的网络状态
	uint8_t deviceAddr[2];	//模块的设备地址
	uint8_t deviceId;	//表示在网表中加入第几个设备
	uint8_t deviceData[20];
}SlaveInfo;



uint16_t RandomNumber(void);
uint8_t SlaveJionNetFuction(void);

void SlaveGetSendTime(void);



DeviceJionFlag WaitJionNetFinish(uint8_t timout);
void MasterSendClockData(void);

#endif