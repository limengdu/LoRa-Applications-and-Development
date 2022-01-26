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
/* �����豸����ʱ��״̬                                                 */
/************************************************************************/
typedef enum
{
	NO_JION = 0,    //δ��������
	JIONING,	  //���ڼ�������
	JIONTIMEOUT,  //������ʱ
	JIONDONE,      //�������
	AGAIN_JION_NET
}DeviceJionStatus;


/************************************************************************/
/* ����Э�����״̬                                                                 */
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
/* �����豸�ڵ�����־                                                 */
/************************************************************************/
typedef enum
{
	No_Node_Jion_Flag = 0,
	Node_Jion_Finish_Flag,
	Node_Jion_No_Finish_Flag,
	New_Node_Jion_Flag
}DeviceJionFlag;

/************************************************************************/
/* �豸���������͵���Ϣ��                                               */
/************************************************************************/
typedef struct 
{
	uint8_t	msgHead;	//������Ϣͷ0x3C
	uint8_t dataLength;	//���ݳ��� type~crc
	uint8_t netType;	//ģ�����������
	uint8_t netPanid[2];	//�豸��PANID
	uint8_t deviceAddr[2];	//ģ����豸��ַ
	uint8_t crcCheck;	//����У�飨�������ݰ�����У��λ��
}SlaveJionNet;

/************************************************************************/
/* �豸���ݣ����͵���Ϣ��                                               */
/************************************************************************/
typedef struct 
{
	uint8_t	netmsgHead;	//������Ϣͷ0x3C
        uint8_t netPanid[2];	//�豸��PANID
        uint8_t	msgHead;	//������Ϣͷ0x21
	uint8_t dataLength;	//���ݳ��� type~crc
        uint8_t dataType;	//ģ�����������
	uint8_t deviceAddr[2];	//ģ����豸��ַ
        uint8_t sensorType;	//ģ��Ĵ���������
        uint8_t buff[4];        //ÿ�ִ�������ֵ�������ֽڱ�ʶ��������ʪ��ռ�ĸ��ֽ�
	uint8_t crcCheck;	//����У�飨�������ݰ�����У��λ��
}SlaveDataNet;

/************************************************************************/
/* ����RTC�����͵���Ϣ��                                               */
/************************************************************************/
typedef struct
{
	uint8_t	msgHead;	//������Ϣͷ0x3C
	uint8_t dataLength;	//���ݳ��� type~crc
	uint8_t netType;	//ģ�����������
	uint8_t netPanid[2];	//�豸��PANID
	uint8_t timeData[5];	//ģ����豸��ַ
	uint8_t crcCheck;	//����У�飨�������ݰ�����У��λ��
}SlaveRtcSync;

/************************************************************************/
/* �豸��Ϣ                                                             */
/************************************************************************/
typedef struct  
{
	uint8_t deviceType;	//ģ����豸����
	DeviceJionStatus deviceNetStatus;	//�豸������״̬
	uint8_t deviceAddr[2];	//ģ����豸��ַ
	uint8_t deviceId;	//��ʾ�������м���ڼ����豸
	uint8_t deviceData[20];
}SlaveInfo;



uint16_t RandomNumber(void);
uint8_t SlaveJionNetFuction(void);

void SlaveGetSendTime(void);



DeviceJionFlag WaitJionNetFinish(uint8_t timout);
void MasterSendClockData(void);

#endif