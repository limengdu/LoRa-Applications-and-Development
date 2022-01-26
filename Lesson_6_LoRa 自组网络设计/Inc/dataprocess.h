#ifndef _DATAPROCESS_H
#define _DATAPROCESS_H


#include "stm32f0xx.h"
#include "netprocess.h"


#define BUFFER_SIZE 128

extern SlaveInfo slaveNetInfo_t[NodeNumber];
extern SlaveInfo slaveNativeInfo_t;

extern uint8_t startUpDateHours;
extern uint8_t startUpDateMinute;
extern uint8_t startUpDateSeconds;
extern uint16_t startUpDateSubSeconds;




void UartDmaGet(void);


uint8_t Sx127xDataGet(void);

void SendJionNetPacke(void);
void SendSensorDataUP(void);
uint8_t SlaveProtocolAnalysis(uint8_t *buff,uint8_t len);
void SendSensorDataUP(void);




uint8_t MasterProtocolAnalysis(uint8_t *buff,uint8_t len);





uint8_t JionNetProtocolAnalysis(uint8_t *buff,uint8_t len);


void NetDataProtocolAnalysis(uint8_t *buff,uint8_t len);



#endif