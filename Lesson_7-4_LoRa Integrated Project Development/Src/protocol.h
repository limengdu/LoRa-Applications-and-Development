#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "stm32f0xx.h"



#define JIONREQUEST      0x3C

#define NETDATA          'N'

#define DATAHEAD         0x21


#define  PAN_ID                 0x1010

#ifdef MASTER
#define  ADDR                   0xFFFF
#else
#define  ADDR                   0x1201
#endif



#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)


uint8_t DataCrcVerify(uint8_t * buff, uint8_t len);
uint8_t crc8(uint8_t *data, uint8_t length);

#endif
