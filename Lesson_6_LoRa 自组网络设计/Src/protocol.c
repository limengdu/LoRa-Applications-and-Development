#include "protocol.h"





/******************************************************************************
* Name:    CRC-8               x8+x2+x+1
* Poly:    0x07
* Init:    0x00
* Refin:   False
* Refout:  False
* Xorout:  0x00
* Note:
*****************************************************************************/
uint8_t crc8(uint8_t *data, uint8_t length)
{
  uint8_t i;
  uint8_t crc = 0;        // Initial value
  while(length--)
  {
    crc ^= *data++;        // crc ^= *data; data++;
    for ( i = 0; i < 8; i++ )
    {
      if ( crc & 0x80 )
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

//**********************************//
//
//函数名称：   DataCrcVerify
//
//函数描述：   CRC8校验
//
//函数参数：   uint8_t * buff, uint8_t len
//
//返回值：     uint8_t
//
//*******************************//

uint8_t DataCrcVerify(uint8_t * buff, uint8_t len)
{
	uint8_t Crc8Data = 0;

	//验证数据是否正确 
	Crc8Data = crc8(buff, len - 1);

	if (Crc8Data == buff[len - 1])
	{
// 		PRINTF1("CRC8 Success!\n");
		return 1;
	}
	else
	{
//		PRINTF1("CRC8 Failed!\n");
		return 0;
	}
}