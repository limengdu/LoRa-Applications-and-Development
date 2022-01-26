#ifndef _MPU6050_H
#define _MPU6050_H


#include "stm32f0xx.h"
#include "stdbool.h"
extern int16_t mpu6050_ACC_X,mpu6050_ACC_Y,mpu6050_ACC_Z;

//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define SlaveAddress     (MPU6050_ADDRESS_AD0_HIGH<<1)
//#define	SlaveAddress		//IIC写入时的地址字节数据，+1为读取
#define	ADDRESS_Write   SlaveAddress | 0x00                //
#define	ADDRESS_Read   SlaveAddress | 0x01	                //

void InitMpu6050(void);	
void  mpu6050_ReadData(float *Mx, float *My, float *Mz);
void  mpu6050_verify(int16_t *x, int16_t *y, int16_t *z);



#endif