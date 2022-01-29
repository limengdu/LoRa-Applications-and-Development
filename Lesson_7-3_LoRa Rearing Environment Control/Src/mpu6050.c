#include "mpu6050.h"


#include "string.h"
#include "stdio.h"

#include "i2c.h"
int16_t Accx,Accy,Accz;


//**********************************//
//
//�������ƣ�  InitMpu6050 
//
//����������   ��ʼ��MPU6050
//
//����������   ��
//
//����ֵ��     ��
//
//*******************************//

void InitMpu6050(void)
{
  uint8_t WriteCmd = 0;
  
  //�������״̬
  WriteCmd = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &WriteCmd, 1, 0x10);
  //ʱ������0x06(1Khz)�����ǲ�����0x07(125Hz)
  WriteCmd = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &WriteCmd, 1, 0x10);      
  WriteCmd = 0x06;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteCmd, 1, 0x10);
  //���Լ죬2000deg/s
  WriteCmd = 0x18;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteCmd, 1, 0x10);
  //(���Լ죬2G��5Hz)
  WriteCmd = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS_Write, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &WriteCmd, 1, 0x10);

  HAL_Delay(10);
  mpu6050_verify(&Accx, &Accy, &Accz); //��ȡ��һ�ε�ֵ
  

}

//**********************************//
//
//�������ƣ�   mpu6050_verify
//
//����������   MPU6050У��
//
//����������   int16_t *x, int16_t *y, int16_t *z
//
//����ֵ��     ��
//
//*******************************//

void  mpu6050_verify(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t ReadBuffer[10] = {0};
  
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *x = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
   HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *y = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
        *z = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
        
        
}

//**********************************//
//
//�������ƣ�   mpu6050_ReadData
//
//����������   MPU6060��ȡ��������
//
//����������   int16_t *x, int16_t *y, int16_t *z
//
//����ֵ��     ��
//
//*******************************//

void  mpu6050_ReadData(float *Mx, float *My, float *Mz)
{
  int16_t x,y,z;
  
  uint8_t ReadBuffer[10] = {0};
         
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
  x = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
  x -= Accx;
  *Mx = ((float)x)/16384;
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_YOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
  y = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
  y -= Accy;
  *My = ((float)y)/16384;
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_L, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[0],1, 0x10);
  HAL_I2C_Mem_Read(&hi2c1, ADDRESS_Read, ACCEL_ZOUT_H, I2C_MEMADD_SIZE_8BIT,&ReadBuffer[1],1, 0x10);  
  z = (ReadBuffer[1]<<8)+ReadBuffer[0] ;
  z -= Accz;
  *Mz = ((float)z)/16384;

  

}





