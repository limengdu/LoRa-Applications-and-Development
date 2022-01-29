#include "stdint.h"
#include "tim.h"
#include "gpio.h"
#include "dht11.h"
#include "delay.h"

//温湿度定义

uint8_t ucharT_data_H=0,ucharT_data_L=0,ucharRH_data_H=0,ucharRH_data_L=0,ucharcheckdata=0;

void DHT11_TEST(void)   //温湿传感启动
{
  uint8_t ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_humidity,ucharRH_data_L_humidity,ucharcheckdata_temp;
  uint8_t ucharFLAG = 0,uchartemp=0;
  uint8_t ucharcomdata;
  uint8_t i;  

  {
  D2_OUT_GPIO_Init();  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
  HAL_Delay_ms(18);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
  D2_IN_GPIO_Init();
  HAL_Delay_10us(4);
  }
    
    if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)) 
         {
            ucharFLAG=2; 
            while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
            ucharFLAG=2;
            while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)&&ucharFLAG++); 
            for(i=0;i<8;i++)    
              {
                 ucharFLAG=2; 
                 while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
                 HAL_Delay_10us(3);
                 uchartemp=0;
                 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))uchartemp=1;
                 ucharFLAG=2;
                 while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)&&ucharFLAG++);   
                 if(ucharFLAG==1)break;    
                 ucharcomdata<<=1;
                 ucharcomdata|=uchartemp; 
               }
            ucharRH_data_H_humidity = ucharcomdata;
            for(i=0;i<8;i++)    
              {
                 ucharFLAG=2; 
                 while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
                 HAL_Delay_10us(3);
                 uchartemp=0;
                 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))uchartemp=1;
                 ucharFLAG=2;
                 while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)&&ucharFLAG++);   
                 if(ucharFLAG==1)break;    
                 ucharcomdata<<=1;
                 ucharcomdata|=uchartemp; 
               }
            ucharRH_data_L_humidity = ucharcomdata;
            for(i=0;i<8;i++)    
              {
                 ucharFLAG=2; 
                 while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
                 HAL_Delay_10us(3);
                 uchartemp=0;
                 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))uchartemp=1;
                 ucharFLAG=2;
                 while((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);   
                 if(ucharFLAG==1)break;    
                 ucharcomdata<<=1;
                 ucharcomdata|=uchartemp; 
               }
            ucharT_data_H_temp      = ucharcomdata;
            for(i=0;i<8;i++)    
              {
                 ucharFLAG=2; 
                 while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
                 HAL_Delay_10us(3);
                 uchartemp=0;
                 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))uchartemp=1;
                 ucharFLAG=2;
                 while((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);   
                 if(ucharFLAG==1)break;    
                 ucharcomdata<<=1;
                 ucharcomdata|=uchartemp; 
               }
            ucharT_data_L_temp      = ucharcomdata;
            for(i=0;i<8;i++)    
              {
                 ucharFLAG=2; 
                 while((!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);
                 HAL_Delay_10us(3);
                 uchartemp=0;
                 if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))uchartemp=1;
                 ucharFLAG=2;
                 while((HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8))&&ucharFLAG++);   
                 if(ucharFLAG==1)break;    
                 ucharcomdata<<=1;
                 ucharcomdata|=uchartemp; 
               }
            ucharcheckdata_temp     = ucharcomdata;
//            humiture=1; 
            uchartemp=(ucharT_data_H_temp+ucharT_data_L_temp+ucharRH_data_H_humidity+ucharRH_data_L_humidity);
             
          if(uchartemp==ucharcheckdata_temp)
          {          
              ucharT_data_H  = ucharT_data_H_temp;
              ucharT_data_L  = ucharT_data_L_temp;
              ucharRH_data_H = ucharRH_data_H_humidity;
              ucharRH_data_L = ucharRH_data_L_humidity;
              ucharcheckdata = ucharcheckdata_temp;                    
          } 
       } 
    else //没用成功读取，返回0
       {
          ucharT_data_H  = 0;
          ucharT_data_L  = 0;
          ucharRH_data_H = 0;
          ucharRH_data_L = 0; 
       } 
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
}