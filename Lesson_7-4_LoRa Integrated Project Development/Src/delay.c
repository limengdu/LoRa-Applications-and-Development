#include "stm32f0xx_hal.h"
#include "delay.h"


/**
  * @}
  */
void HAL_Delay_10us(__IO uint32_t Delay)
{
        HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	uint32_t temp;
	uint8_t fac_us=60;	        
	SysTick->LOAD=Delay*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	
}
/**
  * @}
  */
void HAL_Delay_ms(__IO uint32_t Delay)
{
        HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	uint32_t temp;
	uint16_t fac_ms=6000;	        
	SysTick->LOAD=Delay*fac_ms; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	
}
/**
  * @}
  */
void HAL_Delay_us(__IO uint32_t Delay)
{
        HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	uint32_t temp;
	uint8_t fac_us=6;	        
	SysTick->LOAD=Delay*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	
}

