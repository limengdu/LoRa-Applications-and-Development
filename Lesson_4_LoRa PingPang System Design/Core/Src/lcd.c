#include "gpio.h"
#include "stdint.h"
#include "lcd.h"
#include "font_lcd.h"
#include "string.h"
#include "spi.h"



void Delay_ms(int time)
{
	int i,j;
	for(i=0;i<time*10;i++)
	{
		for(j=0;j<100;j++)
		{
		
		}
	}
}


void LCD_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//向液晶屏写一个8位指令
void Lcd_WriteIndex(uint8_t Index)
{
   //SPI 写命令时序开始               
    //NSS = 0;
    LCD_CS_CLR;       
    LCD_RS_CLR;           //LCD_RS_CLR
    HAL_SPI_Transmit(&hspi1,&Index,1,0xfff);
    //NSS = 1;
    LCD_CS_SET;   
}
//向液晶屏写一个8位数据
void Lcd_WriteData(uint8_t Data)
{
	LCD_CS_CLR;
	LCD_RS_SET;
        HAL_SPI_Transmit(&hspi1,&Data,1,0xfff);
	LCD_CS_SET;
}

void LCD_WriteData_16Bit(uint16_t Data)
{
  uint8_t Data_H = Data>>8;
  uint8_t Data_L = Data&0xFF;
	 LCD_CS_CLR;
	 LCD_RS_SET;
	 HAL_SPI_Transmit(&hspi1,&Data_H,1,0xfff); 	//写入高8位数据
	 HAL_SPI_Transmit(&hspi1,&Data_L,1,0xfff); 	//写入低8位数据	
	 LCD_CS_SET;
}

 //LCD Init For 1.44Inch LCD Panel with ST7735R.
void Lcd_Init(void)
{	
  
        LCD_GPIO_Init();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);      
        
        Lcd_WriteIndex(0x01);//Sleep exit 
        
        HAL_Delay(120);
  
	Lcd_WriteIndex(0x11);//Sleep exit 
	
        HAL_Delay(120);
 
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC8); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	Lcd_WriteIndex(0x29);//Display on		
}

/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void Lcd_SetRegion(uint16_t x_start,uint16_t y_start,uint16_t x_end,uint16_t y_end)
{		
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+3);
	
	Lcd_WriteIndex(0x2c);

}
/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	 LCD_WriteData_16Bit(Color);
    }   
}

/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void Gui_DrawPoint(uint16_t x,uint16_t y,uint16_t Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}    

void Gui_DrawFont_GBK16(uint16_t x0, uint16_t y0, uint16_t fc, uint16_t bc, uint8_t *s)
{ 
  int i,j,k,x,y,xx;
  
  unsigned char qm;
  
  long int ulOffset;
  
  char  ywbuf[32];
 // char   temp[2];
  
  for(i = 0; i<strlen((char*)s);i++)
  {
    if(((unsigned char)(*(s+i))) >= 161)
    {
//      temp[0] = *(s+i);
//      temp[1] = '\0';
      return;
    }
    
    else
    {
      qm = *(s+i);
      
      ulOffset = (long int)(qm) * 16;
      
      for (j = 0; j < 16; j ++)
      {
        ywbuf[j]=Zk_ASCII8X16[ulOffset+j];
      }
      
      for(y = 0;y < 16;y++)
      {
        for(x=0;x<8;x++) 
        {
          k=x % 8;
          
          if(ywbuf[y]&(0x80 >> k))
          {
            xx=x0+x+i*8;     
            Gui_DrawPoint(xx,y+y0,fc);
          }
          else
          {
            
            xx=x0+x+i*8;     
            Gui_DrawPoint(xx,y+y0,bc);
          }  
        }
      }
      
    }
  }  
}

/*************************************************
函数名：showimage
功能：显示一副图片
入口参数：图片缓存
返回值：无
*************************************************/
//因为一副128*128的图片需要32768个8位数组成（数组元素有32768个），很显然数组太大，不能放到内存中，只能放在静态存储区，所以参数使用 const 修饰 
void showimage(const unsigned char *p)             
{	
   unsigned int i;
   uint16_t HData,LData;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i = 0;i < 128*128;i++)//因为我们已经设置显示屏显示范围为128*128，所以写满一行会自动换行，不需单独写代码
   {
     LData = *(p+i*2);
     HData = *(p+i*2+1);
     LCD_WriteData_16Bit(HData<<8|LData);
   }       
}
