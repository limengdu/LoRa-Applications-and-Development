/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include "lcd.h"
#include "led.h"
#include "logo.h"
#include "radio.h"
#include "string.h"

//sx1278
#include "platform.h"
#include "sx1276-Hal.h"
#include "sx12xxEiger.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"

#define BUFFERSIZE 128

uint8_t PingMsg[] = "PING";
uint8_t PangMsg[] = "PANG";

uint16_t BufferSize = BUFFERSIZE;
uint8_t Buffer[BUFFERSIZE];

#ifdef MASTER
uint8_t EnbleMaster = true;
#else
uint8_t EnbleMaster = false;
#endif

uint32_t Master_TxNumber = 0;
uint32_t Master_RxNumber = 0;

uint32_t Slave_TxNumber = 0;
uint32_t Slave_RxNumber = 0;

tRadioDriver *Radio = NULL;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


//�������ݻ�ȡ
void UartDmaGet(void) //���ڽ�����������
{
  if(UsartType1.receive_flag == 1)//������ͻ����������µ����ݣ����䷢�͸�sx1278
  {
    //���ڽ��յ�������ԭ�ⷢ��SX1278��sx1278ͨ�����߽����ݷ��ͳ�ȥ
    Radio->SetTxPacket(UsartType1.usartDMA_rxBuf, UsartType1.Usart_rx_len);
    
    memset(UsartType1.usartDMA_rxBuf,0,UsartType1.Usart_rx_len);
    UsartType1.receive_flag = 0; //�������ݱ�־����
  }
}

//�������ݰ�����
void RxDataPacketNum(void)
{
  if(EnbleMaster == true)
    Master_RxNumber++;
  else
    Slave_RxNumber++;
}

//�������ݰ�����
void TxDataPacketNum(void)
{
  if(EnbleMaster == true)
    Master_TxNumber++;
  else
    Slave_TxNumber++;
}

//��ȡsx127x��Ƶ��Ƶ����
void Sx127xDataGet(void)
{
  switch( Radio->Process( ) )
  {
  case RF_RX_TIMEOUT:
    printf("RF_RX_TIMEOUT\n");
    break;
  case RF_RX_DONE:
    Radio->GetRxPacket( Buffer, ( uint16_t* )&BufferSize );
    if(EnbleMaster == true)
      printf("master Rx__%s,__,%d,%d\n",Buffer,strlen((char*)Buffer),BufferSize);
    else
      printf("slave Rx__%s,__,%d,%d\n",Buffer,strlen((char*)Buffer),BufferSize);      
    if( BufferSize > 0 )//&& (BufferSize == strlen((char*)Buffer)))
    {
      //����������˸
      LedBlink( LED_RX );
      //����������ݵĸ���
      RxDataPacketNum();
 
      //���sx127x���ջ�����
      memset(Buffer,0,BufferSize );
    }            
    break;
  case RF_TX_DONE:
    //������˸
    LedBlink( LED_TX );
    //���㷢�����ݵĸ���
    TxDataPacketNum();
    Radio->StartRx();//�򿪽���ģʽ
    break;
  case RF_TX_TIMEOUT:
    printf("RF_TX_TIMEOUT\n");
    break; 
  default:
    break;
  }
}

//������ʾ����
void MLCD_Show(void){
  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();    //LCD��ʼ��
  
  sprintf((char*)str,"%d",Master_RxNumber);        //�����յ�������ʾ����
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);
  memset((char*)str,0,strlen((const char*)str));  //�����µ��ڴ��ʼ��
  
  sprintf((char*)str,"%d",Master_TxNumber);        //�����͵Ķ�����ʾ����
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  
  //SPI��ʼ��
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}

//�ӻ���ʾ����
void SLCD_Show(void){
  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();    //LCD��ʼ��
  
  sprintf((char*)str,"%d",Slave_RxNumber);         //�����յ�������ʾ����
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);
  memset((char*)str,0,strlen((const char*)str));  //�����µ��ڴ��ʼ��
  
  sprintf((char*)str,"%d",Slave_TxNumber);         //�����͵Ķ�����ʾ����
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  
  //SPI��ʼ��
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  uint8_t RegVersion = 0;

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  
  Lcd_Init();
  Lcd_Clear(YELLOW);
  
  //showimage(gImage_logo);
  HAL_Delay(500);
  Lcd_Clear(YELLOW);
  Gui_DrawFont_GBK16(0,0,RED,GREEN,"   LoRa Topology   ");
#ifdef MASTER
  Gui_DrawFont_GBK16(0,16,RED,GREEN,"      Master      ");
#else
  Gui_DrawFont_GBK16(0,16,RED,GREEN,"      Slave       ");
#endif
  
  Gui_DrawFont_GBK16(0,32,BLACK,YELLOW,"SSID:");
  Gui_DrawFont_GBK16(64,32,BLACK,YELLOW,"30");
  
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"RX:");
  Gui_DrawFont_GBK16(0,64,BLACK,YELLOW,"TX");
  
  //SPI��ʼ��
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  
  //��������1��ʹ�ܴ��ڿ����ж� ������Ϊ�����ж�ģʽ����������֮��Ž������ݵ��շ���Ȼ������жϣ������ظ��������ݵ��շ�����С������
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//ע�⣺ʹ�ܿ����жϺ󣬲�����û�н������ݣ����ȴ���һ���жϣ�����һ������ 
  HAL_UART_Receive_DMA(&huart1,a_Usart1_RxBuffer,RXLENGHT); //��DMA���գ����ڿ����ж�ģʽ���DMA�������ݵĽ���

  
  SX1276Read( REG_LR_VERSION, &RegVersion );
  
  if(RegVersion != 0x12)
  {
    printf("LoRa read Error!\r\n");
    printf("LoRa RegVersion = %d!\r\n",RegVersion);
  }
  else
  {
    printf("LoRa read OK!\r\n");
    printf("LoRa RegVersion = %d!\r\n",RegVersion);
  }
  
  //�����汾�ź󣬹ر�3�ֵ�
  LedOff(LED_RX);
  LedOff(LED_TX);
  LedOff(LED_NT);
  
  Radio = RadioDriverInit();
  Radio->Init();
  
  printf("system init ok!");
  
  Radio->StartRx( );//�򿪽���ģʽ

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    UartDmaGet();//���ڽ�����������
    
    Sx127xDataGet();//��ȡsx127x��Ƶ��Ƶ����
    
    if(EnbleMaster == true){
      MLCD_Show();       //������ʾ
    }
    else{
      SLCD_Show();       //�ӻ���ʾ
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

int fputc(int ch, FILE *f){
  while((USART1->ISR&0X40) == 0);
  USART1->TDR =(uint8_t)ch;
  return ch;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
