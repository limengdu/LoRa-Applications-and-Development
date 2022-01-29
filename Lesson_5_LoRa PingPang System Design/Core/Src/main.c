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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#define BUFFERSIZE 4

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//主机显示任务
void MLCD_Show(void){
  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();    //LCD初始化
  
  sprintf((char*)str,"%d",Master_RxNumber);        //将接收的内容显示出来
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);
  memset((char*)str,0,strlen((const char*)str));  //申请新的内存初始化
  
  sprintf((char*)str,"%d",Master_TxNumber);        //将发送的东西显示出来
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  
  //SPI初始化
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}

//从机显示任务
void SLCD_Show(void){
  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();    //LCD初始化
  
  sprintf((char*)str,"%d",Slave_RxNumber);         //将接收的内容显示出来
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);
  memset((char*)str,0,strlen((const char*)str));  //申请新的内存初始化
  
  sprintf((char*)str,"%d",Slave_TxNumber);         //将发送的东西显示出来
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  
  //SPI初始化
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}

//主机无线任务
void Master_Task(void){
  switch(Radio->Process()){       //无线任务处理进程判断
    case RF_RX_DONE:              //接收完成
      Radio->GetRxPacket(Buffer,&BufferSize);  //获取接收数据
      printf("Master_Task:RX______%s\n",Buffer);
      if(strncmp((const char*)Buffer,(const char*)PangMsg,strlen((const char*)PangMsg)) == 0){  //判断是否为Pang数据
        LedToggle(LED_RX);        //接收指示灯闪烁
        Master_RxNumber++;        //统计接收数据量
        Radio->SetTxPacket(PingMsg, strlen((const char*)PingMsg));  //发送Ping数据
      }
    break;
    case RF_TX_DONE:              //发送完成
      LedToggle(LED_TX);          //发送指示灯闪烁
      Master_TxNumber++;          //统计发送数据量
      Radio->StartRx();           //设置为接收状态
    break;
    default:
    break;
  }
}

//从机无线任务
void Slave_Task(void){
  switch(Radio->Process()){       //无线任务处理进程判断
    case RF_RX_DONE:              //接收完成
      Radio->GetRxPacket(Buffer,&BufferSize);  //获取接收数据
      printf("Slave_Task:RX______%s\n",Buffer);
      if(strncmp((const char*)Buffer,(const char*)PingMsg,strlen((const char*)PingMsg)) == 0){  //判断是否为Pang数据
        LedToggle(LED_RX);        //接收指示灯闪烁
        Slave_RxNumber++;         //统计接收数据量
        Radio->SetTxPacket(PingMsg, strlen((const char*)PangMsg));  //发送Ping数据
      }
    break;
    case RF_TX_DONE:             //发送完成
      LedToggle(LED_TX);         //发送指示灯闪烁
      Slave_TxNumber++;          //统计发送数据量
      Radio->StartRx();          //设置为接收状态
    break;
    default:
    break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  uint8_t RegVersion = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

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
  
  //SPI初始化
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  
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
  
  Radio = RadioDriverInit();
  Radio->Init();
  
#ifdef MASTER
  Radio->SetTxPacket(PingMsg, strlen((const char*)PangMsg));  //发送Ping数据
  printf("I am Master!");
#else
  Radio->StartRx();  //开启接收
  printf("I am Salve!");
#endif
  
  printf("system init ok!");
  
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    if(EnbleMaster == true){
      MLCD_Show();       //主机显示
      Master_Task();     //主机无线任务
    }
    else{
      SLCD_Show();       //从机显示
      Slave_Task();      //从机无线任务
    }
  }
  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
