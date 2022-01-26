/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"
#include "lcd.h"
#include "led.h"
#include "mpu6050.h"
#include "dht11.h"
#include "fan.h"

#include "logo.h"
    
#include "dataprocess.h"    
    
#include "netprocess.h"        
#include "protocol.h"     

//sx1278
#include "platform.h"
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


float Mx,My,Mz;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

#if defined(MASTER)
uint8_t EnableMaster = true;

#elif defined(SLAVE)

uint8_t EnableMaster = false;

#endif


volatile uint8_t FanStaus = false;

tRadioDriver *Radio = NULL;

uint32_t Master_RxNumber = 0;
uint32_t Master_TxNumber = 0;

uint32_t Slave_RxNumber = 0;
uint32_t Slave_TxNumber = 0;



//�����豸��������ĵ�ǰ���
DeviceJionFlag	JionDeviceStatu = No_Node_Jion_Flag;

//ʱ��ͬ����־
volatile uint8_t MasterSendTimeSliceFlag = 0;

volatile uint8_t SendDataOkFlag = 0;

extern volatile uint8_t SendClockFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


//**********************************//
//
//�������ƣ�MLCD_Show   
//
//����������LoRa������Ļ��ʾ����   
//
//������������   
//
//����ֵ����  
//
//*******************************//

void MLCD_Show(void)
{
  
  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();
  
  
  
  sprintf((char*)str,"%d",Master_RxNumber);
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);
  
  memset((char*)str,0,strlen((char*)str));
  
  
  sprintf((char*)str,"%d",Master_TxNumber);
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();







}
//**********************************//
//
//�������ƣ�SLCD_Show   
//
//����������LoRa�ӻ���Ļ��ʾ����    
//
//������������   
//
//����ֵ��  �� 
//
//*******************************//

void SLCD_Show(void)
{


  uint8_t str[20] = {0};
  
  LCD_GPIO_Init();
  
#if defined(MPU6050)
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,"       ");
  memset((char*)str,0,strlen((char*)str));
  sprintf((char*)str,"%.3f",Mx);
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);

  
  
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,"       ");
  memset((char*)str,0,strlen((char*)str));
  sprintf((char*)str,"%.3f",My);
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);

  Gui_DrawFont_GBK16(64,80,BLACK,YELLOW,"       ");
  memset((char*)str,0,strlen((char*)str));
  sprintf((char*)str,"%.3f",Mz);
  Gui_DrawFont_GBK16(64,80,BLACK,YELLOW,str);
  
  
#elif defined(DHT11)

  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,"       ");
  memset((char*)str,0,strlen((char*)str));
  sprintf((char*)str,"%d",ucharT_data_H);
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,str);

  
  
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,"       ");
  memset((char*)str,0,strlen((char*)str));
  sprintf((char*)str,"%d",ucharRH_data_H);
  Gui_DrawFont_GBK16(64,64,BLACK,YELLOW,str);
  

  
  
#elif defined(FAN)
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,"       ");
  if (FanStaus)
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,"ON");
  else
  Gui_DrawFont_GBK16(64,48,BLACK,YELLOW,"OFF");
    
  
  
#endif  
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  
}



/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  uint8_t RegVersion = 0;
  uint8_t str[20] = {0};
  uint16_t addr = ADDR;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    uint32_t DelayTime = 0;


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
  MX_RTC_Init();
  MX_TIM2_Init();


  /* USER CODE BEGIN 2 */
  
#if defined (MPU6050)
  
 //��ʼ�����ᴫ����  
  MX_I2C1_Init();
  InitMpu6050();  
  
  
#elif defined (FAN)
  
  D1_OUT_GPIO_Init();
  
#endif
  
  
  
   Lcd_Init();
 // showimage(gImage_logo);
  HAL_Delay(500);
  Lcd_Clear(YELLOW);

  Gui_DrawFont_GBK16(0,0,RED,GREEN,"  LoRa Topology  ");
#if defined (SLAVE)  
  Gui_DrawFont_GBK16(0,16,RED,GREEN,"     Slave      ");
  

#if defined (MPU6050)  
  //���ᴫ������ʾ
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"X:");

  Gui_DrawFont_GBK16(0,64,BLACK,YELLOW,"Y:");

  Gui_DrawFont_GBK16(0,80,BLACK,YELLOW,"Z:");
#elif defined (DHT11)
  //������ʪ�ȴ�������ʾ
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"TEMP:");

  Gui_DrawFont_GBK16(0,64,BLACK,YELLOW,"HUM:");

#elif defined (FAN)
  //���ȴ�������ʾ
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"FAN:");
  
#endif
  
  
#elif defined (MASTER)
  Gui_DrawFont_GBK16(0,16,RED,GREEN,"     Master     ");
  Gui_DrawFont_GBK16(0,48,BLACK,YELLOW,"RX:");
  Gui_DrawFont_GBK16(0,64,BLACK,YELLOW,"TX:");
  
#endif
  Gui_DrawFont_GBK16(0,32,BLACK,YELLOW,"ADDR:");
  sprintf((char*)str,"%x",addr);
  Gui_DrawFont_GBK16(64,32,BLACK,YELLOW,str);


  

  
  
  
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
  
  
  //��������1��ʹ�ܴ��ڿ����ж�  
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE); 
  HAL_UART_Receive_DMA(&huart1,a_Usart1_RxBuffer,RXLENGHT); 

  SX1276Read( REG_LR_VERSION, &RegVersion );
  
  if(RegVersion != 0x12)
  {
    printf("LoRa read Error!\r\n");
    printf("LoRa RegVersion = %d!\r\n",RegVersion);
  
  }
  else
  {
    printf("LoRa read Ok!\r\n");
    printf("LoRa RegVersion = %d!\r\n",RegVersion);
  
  
  
  }
  
  //�����汾�ź󣬹ر�3�ֵ�
  LedOff(LED_RX);
  LedOff(LED_TX);
  LedOff(LED_NT);

  
  
  Radio = RadioDriverInit();
  
  Radio->Init();
  

  
  
  printf("systerm init ok!\n");
  
  Radio->StartRx( );  
  
    
  
  
#if SLAVE
  //��ȡ�������ʱ��
  DelayTime = RandomNumber();
  printf("JionTime = %d\n",DelayTime);
  HAL_Delay(DelayTime);
  //�ȴ������ɹ�
  while (SlaveJionNetFuction());
  //��ȡ�ڵ㷢��ʱ��Ƭ
  SlaveGetSendTime();
  
#else
    //����ֱ�ӳ�ʼ��RTC
    MX_RTC_Init();
    HAL_TIM_Base_Start_IT(&htim2);
  
#endif
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
    Sx127xDataGet();
    
#if SLAVE
    if(sendUpDataFlag == 1)
    {
    
      SendSensorDataUP();
      sendUpDataFlag = 0;
    
    
    
    }
    
    
#else
    UartDmaGet();
    //�ȴ��ڵ�����
    if (JionDeviceStatu != Node_Jion_Finish_Flag)
    {
      printf("main �ȴ���������\n");
      JionDeviceStatu = WaitJionNetFinish(10);
    }
    
        /* ���½ڵ���� */
    if (currentDeviceNumber != oldNodeNumber)
    {
      printf("main �½ڵ��������\n");
      HAL_TIM_Base_Start_IT(&htim2);
      JionDeviceStatu = New_Node_Jion_Flag;
      SendClockFlag = 0; //���ͷ�ʱʱ��Ƭ
    }
       /* �оɽڵ���� */
    for (int i = 0; i < currentDeviceNumber;i++)
    {
      /* ��ѯ�Ƿ��оɽڵ����¼���*/
      if (slaveNetInfo_t[i].deviceNetStatus == AGAIN_JION_NET)
      {
        printf("main �ɽڵ��������\n");
        slaveNetInfo_t[i].deviceNetStatus = JIONDONE;
        JionDeviceStatu = New_Node_Jion_Flag;
		SendClockFlag = 0; //���ͷ�ʱʱ��Ƭ
        HAL_TIM_Base_Start_IT(&htim2);
      }
    }
    
        /* ���ӻ��ַ�ʱ��Ƭ */
    if ((JionDeviceStatu == Node_Jion_Finish_Flag)&&(SendClockFlag == 0)
        &&(currentDeviceNumber != 0))
    {
		if (SendDataOkFlag == 1) {
			SendDataOkFlag = 0;
                        printf("main ����ʱ��ͬ��\n");
			//�������нڵ㿪ʼ�ϴ�����
			MasterSendClockData();
			SendClockFlag = 1;
			while(!SendDataOkFlag)  //�ȴ��������
			{
				Sx127xDataGet();
			}
			SendDataOkFlag = 1;
		}
    }
#endif
    
    
    if(EnableMaster == true)
    {
     MLCD_Show();

    }
    else
    {
      SLCD_Show();
    }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV32;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{     
  while((USART1->ISR & 0X40)==0);
  USART1->TDR = (uint8_t)ch;      
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
