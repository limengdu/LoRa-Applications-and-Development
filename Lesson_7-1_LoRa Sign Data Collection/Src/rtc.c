/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */
#include "netprocess.h"
#include "string.h"
#include "dataprocess.h"

#define CLOCKHOURS 5

volatile uint8_t SendClockFlag = 0;

volatile uint8_t sendUpDataFlag = 0;

RTC_AlarmTypeDef gAlarm;

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 125-1;
  hrtc.Init.SynchPrediv = 2000-1;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x1;
  sDate.Year = 0x18;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }
    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt Deinit */
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

//**********************************//
//
//�������ƣ�HAL_RTC_AlarmAEventCallback   
//
//���������� �����¼��ص�����  
//
//����������   RTC_HandleTypeDef *hrtc
//
//����ֵ��    �� 
//
//*******************************//

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  
  RTC_TimeTypeDef masterTime;
  RTC_TimeTypeDef SlaveTime;
  RTC_DateTypeDef masterDate;
  
  
  
#if MASTER  
  //��λͬ��ʱ�ӱ�־
    SendClockFlag = 0;
  //��ȡ�´�����ʱ��
    HAL_RTC_GetTime(hrtc, &masterTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, &masterDate, RTC_FORMAT_BIN);
    gAlarm.AlarmTime.Hours = masterTime.Hours + CLOCKHOURS;
    gAlarm.AlarmTime.Minutes = masterTime.Minutes;
    gAlarm.AlarmTime.Seconds = masterTime.Seconds;
    gAlarm.AlarmTime.SubSeconds = masterTime.SubSeconds;
    
#else
    sendUpDataFlag = 1;
    HAL_RTC_GetTime(hrtc, &SlaveTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, &masterDate, RTC_FORMAT_BIN);
    gAlarm.AlarmTime.Hours = SlaveTime.Hours + DataUpTimeHours;
    gAlarm.AlarmTime.Minutes = SlaveTime.Minutes + DataUpTimeMinute;
    gAlarm.AlarmTime.Seconds = SlaveTime.Seconds + DataUpTimeSeconds;
    gAlarm.AlarmTime.SubSeconds = SlaveTime.SubSeconds + DataUpTimeSubSeconds;
#endif
    
    
    if (gAlarm.AlarmTime.Seconds > 59)
    {
	   gAlarm.AlarmTime.Seconds -= 60;
	   gAlarm.AlarmTime.Minutes += 1;
    }

   if ( gAlarm.AlarmTime.Minutes >59)
   {
	   gAlarm.AlarmTime.Minutes -= 60;
	   gAlarm.AlarmTime.Hours += 1;
   }
   if (gAlarm.AlarmTime.Hours > 23)
   {
	   gAlarm.AlarmTime.Hours -= 24;
   }
    
   printf("RTC\n");
  //ʹ�������ж�
    if (HAL_RTC_SetAlarm_IT(hrtc, &gAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
    
  
  
}


//**********************************//
//
//�������ƣ�   GetTimeHMS
//
//����������   ʱ����ת��
//
//����������   uint32_t timeData,uint8_t *hours,uint8_t *minute,uint8_t *seconds,uint32_t *subSeconds
//
//����ֵ��     ��
//
//*******************************//

void GetTimeHMS(uint32_t timeData,uint8_t *hours,uint8_t *minute,uint8_t *seconds,uint32_t *subSeconds) 
{
	/* ������� */
	*subSeconds = timeData % 1000;
	/* �������*/
	timeData = timeData / 1000;
	*seconds = timeData % 60;
	/* ��÷���*/
	timeData = timeData / 60;
	*minute = timeData % 60;
	/* ���Сʱ */
	*hours = timeData / 60;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
