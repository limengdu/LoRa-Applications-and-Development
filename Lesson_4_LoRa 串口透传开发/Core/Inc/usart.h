/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define RXLENGHT  128
#define RECEIVELEN 2048
#define USART_DMA_SENDING       1//发送未完成
#define USART_DMA_SENDOVER      0//发送完成
 
typedef struct
{
  uint8_t receive_flag ;//空闲接收标记
  uint8_t dmaSend_flag ;//发送完成标记
  uint16_t Usart_rx_len;//接收长度
  uint8_t usartDMA_rxBuf[RECEIVELEN];//无线发送缓存
}USART_RECEIVETYPE;
 
extern USART_RECEIVETYPE UsartType1;
extern USART_RECEIVETYPE UsartType2;
//局部变量
//extern uint8_t g_Usart1_RxBuffer[RXLENGHT]; 
//extern uint8_t g_Usart2_RxBuffer[RXLENGHT]; 
//全局变量
extern uint8_t a_Usart1_RxBuffer[RXLENGHT]; //DMA接收缓存
//extern uint8_t a_Usart2_RxBuffer[RXLENGHT];
/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
