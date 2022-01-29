#ifndef _FAN_H
#define _FAN_H


#define FAN_PIN                         GPIO_PIN_7
#define FAN_GPIO_PORT                   GPIOB

#define FAN_ON                           GPIO_PIN_RESET
#define FAN_OFF                          GPIO_PIN_SET

void FanOn(void);

void FanOff(void);

uint8_t FanReadStaus(void);

#endif