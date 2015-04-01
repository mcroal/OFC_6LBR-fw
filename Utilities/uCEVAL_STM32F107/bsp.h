#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"

typedef enum 
{
  LEDALL = 0,
  LED1   = 1,
  LED2   = 2,
  LED3   = 3
} Led_TypeDef;

#define  BSP_LED_START_BIT  (13 - 1) 

#define  BSP_GPIOD_LED1                          GPIO_Pin_13
#define  BSP_GPIOD_LED2                          GPIO_Pin_14
#define  BSP_GPIOD_LED3                          GPIO_Pin_15

#define  BSP_GPIOD_LEDS                         (BSP_GPIOD_LED1 | \
                                                 BSP_GPIOD_LED2 | \
                                                 BSP_GPIOD_LED3)

/*****************************************************************************/
void  BSP_LED_Init (void);
void  BSP_LED_On (Led_TypeDef led);
void  BSP_LED_Off (Led_TypeDef led);
void  BSP_LED_Toggle (Led_TypeDef led);

void BSP_SPI_Init();
unsigned char BSP_SPI_ReadWriteByte(unsigned char TxData);
void BSP_SPI_ReadWrite(uint8_t* TxData, uint8_t lenTxData, uint8_t* RxData);

void BSP_USART_Init(unsigned int bpr);
void BSP_USART_PUTC(unsigned char x);
void putstring(char *s);
void puthex(uint8_t c);
void putbin(uint8_t c);

#endif