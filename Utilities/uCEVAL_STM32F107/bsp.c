/**
\brief openmoteSTM32 definition of the "spi" bsp module.

\author Ryomamcroal <mcroal@qq.com>,  2013.11.
*/
#include <stdio.h>
#include "bsp.h"

/*---------------------------------------------------------------------------*/
static const char hexconv[] = "0123456789abcdef";
static const char binconv[] = "01";
/*---------------------------------------------------------------------------*/

/* LED 相关 ------------------------------------------*/
void  BSP_LED_Init (void)
{
    GPIO_InitTypeDef  gpio_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    gpio_init.GPIO_Pin   = BSP_GPIOD_LEDS;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode  = GPIO_Mode_Out_PP;

    GPIO_Init(GPIOD, &gpio_init);
}

void  BSP_LED_On (Led_TypeDef led)
{
    switch (led) {
        case 0:
             GPIO_SetBits(GPIOD, BSP_GPIOD_LEDS);
             break;

        case 1:
             GPIO_SetBits(GPIOD, BSP_GPIOD_LED1);
             break;

        case 2:
             GPIO_SetBits(GPIOD, BSP_GPIOD_LED2);
             break;

        case 3:
             GPIO_SetBits(GPIOD, BSP_GPIOD_LED3);
             break;

        default:
             break;
    }
}

void  BSP_LED_Off (Led_TypeDef led)
{
    switch (led) {
        case 0:
             GPIO_ResetBits(GPIOD, BSP_GPIOD_LEDS);
             break;

        case 1:
             GPIO_ResetBits(GPIOD, BSP_GPIOD_LED1);
             break;

        case 2:
             GPIO_ResetBits(GPIOD, BSP_GPIOD_LED2);
             break;

        case 3:
             GPIO_ResetBits(GPIOD, BSP_GPIOD_LED3);
             break;

        default:
             break;
    }
}

void  BSP_LED_Toggle (Led_TypeDef led)
{
    unsigned  int  pins;

    switch (led) {
        case 0:
             pins =  GPIO_ReadOutputData(GPIOD);
             pins ^= BSP_GPIOD_LEDS;
             GPIO_SetBits(  GPIOD,   pins  & BSP_GPIOD_LEDS);
             GPIO_ResetBits(GPIOD, (~pins) & BSP_GPIOD_LEDS);
             break;

        case 1:
        case 2:
        case 3:
            pins = GPIO_ReadOutputData(GPIOD);
            if ((pins & (1 << (led + BSP_LED_START_BIT))) == 0) {
                 GPIO_SetBits(  GPIOD, (1 << (led + BSP_LED_START_BIT)));
             } else {
                 GPIO_ResetBits(GPIOD, (1 << (led + BSP_LED_START_BIT)));
             }
            break;

        default:
             break;
    }
}

/* SPI 相关 ------------------------------------------*/
void BSP_SPI_Init() 
{
   
  SPI_InitTypeDef  SPI_InitStructure;

  /* 使能 SPI1 和 GPIOB 的时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
   
  /* 设置 SPI 相关引脚: PA4 - NSS ,PA5 - SCK , PA6 - MISO , PA7 - MOSI */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* 在P_ON状态，将 NSS 拉高 */
  GPIOA->ODR |= 0X0010;     //PA4 

  /* 设置 SPI1 参数 */
  SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; //设置为双线双向全双工
  SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;                 //设置为主SPI
  SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;                 //发送接收8位帧格式
  SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;                    //时钟悬空低 
  SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;                  //数据捕获于第1个时钟沿
  SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;                    //软件控制NSS信号
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;        //比特率预分频值为16 
  SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;                //数据传输从MSB位开始
  SPI_InitStructure.SPI_CRCPolynomial     = 7;                               //定义CRC值计算的多项式7
  SPI_Init(SPI1, &SPI_InitStructure);

  /* 使能 SPI1 */
  SPI_Cmd(SPI1, ENABLE);
}

/* SPI 读写一个字节 */
/* 返回值:读取到的字节 */
u8 BSP_SPI_ReadWriteByte(u8 TxData)
{
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, TxData);
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}

void BSP_SPI_ReadWrite(uint8_t* TxData, uint8_t lenTxData, uint8_t* RxData)
{		
  /* 拉低 CS 引脚，告知从设备进入侦听状态 */
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  /* 发送所有字节 */
  while (lenTxData > 0){
    /* 写一个字节到 TX 缓冲区 */
    SPI_I2S_SendData(SPI1, *TxData);
  
    /* 忙状态，等待中断标志位 */
    while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);  //接收缓冲区为空   
    
    /* 清楚中断标志位 */
    SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
    /* 保存刚刚从 RX 缓冲区接收到的数据 */
    *RxData = SPI_I2S_ReceiveData(SPI1); 
    RxData++;
    TxData++;
    /* 发送后少了一个字节 */
    lenTxData--;
  }

  /* 拉高 CS 引脚，告知从设备知道发送结束 */
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

/* UART 相关 ------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void BSP_USART_Init(unsigned int bpr)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
  
  /* Configure USARTx_Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure USARTx_Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  
  USART_InitStructure.USART_BaudRate = bpr;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART2, &USART_InitStructure);
  
  /* Enable the USARTz Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable USARTy Receive and Transmit interrupts */
//  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  
   /* Enable the USARTy */
  USART_Cmd(USART2, ENABLE);
  
}

void BSP_USART_PUTC(unsigned char x)
{
  USART_SendData(USART2, x);
    /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}


PUTCHAR_PROTOTYPE
{
  BSP_USART_PUTC(ch);
  return ch;
}

void putstring(char *s)
{
  while(*s) {
    BSP_USART_PUTC(*s++);
  }
}

void puthex(uint8_t c)
{
  BSP_USART_PUTC(hexconv[c >> 4]);
  BSP_USART_PUTC(hexconv[c & 0x0f]);
}

void putbin(uint8_t c)
{
  unsigned char i = 0x80;
  while(i) {
    BSP_USART_PUTC(binconv[(c & i) != 0]);
    i >>=1;
  }
}