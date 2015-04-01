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

/* LED ��� ------------------------------------------*/
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

/* SPI ��� ------------------------------------------*/
void BSP_SPI_Init() 
{
   
  SPI_InitTypeDef  SPI_InitStructure;

  /* ʹ�� SPI1 �� GPIOB ��ʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
   
  /* ���� SPI �������: PA4 - NSS ,PA5 - SCK , PA6 - MISO , PA7 - MOSI */
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
  /* ��P_ON״̬���� NSS ���� */
  GPIOA->ODR |= 0X0010;     //PA4 

  /* ���� SPI1 ���� */
  SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex; //����Ϊ˫��˫��ȫ˫��
  SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;                 //����Ϊ��SPI
  SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;                 //���ͽ���8λ֡��ʽ
  SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;                    //ʱ�����յ� 
  SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;                  //���ݲ����ڵ�1��ʱ����
  SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;                    //�������NSS�ź�
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;        //������Ԥ��ƵֵΪ16 
  SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;                //���ݴ����MSBλ��ʼ
  SPI_InitStructure.SPI_CRCPolynomial     = 7;                               //����CRCֵ����Ķ���ʽ7
  SPI_Init(SPI1, &SPI_InitStructure);

  /* ʹ�� SPI1 */
  SPI_Cmd(SPI1, ENABLE);
}

/* SPI ��дһ���ֽ� */
/* ����ֵ:��ȡ�����ֽ� */
u8 BSP_SPI_ReadWriteByte(u8 TxData)
{
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, TxData);
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
  return SPI_I2S_ReceiveData(SPI1);
}

void BSP_SPI_ReadWrite(uint8_t* TxData, uint8_t lenTxData, uint8_t* RxData)
{		
  /* ���� CS ���ţ���֪���豸��������״̬ */
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  /* ���������ֽ� */
  while (lenTxData > 0){
    /* дһ���ֽڵ� TX ������ */
    SPI_I2S_SendData(SPI1, *TxData);
  
    /* æ״̬���ȴ��жϱ�־λ */
    while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);  //���ջ�����Ϊ��   
    
    /* ����жϱ�־λ */
    SPI_I2S_ClearFlag(SPI1,SPI_I2S_FLAG_RXNE);
    /* ����ոմ� RX ���������յ������� */
    *RxData = SPI_I2S_ReceiveData(SPI1); 
    RxData++;
    TxData++;
    /* ���ͺ�����һ���ֽ� */
    lenTxData--;
  }

  /* ���� CS ���ţ���֪���豸֪�����ͽ��� */
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

/* UART ��� ------------------------------------------*/
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