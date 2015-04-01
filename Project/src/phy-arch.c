/*
 * at86rf231 device implement
 * change by mcroal.(mcroal@qq.com)
 * data:2013.11.26.
 * biref:This code is almost device independent and should be easy to port.
 * when you want to port, adds your SPI interface.
 */

/*- Includes ---------------------------------------------------------------*/
#include "contiki.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"

#include "bsp.h"
#include "phy-arch.h"
#include "at86rf231.h"

#define DEBUG 0
#if DEBUG
#include "bsp.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*- Implementations --------------------------------------------------------*/
/*************************************************************************//**
*****************************************************************************/
void phy_arch_init(void)
{
  BSP_SPI_Init();
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
    
  /* 设置 PB0 为AT86RF231的 SLP_TR 脚 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* 在P_ON状态，将 SLP_TR 电平拉低 */
  GPIO_ResetBits(GPIOB, GPIO_Pin_0);
  
  /* 设置 PB1 为AT86RF231的 /RST 脚 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* 在P_ON状态，将 /RST 电平拉高 */
  GPIO_SetBits(GPIOB, GPIO_Pin_1);
  
  /* 设置 PB.9 为AT86RF231的 RF_IRQ 脚 (EXTI Line4) */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /* 设置 GPIOB 作为外部中断9 */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
  EXTI_ClearITPendingBit(EXTI_Line4);

  /* 设置外部中断9上升沿触发 */
  EXTI_InitStructure.EXTI_Line    = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
  EXTI_Init(&EXTI_InitStructure);
  
  /* 设置外部中断4的 NVIC: Preemption Priority = 2 和 Sub Priority = 1 */
  NVIC_InitStructure.NVIC_IRQChannel                     = EXTI4_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 1; 
  NVIC_InitStructure.NVIC_IRQChannelCmd                  = ENABLE; 
  NVIC_Init(&NVIC_InitStructure);
}
void phy_INT_ENABLE()
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel                     = EXTI4_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //使能中断
  NVIC_Init(&NVIC_InitStructure);
}
/*************************************************************************//**
*****************************************************************************/
void phy_INT_DISABLE()
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel                     = EXTI4_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 1; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;         //失能中断
  NVIC_Init(&NVIC_InitStructure);
}
/*************************************************************************//**
*****************************************************************************/
uint8_t HAL_PhySlpTrGet(void)
{
  return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
}
/*************************************************************************//**
*****************************************************************************/
void 
phyWriteRegister(uint8_t address, uint8_t value)
{
  uint8_t spi_tx_buffer[2];
  uint8_t spi_rx_buffer[2];
  
  spi_tx_buffer[0] = (0xC0 | address);     // 转换地址为一个读寄存器地址
  spi_tx_buffer[1] = value;                // 发送一个操作命令 
  
  BSP_SPI_ReadWrite(spi_tx_buffer, sizeof(spi_tx_buffer), spi_rx_buffer);
}
/*************************************************************************//**
*****************************************************************************/
uint8_t 
phyReadRegister(uint8_t address)
{
  uint8_t spi_tx_buffer[2];
  uint8_t spi_rx_buffer[2];
  
  spi_tx_buffer[0] = (0x80 | address);         // 转换地址为一个读寄存器地址
  spi_tx_buffer[1] = 0x00;                     // 发送一个空操作命令 ，只是为了获得寄存器值
  BSP_SPI_ReadWrite(spi_tx_buffer, sizeof(spi_tx_buffer), spi_rx_buffer);
  return spi_rx_buffer[1];
}
/*************************************************************************//**
*****************************************************************************/
void
phyWriteSubregister(uint8_t address, uint8_t mask, uint8_t position,
                            uint8_t value)
{
    /* Read current register value and mask area outside the subregister. */
    volatile uint8_t register_value = phyReadRegister(address);
    register_value &= ~mask;

    /* Start preparing the new subregister value. shift in place and mask. */
    value <<= position;
    value &= mask;

    value |= register_value; /* Set the new subregister value. */

    /* Write the modified register value. */
    phyWriteRegister(address, value);
}
/*************************************************************************//**
*****************************************************************************/
uint8_t
phyReadSubregister(uint8_t address, uint8_t mask, uint8_t position)
{
  /* Read current register value and mask out subregister. */
  uint8_t register_value = phyReadRegister(address);
  register_value &= mask;
  register_value >>= position; /* Align subregister value. */
  
  return register_value;
}

/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
phy_frame_write(uint8_t *write_buffer, uint8_t length)
{
  /* Optionally truncate length to maximum frame length.
   * Not doing this is a fast way to know when the application needs fixing!
   */
  HAL_PhySpiSelect();  
  /* Send Frame Transmit (long mode) command and frame length */
  BSP_SPI_ReadWriteByte(0x60);
  /* write first octet(PHR) */
  BSP_SPI_ReadWriteByte(length);   
  
  /* Download to the Frame Buffer.
  * When the FCS is autogenerated there is no need to transfer the last two bytes
  * since they will be overwritten.
  */
  #if !RF231_CONF_CHECKSUM
    length -= 2;
  #endif
  do BSP_SPI_ReadWriteByte(*write_buffer++); while (--length);
  
  HAL_PhySpiDeselect();
}
/*----------------------------------------------------------------------------*/
/** \brief  Transfer a frame from the radio transceiver to a RAM buffer
 *
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *          If the frame length is out of the defined bounds, the length, lqi and crc
 *          are set to zero.
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 */
void
phy_frame_read(rx_frame_t *rx_frame)
{
  uint8_t *rx_data;
  
  /*Send frame read (long mode) command.*/
  HAL_PhySpiSelect();  
  BSP_SPI_ReadWriteByte(0x20);
  
  /*Read frame length. This includes the checksum. */
  uint8_t frame_length = BSP_SPI_ReadWriteByte(0);
  
  /*Check for correct frame length. Bypassing this test can result in a buffer overrun! */
  if ( 0 || ((frame_length >= HAL_MIN_FRAME_LENGTH) && 
             (frame_length <= HAL_MAX_FRAME_LENGTH))) {
  
    rx_data = (rx_frame->data);
    rx_frame->length = frame_length;
    
    /*Transfer frame buffer to RAM buffer */
    
    do{
      
      *rx_data++ = BSP_SPI_ReadWriteByte(0);    
    /* CRC was checked in hardware, but redoing the checksum here ensures the rx buffer
    * is not being overwritten by the next packet. Since that lengthy computation makes
    * such overwrites more likely, we skip it and hope for the best.
    * Without the check a full buffer is read in 320us at 2x spi clocking.
    * The 802.15.4 standard requires 640us after a greater than 18 byte frame.
    * With a low interrupt latency overwrites should never occur.
    */

    } while (--frame_length > 0);
       
    /*Read LQI value for this frame.*/
    rx_frame->lqi = BSP_SPI_ReadWriteByte(0);
            
    /* If crc was calculated set crc field in hal_rx_frame_t accordingly.
     * Else show the crc has passed the hardware check.
     */
    rx_frame->crc   = true;
  } else {
    /* Length test failed */
    rx_frame->length = 0;
    rx_frame->lqi    = 0;
    rx_frame->crc    = false;
  }
  
  HAL_PhySpiDeselect();
}
/*---------------------------------------------------------------------------*/
extern rx_frame_t rxframe[RF231_CONF_RX_BUFFERS];
extern uint8_t rxframe_head,rxframe_tail;
volatile extern signed char rf231_last_rssi;
/*---------------------------------------------------------------------------*/
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line4);
    /*开始中断处理*/   
    PRINTF("interrupt: \n");
    BSP_LED_Toggle(LED1);
    volatile uint8_t state;
    uint8_t interrupt_source; /* used after HAL_SPI_TRANSFER_OPEN/CLOSE block */ 
        
    interrupt_source = phyReadRegister(RG_IRQ_STATUS);
    PRINTF("interrupt_source: %02x\n",interrupt_source);
    /*Handle the incomming interrupt. Prioritized.*/

    if (0 == (interrupt_source & HAL_TRX_END_MASK ))
      return;
    state = phyReadSubregister(SR_TRX_STATUS);
    
    if((state == BUSY_RX_AACK) || (state == RX_ON) || \
       (state == BUSY_RX) || (state == RX_AACK_ON)){
        rf231_last_rssi = phyReadRegister(RG_PHY_ED_LEVEL);
        phy_frame_read(&rxframe[rxframe_tail]);
        rxframe_tail++;if (rxframe_tail >= RF231_CONF_RX_BUFFERS) rxframe_tail=0;
        rf231_interrupt();
    }
  }
}