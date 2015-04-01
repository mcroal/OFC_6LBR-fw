/*
 * at89rf231 device implement
 * change by mcroal.(mcroal@qq.com)
 * data:2013.11.26.
 * biref:This code is almost device independent and should be easy to port.
 * when you want to port, adds your SPI interface.
 */

#ifndef _PHY_ARCH_H_
#define _PHY_ARCH_H_

/*- Includes ---------------------------------------------------------------*/
#include "stdbool.h"
#include "stm32f10x_gpio.h"

/*- Definitions ------------------------------------------------------------*/
#define HAL_PhySpiDeselect()         GPIOA->ODR |=  0X0010; /* PA4 */
#define HAL_PhySpiSelect()           GPIOA->ODR &= ~0X0010;
#define HAL_PhySlpTrSet()            GPIOB->ODR |=  0X0001; /* PB0 */
#define HAL_PhySlpTrClear()          GPIOB->ODR &= ~0X0001;
#define HAL_PhyRstSet()              GPIOB->ODR |=  0X0002; /* PB1 */
#define HAL_PhyRstClear()            GPIOB->ODR &= ~0X0002;

#define HAL_BAT_LOW_MASK       ( 0x80 ) /**< Mask for the BAT_LOW interrupt. */
#define HAL_TRX_UR_MASK        ( 0x40 ) /**< Mask for the TRX_UR interrupt. */
#define HAL_TRX_END_MASK       ( 0x08 ) /**< Mask for the TRX_END interrupt. */
#define HAL_RX_START_MASK      ( 0x04 ) /**< Mask for the RX_START interrupt. */
#define HAL_PLL_UNLOCK_MASK    ( 0x02 ) /**< Mask for the PLL_UNLOCK interrupt. */
#define HAL_PLL_LOCK_MASK      ( 0x01 ) /**< Mask for the PLL_LOCK interrupt. */

#define HAL_MIN_FRAME_LENGTH   ( 0x03 ) /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   ( 0x7F ) /**< A frame should no more than 127 bytes. */

/* Number of receive buffers in RAM. */
#ifndef RF231_CONF_RX_BUFFERS
#define RF231_CONF_RX_BUFFERS 1
#endif
/*- Typdefs -------------------------------------------------------------*/
/** \struct rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct{
    uint8_t length;                       /**< Length of frame. */
    uint8_t data[ HAL_MAX_FRAME_LENGTH ]; /**< Actual frame data. */
    uint8_t lqi;                          /**< LQI value for received frame. */
    bool crc;                             /**< Flag - did CRC pass for received frame? */
} rx_frame_t;

void phy_arch_init(void);
void phy_INT_ENABLE();
void phy_INT_DISABLE();

void phyWriteRegister(uint8_t address, uint8_t value);
uint8_t phyReadRegister(uint8_t address);
void phyWriteSubregister(uint8_t address, uint8_t mask, uint8_t position, uint8_t value);
uint8_t phyReadSubregister(uint8_t address, uint8_t mask, uint8_t position);

void phy_frame_write(uint8_t *write_buffer, uint8_t length);
void phy_frame_read(rx_frame_t *rx_frame);
/*----------------------------------------------------------------------------*/
uint8_t HAL_PhySlpTrGet(void);
/*----------------------------------------------------------------------------*/
#endif // _PHY_ARCH_H_
