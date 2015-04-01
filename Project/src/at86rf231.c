/*
 * at86rf231 device implement
 * change by mcroal.(mcroal@qq.com)
 * data:2013.11.26.
 * biref:This code is almost device independent and should be easy to port.
 * when you want to port, adds your SPI interface.
 */
#include "contiki.h"
#include "dev/radio.h"
#include "bsp.h"
#include "phy-arch.h"
#include "at86rf231.h"
#include "sys/clock.h"
#include "sys/rtimer.h"

#include "net/mac/frame802154.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/linkaddr.h"
#include "net/netstack.h"

#include <string.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* RF231 hardware delay times, from datasheet */
typedef enum{
    TIME_TO_ENTER_P_ON               = 330, /**<  Transition time from VCC is applied to P_ON - most favorable case! */
    TIME_P_ON_TO_TRX_OFF             = 330,   /**<  Transition time from P_ON to TRX_OFF. */
    TIME_SLEEP_TO_TRX_OFF            = 380, /**<  Transition time from SLEEP to TRX_OFF. */
    TIME_RESET                       = 6,   /**<  Time to hold the RST pin low during reset */
    TIME_ED_MEASUREMENT              = 140, /**<  Time it takes to do a ED measurement. */
    TIME_CCA                         = 140, /**<  Time it takes to do a CCA. */
    TIME_PLL_LOCK                    = 150, /**<  Maximum time it should take for the PLL to lock. */
    TIME_FTN_TUNING                  = 25,  /**<  Maximum time it should take to do the filter tuning. */
    TIME_NOCLK_TO_WAKE               = 6,   /**<  Transition time from *_NOCLK to being awake. */
    TIME_CMD_FORCE_TRX_OFF           = 1,   /**<  Time it takes to execute the FORCE_TRX_OFF command. */
    TIME_TRX_OFF_TO_PLL_ACTIVE       = 110, /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
    TIME_STATE_TRANSITION_PLL_ACTIVE = 1,   /**<  Transition time from all states to PLL active state. */
}radio_trx_timing_t;
/*---------------------------------------------------------------------------*/
rx_frame_t rxframe[RF231_CONF_RX_BUFFERS];
uint8_t rxframe_head,rxframe_tail;
uint8_t receive_on;
uint8_t rf231_last_correlation,rf231_last_rssi;
uint8_t volatile rf_pending;
/*---------------------------------------------------------------------------*/
PROCESS(at86rf231_process, "RF231 driver");
/*----------------------------------------------------------------------------*/
#define RF_CCA_CLEAR             1
#define RF_CCA_BUSY              0
/*---------------------------------------------------------------------------*/
static int on(void); /* prepare() needs our prototype */
static int off(void); /* transmit() needs our prototype */
static int channel_clear(void); /* transmit() needs our prototype */
/*---------------------------------------------------------------------------*/
int8_t
at86rf231_rf_channel_set(uint8_t channel)
{
  PRINTF("RF: Set Chan\n");

  phyWriteSubregister(SR_CHANNEL, channel);

  return (int8_t) channel;
}
/*---------------------------------------------------------------------------*/
uint8_t
at86rf231_rf_power_set(uint8_t new_power)
{
  PRINTF("RF: Set Power\n");
  if (new_power > TX_PWR_17_2DBM){
    new_power=TX_PWR_17_2DBM;
  }
  if (HAL_PhySlpTrGet()) {
    PRINTF("at86rf231_set_txpower:Sleeping");       //happens with cxmac
  } else {
    phyWriteSubregister(SR_TX_PWR, new_power);
  }
  return new_power;
}
/*---------------------------------------------------------------------------*/
void
at86rf231_rf_set_addr(uint16_t pan)
{
  
#if LINKADDR_SIZE==8 /* EXT_ADDR[7:0] is ignored when using short addresses */
  phyWriteRegister(RG_IEEE_ADDR_7, linkaddr_node_addr.u8[LINKADDR_SIZE - 8]);
  phyWriteRegister(RG_IEEE_ADDR_6, linkaddr_node_addr.u8[LINKADDR_SIZE - 7]);
  phyWriteRegister(RG_IEEE_ADDR_5, linkaddr_node_addr.u8[LINKADDR_SIZE - 6]);
  phyWriteRegister(RG_IEEE_ADDR_4, linkaddr_node_addr.u8[LINKADDR_SIZE - 5]);
  phyWriteRegister(RG_IEEE_ADDR_3, linkaddr_node_addr.u8[LINKADDR_SIZE - 4]);
  phyWriteRegister(RG_IEEE_ADDR_2, linkaddr_node_addr.u8[LINKADDR_SIZE - 3]);
  phyWriteRegister(RG_IEEE_ADDR_1, linkaddr_node_addr.u8[LINKADDR_SIZE - 2]);
  phyWriteRegister(RG_IEEE_ADDR_0, linkaddr_node_addr.u8[LINKADDR_SIZE - 1]);
#endif

  uint8_t abyte;
  abyte = pan & 0xFF;
  phyWriteRegister(RG_PAN_ID_0,abyte);
  abyte = (pan >> 8*1) & 0xFF;
  phyWriteRegister(RG_PAN_ID_1, abyte);

  phyWriteRegister(RG_SHORT_ADDR_0, linkaddr_node_addr.u8[LINKADDR_SIZE - 1]);
  phyWriteRegister(RG_SHORT_ADDR_1, linkaddr_node_addr.u8[LINKADDR_SIZE - 2]); 

}
/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */
static radio_status_t
radio_set_trx_state(uint8_t new_state)
{
  uint8_t original_state;

  /*Check function paramter and current state of the radio transceiver.*/
  if (!((new_state == TRX_OFF)    ||
        (new_state == RX_ON)      ||
        (new_state == PLL_ON)     ||
        (new_state == RX_AACK_ON) ||
        (new_state == TX_ARET_ON))){
    return RADIO_INVALID_ARGUMENT;
  }

  original_state = phyReadSubregister(SR_TRX_STATUS);

  if (new_state == original_state){
    return RADIO_SUCCESS;
  }

  /* At this point it is clear that the requested new_state is: */
  /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON. */

  /* The radio transceiver can be in one of the following states: */
  /* TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON. */
  if(new_state == TRX_OFF){
    phyWriteSubregister(SR_TRX_CMD, CMD_FORCE_TRX_OFF);     /* Go to TRX_OFF from any state. */
    clock_delay(TIME_CMD_FORCE_TRX_OFF); 
  } else {
    /* It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to */
    /* TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON. */
    if ((new_state == TX_ARET_ON) &&
        (original_state == RX_AACK_ON)){
      /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
      /* The final state transition to TX_ARET_ON is handled after the if-else if. */
      phyWriteSubregister(SR_TRX_CMD, PLL_ON);
      clock_delay(TIME_STATE_TRANSITION_PLL_ACTIVE);
    } else if ((new_state == RX_AACK_ON) &&
             (original_state == TX_ARET_ON)){
      /* First do intermediate state transition to PLL_ON, then to RX_AACK_ON. */
      /* The final state transition to RX_AACK_ON is handled after the if-else if. */
      phyWriteSubregister(SR_TRX_CMD, PLL_ON);
      clock_delay(TIME_STATE_TRANSITION_PLL_ACTIVE);
    } 
   
    /* Any other state transition can be done directly. */
    phyWriteSubregister(SR_TRX_CMD, new_state);
    
    /* When the PLL is active most states can be reached in 1us. However, from */
    /* TRX_OFF the PLL needs time to activate. */
    if (original_state == TRX_OFF){
      clock_delay(TIME_TRX_OFF_TO_PLL_ACTIVE);
    } else {
      clock_delay(TIME_STATE_TRANSITION_PLL_ACTIVE);
    }
  } /*  end: if(new_state == TRX_OFF) ... */
  
  /*Verify state transition.*/
  radio_status_t set_state_status = RADIO_TIMED_OUT;

  if (phyReadSubregister(SR_TRX_STATUS) == new_state){
    set_state_status = RADIO_SUCCESS;
  }
  return set_state_status;
}
/*---------------------------------------------------------------------------*/
/* Netstack API radio driver functions */
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  PRINTF("RF: Init\n");
  
  uint8_t i;
  /* Wait in case VCC just applied */
  clock_delay(TIME_TO_ENTER_P_ON);
  /* Initialize Hardware Abstraction Layer */
  phy_arch_init();

  /* Set receive buffers empty and point to the first */
  for (i=0;i<RF231_CONF_RX_BUFFERS;i++) rxframe[i].length=0;
  rxframe_head=0;rxframe_tail=0;
             
  /* Force transition to TRX_OFF */
  phyWriteSubregister(SR_TRX_CMD, CMD_TRX_OFF);
  clock_delay(TIME_P_ON_TO_TRX_OFF); 
  while (!(TRX_OFF == phyReadSubregister(SR_TRX_STATUS)));   

  phyWriteRegister(RG_IRQ_MASK, 0x00);
  phyReadRegister(RG_IRQ_STATUS); 
  phyWriteRegister(RG_IRQ_MASK, TRX_END_MASK);

  at86rf231_rf_power_set(TX_PWR_3DBM);
  at86rf231_rf_channel_set(RF_CHANNEL);

{  
  /* Set up number of automatic retries 0-15 (0 implies PLL_ON sends instead of the extended TX_ARET mode */
  phyWriteSubregister(SR_MAX_FRAME_RETRIES, RF231_CONF_AUTORETRIES ); 
  /* Set up carrier sense/clear channel assesment parameters for extended operating mode */
  phyWriteSubregister(SR_MAX_CSMA_RETRIES, 2 );  
  phyWriteRegister(RG_CSMA_BE, 0x80);            
  phyWriteRegister(RG_CSMA_SEED_0,phyReadRegister(RG_PHY_RSSI));
  /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
  phyWriteSubregister(SR_CCA_MODE,3);             
  phyWriteSubregister(SR_TX_AUTO_CRC_ON, 1);
}
  /* Start the packet receive process */
  process_start(&at86rf231_process, NULL);
  
  while(!(radio_set_trx_state(RX_AACK_ON) == RADIO_SUCCESS));
  receive_on = 1;
  return 1;
}
/*---------------------------------------------------------------------------*/
/*define send/recive buffer*/
static uint8_t buffer[RF231_MAX_TX_FRAME_LENGTH+CHECKSUM_LEN];
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  int ret = 0;
  uint8_t total_len,*pbuf;
 
  PRINTF("rf231_prepare: %d", (int)(payload_len + CHECKSUM_LEN));
  PRINTF("bytes\n");
  
  RIMESTATS_ADD(tx);
  
  /* Copy payload to RAM buffer */
  total_len = payload_len + CHECKSUM_LEN;
  if (total_len > RF231_MAX_TX_FRAME_LENGTH){
    PRINTF("rf231_prepare: packet too large (%d, max: %d)\n",total_len,RF231_MAX_TX_FRAME_LENGTH);
    ret = -1;
    goto bail;
  } 
  pbuf=&buffer[0];
  memcpy(pbuf,payload,payload_len);
  pbuf+=payload_len;
  
bail:
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short payload_len)
{

  if(channel_clear() == RF_CCA_BUSY) {
    RIMESTATS_ADD(contentiondrop);
    return RADIO_TX_COLLISION;
  }

  /* Prepare to transmit */
  radio_set_trx_state(TX_ARET_ON);
   
  uint8_t total_len = payload_len + CHECKSUM_LEN;
  
//  phy_INT_DISABLE();
  /* Toggle the SLP_TR pin to initiate the frame transmission */
  HAL_PhySlpTrSet();            //SLP拉高
  HAL_PhySlpTrClear();          //SLP拉低
  phy_frame_write(buffer, total_len);
//  phy_INT_ENABLE();
  
  PRINTF("rf231_transmit: %d\n", (int)total_len);
  
  uint8_t tx_result = phyReadSubregister(SR_TRAC_STATUS);

  if(receive_on) {
    receive_on = 1;
    radio_set_trx_state(RX_AACK_ON);
  }
  
  if (tx_result == SUCCESS_DATA_PENDING) {                   //success, data pending from addressee
    tx_result=RADIO_TX_OK;                                   //handle as ordinary success
  }
  if (tx_result == RADIO_TX_OK) {
    RIMESTATS_ADD(lltx);
    if(packetbuf_attr(PACKETBUF_ATTR_RELIABLE))
      RIMESTATS_ADD(ackrx);		                     //ack was requested and received
  } else if (tx_result == CHANNEL_ACCESS_FAILURE) {          //CSMA channel access failure
    RIMESTATS_ADD(contentiondrop);
    PRINTF("rf231_transmit: Transmission never started\n");
    tx_result = RADIO_TX_COLLISION;
  } else if (tx_result == NO_ACK) {                          //Expected ACK, none received
    tx_result = RADIO_TX_NOACK;
    PRINTF("rf231_transmit: ACK not received\n");
    RIMESTATS_ADD(badackrx);                                 //ack was requested but not received
  } else if (tx_result == INVALID) {                         //Invalid (Can't happen since waited for idle above?)
    tx_result = RADIO_TX_ERR;
  }
  
  return tx_result;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
  int ret = 0;
  
  ret = prepare(payload, payload_len);
  if(ret) {
    PRINTF("rf231_send: Unable to send, prep failed (%d)\n",ret);
    goto bail;
  }  
  ret = transmit(payload_len);
  
  bail:
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
read(void *buf, unsigned short bufsize)
{
  PRINTF("read\n");

  uint8_t len,*framep; 

  /* The length includes the twp-byte checksum but not the LQI byte */
  len = rxframe[rxframe_head].length;
  if (len==0) {
    return 0;
  }
  if (!receive_on) {
    return 0;
  } 
  
  /*revice packet too long*/
  if(len > RF231_MAX_TX_FRAME_LENGTH) {
    /* Oops, we must be out of sync. */
    rxframe[rxframe_head].length=0;
    RIMESTATS_ADD(badsynch);
    return 0;
  }
  /*revice packet too small*/
  if(len <= CHECKSUM_LEN) {
    rxframe[rxframe_head].length=0;
    RIMESTATS_ADD(tooshort);
    return 0;
  }
  /*revice packet is beyond size that we will read*/
  if(len - CHECKSUM_LEN > bufsize) {
    rxframe[rxframe_head].length=0;
    RIMESTATS_ADD(toolong);
    return 0;
  } 
  /* Transfer the frame, stripping the footer, but copying the checksum */
  framep=&(rxframe[rxframe_head].data[0]);
  memcpy(buf,framep,len);
  rf231_last_correlation = rxframe[rxframe_head].lqi;
  
  /* Clear the length field to allow buffering of the next packet */
  rxframe[rxframe_head].length=0;
  rxframe_head++;if (rxframe_head >= RF231_CONF_RX_BUFFERS) rxframe_head=0;
  /* If another packet has been buffered, schedule another receive poll */
  if (rxframe[rxframe_head].length) rf231_interrupt();   //执行速度太快会在在这里出错 
  
  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rf231_last_rssi + PHY_RSSI_BASE_VAL);
  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, rf231_last_correlation);

  RIMESTATS_ADD(rx);   

  /* Here return just the data length. The checksum is however still in the buffer for packet sniffing */
  return len - CHECKSUM_LEN;
}
/*---------------------------------------------------------------------------*/
static char
rf231_isidle(void)
{
  uint8_t radio_state;
  radio_state = phyReadSubregister(SR_TRX_STATUS);
  if (radio_state != BUSY_TX_ARET &&
      radio_state != BUSY_RX_AACK &&
      radio_state != STATE_TRANSITION &&
      radio_state != BUSY_RX && 
      radio_state != BUSY_TX) {
    return(1);
  } else {
    PRINTF(".%u",radio_state);
    return(0);
  } 
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  uint8_t cca=0;
  uint8_t radio_was_off = 0;

  /* Turn radio on if necessary. If radio is currently busy return busy channel */
  /* This may happen when testing radio duty cycling with RADIOALWAYSON */
  if(receive_on) {
    if (!rf231_isidle()) goto busyexit;
  } else {
    radio_was_off = 1;
    receive_on = 1;
    radio_set_trx_state(RX_AACK_ON);
  }
  
  /* Don't allow interrupts! */
  phy_INT_DISABLE();
  /* Start the CCA, wait till done, return result */
  /* Note reading the TRX_STATUS register clears both CCA_STATUS and CCA_DONE bits */
  phyWriteSubregister(SR_CCA_REQUEST,1);
  clock_delay(TIME_CCA);
  while((cca & 0x80) == 0 ) {
    cca=phyReadRegister(RG_TRX_STATUS);
  }  
  phy_INT_ENABLE();  

  if(radio_was_off) {
    phyWriteSubregister(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
    clock_delay(TIME_CMD_FORCE_TRX_OFF);
    receive_on = 0;
  }
  
  if(cca & 0x40) {
    return 1;
  } else {
  busyexit:
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  uint8_t radio_state;
  if (HAL_PhySlpTrGet()) {
    PRINTF("SLP_TR High\n");
  } else {  
    radio_state = phyReadSubregister(SR_TRX_STATUS);
    if ((radio_state==BUSY_RX) || (radio_state==BUSY_RX_AACK)) {
      PRINTF("BUSY_RX or BUSY_RX_AACK\n");
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  PRINTF("PENDING PACKET\n"); 
  return rf_pending;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  if(receive_on) {
    return 1;
  }
  
  receive_on = 1;
  radio_set_trx_state(RX_AACK_ON);
  
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  /* Don't do anything if we are already turned off. */
  if(receive_on == 0) {
    return 1;
  }
  /* If we are currently receiving a packet, we still call off(),
   * as that routine waits until Rx is complete (packet uploaded in ISR
   * so no worries about losing it). If using RX_AACK_MODE, chances are
   * the packet is not for us and will be discarded. */
  if (!rf231_isidle()) {
    PRINTF("rf231_off: busy receiving\r\n");
    return 0;
  }
  phyWriteSubregister(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
  clock_delay(TIME_CMD_FORCE_TRX_OFF);
  receive_on = 0;

  return 1;
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
int
rf231_interrupt(void)
{
  /* Poll the receive process, unless the stack thinks the radio is off */
  process_poll(&at86rf231_process);
  rf_pending = 1;  
  RIMESTATS_ADD(llrx);
  return 1;
}
/*---------------------------------------------------------------------------*/
/* Process to handle input packets
 * Receive interrupts cause this process to be polled
 * It calls the core MAC layer which calls rf230_read to get the packet
 */
PROCESS_THREAD(at86rf231_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    rf_pending = 0;
    packetbuf_clear();
    /* Turn off interrupts to avoid ISR writing to the same buffers we are reading. */
    phy_INT_DISABLE();   
    
    len = read(packetbuf_dataptr(), PACKETBUF_SIZE);        
    
    /* Restore interrupts. */
    phy_INT_ENABLE();
    
    PRINTF("rf231_read: %u bytes lqi %u\n",len,rf231_last_correlation);
    if(len > 0) {
      packetbuf_set_datalen(len);
      NETSTACK_RDC.input();
    }
  }  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
const struct radio_driver at86rf231_radio_driver = {
  init,
  prepare,
  transmit,
  send,
  read,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
};
/*---------------------------------------------------------------------------*/