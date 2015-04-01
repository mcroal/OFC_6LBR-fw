/**
  ******************************************************************************
  * @file    stm32_eth-driver.c
  * @author  mcroal
  * @version V1.0.0
  * @date    11/20/2013
  * @brief   eth net program body
  ******************************************************************************
  */

#include "contiki.h"
#include "contiki-net.h"

#include "main.h"
#include "stm32_eth.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"
#include "packet-filter.h"
#include "sicslow-ethernet.h"
#include "eth-drv.h"

#include <string.h>
#include <stdio.h>

#if UIP_CONF_LLH_LEN == 0
uint8_t ll_header[ETHERNET_LLH_LEN];
#endif
extern void eth_input(void);

#define ETH_RXBUFNB           4
#define ETH_TXBUFNB           2
#define ETH_DMARxDesc_FrameLengthShift          16
#define ETH_ERROR             ((u32)0)
#define ETH_SUCCESS           ((u32)1)
#define ETH_CONF_BUFSIZE      1

typedef struct{
u32 length;
u32 buffer;
ETH_DMADESCTypeDef *descriptor;
}FrameTypeDef;

ETH_DMADESCTypeDef  DMARxDscrTab[ETH_RXBUFNB], DMATxDscrTab[ETH_TXBUFNB];/* Ethernet Rx & Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE], Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE];/* Ethernet buffers */
ETH_DMADESCTypeDef  *DMATxDesc = DMATxDscrTab;
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

FrameTypeDef ETH_RxPkt_ChainMode(void);       
u32 ETH_GetCurrentTxBuffer(void);
u32 ETH_TxPkt_ChainMode(u16 FrameLength);

/*---------------------------------------------------------------------------*/
PROCESS(eth_drv_process, "STM32107-ETH driver");
/*---------------------------------------------------------------------------*/
static void
eth_drv_init(void)
{

  ETH_MACAddressConfig(ETH_MAC_Address0, (uint8_t *)&eth_mac_addr);  
  
  /* Initialize Tx Descriptors list: Chain Mode */
  ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
  /* Initialize Rx Descriptors list: Chain Mode  */
  ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

  /* Enable Ethernet Rx interrrupt */
  { int i;
    for(i=0; i<ETH_RXBUFNB; i++)
    {
      ETH_DMARxDescReceiveITConfig(&DMARxDscrTab[i], ENABLE);
    }
  }

#ifdef CHECKSUM_BY_HARDWARE
  /* Enable the checksum insertion for the Tx frames */
  { int i;
    for(i=0; i<ETH_TXBUFNB; i++)
    {
      ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
    }
  }
#endif

  /* Enable MAC and DMA transmission and reception */
  ETH_Start();
}
/*---------------------------------------------------------------------------*/
void
eth_drv_send(void)
{
  u8 *buffer =  (u8 *)ETH_GetCurrentTxBuffer();
  
  memcpy((u8_t*)&buffer[0], ll_header, ETHERNET_LLH_LEN);
  memcpy((u8_t*)&buffer[ETHERNET_LLH_LEN], uip_buf, uip_len);

  ETH_TxPkt_ChainMode(uip_len + sizeof(struct uip_eth_hdr));
}
/*---------------------------------------------------------------------------*/
void
eth_drv_exit(void)
{
  /* @mcroal:Noting, for linux driver */
  
}
/*---------------------------------------------------------------------------*/
void 
eth_drv_input(void)
{  
  FrameTypeDef frame;
  u8 *p;
  
  frame = ETH_RxPkt_ChainMode();
  /* Obtain the size of the packet and put it into the "len"
     variable. */
  uip_len = frame.length - ETHERNET_LLH_LEN;
  p = (u8 *)frame.buffer;
  
  memcpy(ll_header, (uint8_t *)&p[0], ETHERNET_LLH_LEN);
  memcpy(uip_buf, (uint8_t *)&p[ETHERNET_LLH_LEN], uip_len);

  /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
  frame.descriptor->Status = ETH_DMARxDesc_OWN; 
  
  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
  {
    /* Clear RBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    ETH->DMARPDR = 0;
  }
  if(uip_len > 0) { 
    process_poll(&eth_drv_process);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(eth_drv_process, ev, data)
{

  PROCESS_BEGIN();

  eth_drv_init();
  ethernet_ready = 1;
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, DISABLE);
    eth_input();
    ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R, ENABLE);
  }  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
FrameTypeDef ETH_RxPkt_ChainMode(void)
{ 
  u32 framelength = 0;
  FrameTypeDef frame = {0,0}; 

  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
  {	
    frame.length = ETH_ERROR;

    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)  
    {
      /* Clear RBUS ETHERNET DMA flag */
      ETH->DMASR = ETH_DMASR_RBUS;
      /* Resume DMA reception */
      ETH->DMARPDR = 0;
    }

    /* Return error: OWN bit set */
    return frame; 
  }
  
  if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) && 
     ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&  
     ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))  
  {      
    /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
    framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;
	
	/* Get the addrees of the actual buffer */
	frame.buffer = DMARxDescToGet->Buffer1Addr;	
  }
  else
  {
    /* Return ERROR */
    framelength = ETH_ERROR;
  }

  frame.length = framelength;


  frame.descriptor = DMARxDescToGet;
  
  /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */      
  /* Chained Mode */    
  /* Selects the next DMA Rx descriptor list for next buffer to read */ 
  DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);    
  
  /* Return Frame */
  return (frame);  
}
/*---------------------------------------------------------------------------*/
u32 ETH_TxPkt_ChainMode(u16 FrameLength)
{   
  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
  {  
	/* Return ERROR: OWN bit set */
    return ETH_ERROR;
  }
        
  /* Setting the Frame Length: bits[12:0] */
  DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);

  /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */    
  DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

  /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
  DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

  /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
  if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
  {
    /* Clear TBUS ETHERNET DMA flag */
    ETH->DMASR = ETH_DMASR_TBUS;
    /* Resume DMA transmission*/
    ETH->DMATPDR = 0;
  }
  
  /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */  
  /* Chained Mode */
  /* Selects the next DMA Tx descriptor list for next buffer to send */ 
  DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);    


  /* Return SUCCESS */
  return ETH_SUCCESS;   
}
/*---------------------------------------------------------------------------*/
u32 ETH_GetCurrentTxBuffer(void)
{
  /*Return Buffer address */
  return (DMATxDescToSet->Buffer1Addr);
}
/*---------------------------------------------------------------------------*/
void ETH_IRQHandler(void)
{
  /* Handles all the received frames */
  while(ETH_GetRxPktSize() != 0) 
  {	        
    eth_drv_input();
  }

  /* Clear the Eth DMA Rx IT pending bits */
  ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
  ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}
/*---------------------------------------------------------------------------*/
