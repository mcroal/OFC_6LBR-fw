/**
  ******************************************************************************
  * @file    main.c
  * @author  mcroal
  * @version V1.0.0
  * @date    11/20/2013
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include <sys/process.h>
#include <etimer.h>
#include <string.h>

#include "net/mac/frame802154.h"
#include "at86rf231.h"
#include "bsp.h"
#include "dev/watchdog.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
  #include <stdio.h>
  #define PRINTF(...) printf(__VA_ARGS__)
#else
  #define PRINTF(...)
#endif

#define XMACADDR "\x00\x12\x4b\x00\x01\x0e\x51\xba"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned int idle_count = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void set_rime_addr(void *addr)
{
  memcpy(&uip_lladdr.addr, addr, sizeof(uip_lladdr.addr));
  
#if UIP_CONF_IPV6
  linkaddr_set_node_addr((linkaddr_t *)&uip_lladdr.addr);
#else
  linkaddr_set_node_addr((linkaddr_t *)&uip_lladdr.addr[8-LINKADDR_SIZE]);
#endif
  {
    int i;
    PRINTF("Link set with address ");
    for(i = 0; i < sizeof(linkaddr_t) - 1; i++) {
      printf(" %02x", linkaddr_node_addr.u8[i]);
    }
    PRINTF(" %02x\n", linkaddr_node_addr.u8[i]);
  } 
  
  at86rf231_rf_set_addr(IEEE802154_PANID);
}
/*---------------------------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  
  /* Setup STM32 system (clocks, Ethernet, GPIO, NVIC) and STM3210C-EVAL resources */
  System_Setup();
    
  rtimer_init();
  process_init();
  
  /* start service */
  process_start(&etimer_process, NULL); 
  ctimer_init();
  
  /* initialize the netstack */
  netstack_init();
  set_rime_addr(XMACADDR);

#if UIP_CONF_IPV6
  memcpy(&uip_lladdr.addr, &linkaddr_node_addr, sizeof(uip_lladdr.addr));
  queuebuf_init();
#endif /* UIP_CONF_IPV6 */

#if AUTOSTART_ENABLE    
  autostart_start(autostart_processes);
#endif
    
  while(1) 
  {
    uint8_t r;
    do { 
      /* Reset watchdog and handle polls and events */
//      watchdog_periodic();      
      r = process_run();
    } while(r > 0);
    idle_count++;
    /* Idle! */
    /* Stop processor clock */
    /* asm("wfi"::); */ 
  }
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
