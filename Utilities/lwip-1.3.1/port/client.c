/**
  ******************************************************************************
  * @file    client.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   A sample UDP/TCP client
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct tcp_pcb *TcpPCB;
const static uint8_t TCP_TestData[]="This is LwIP TCP Client STM32F107 Cortex-M3…œµƒ≤‚ ‘£°\r\n";
/* Private function prototypes -----------------------------------------------*/
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
void tcp_client_err(void *arg, err_t err);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the client application.
  * @param  None
  * @retval None
  */
void client_init(void)
{
  struct tcp_pcb *pcb;
  struct ip_addr iptab;
      
  IP4_ADDR(&iptab, 172, 23, 11, 66);

  /* Create a new TCP control block  */
  pcb = tcp_new();

  /* Assign to the new pcb a local IP address and a port number */
  tcp_bind(pcb, IP_ADDR_ANY, 10011);

  /* Connect to the server: send the SYN */
  tcp_connect(pcb, &iptab, 10012, tcp_client_connected);
  
}

/**
  * @brief  This function is called when the connection with the remote 
  *         server is established
  * @param arg user supplied argument
  * @param tpcb the tcp_pcb which received data
  * @param err error value returned by the tcp_connect 
  * @retval error value
  */
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{

  TcpPCB = tpcb;
  
  return ERR_OK;
}
/**
  * @brief  Send to the server the data.
  * @param  arg user supplied argument
  * @retval None
  */
void tcp_client_sent(void)
{
  
  tcp_write(TcpPCB, TCP_TestData, sizeof(TCP_TestData), 1);

  /* send the data right now */
  tcp_output(TcpPCB);
  
}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
