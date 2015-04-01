#include "stm32f10x.h"
#include "stm32f10x_it.h"

#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>
/*---------------------------------------------------------------------------*/
static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_SECOND;
/*---------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
  current_clock++;
  if(etimer_pending() && etimer_next_expiration_time() <= current_clock) {
    etimer_request_poll();
  }
  if (--second_countdown == 0) {
    current_seconds++;
    second_countdown = CLOCK_SECOND;
  }
}
/*---------------------------------------------------------------------------*/
void
clock_init(void)
{  
//已经在stm32f107.c设置了SysTick的速度 
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  return current_seconds;
}
/*---------------------------------------------------------------------------*/
void 
clock_set_seconds(unsigned long sec)
{
  current_seconds = sec;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return current_clock;
}
/*---------------------------------------------------------------------------*/
/**
 * busy-wait the CPU for a duration depending on CPU speed.
 * (1/32)*32us = 1us
 */
void
clock_delay(unsigned int i)
{
  for(; i > 0; i--) {
    unsigned int j;
    for(j = 32; j > 0; j--) {
      asm ("nop");
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of clock ticks (7.8 ms at 128 Hz).
 */
void
clock_wait(clock_time_t i)
{
  clock_time_t start;
  start = clock_time();
  while(clock_time() - start < (clock_time_t)i);
}
/*---------------------------------------------------------------------------*/

