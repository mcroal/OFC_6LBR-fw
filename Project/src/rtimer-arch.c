/**
\brief openmoteSTM32 definition of the TIMER.

\author Ryomamcroal <mcroal@qq.com>,  2013.11.
*/

/*- Includes ---------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "rtimer-arch.h"

/*- Definitions ------------------------------------------------------------*/
#define TIMER_PRESCALER     (RTIMER_SECOND-1)

/*- Variables --------------------------------------------------------------*/
volatile uint32_t rtimer_clock = 0uL;
/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
void TIM2_IRQHandler(void)
{  
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    /* clear interrupt pending flag */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    rtimer_clock++;
  
  /* check for, and run, any pending rtimers */
  // XXX no, later, when we have implemented rtimer timer expiration properly
  //rtimer_run_next();    
  }
}
/*---------------------------------------------------------------------------*/
void rtimer_arch_init(void)
{
  /* 设置定时器2的时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

  /* 设置定时器2 */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
  TIM_TimeBaseStructure.TIM_Period        = 1;
  TIM_TimeBaseStructure.TIM_Prescaler     = TIMER_PRESCALER;     
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* 设置定时器2的输出比较模式 */
//  TIM_OCInitTypeDef TIM_OCInitStructure;
//  TIM_OCStructInit(&TIM_OCInitStructure);
//  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_Timing;       //选择定时器模式为输出比较触发模式
//  TIM_OCInitStructure.TIM_Pulse       = 0;                       //设置待装入捕获比较寄存器的脉冲值
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //选择使能输出比较状态
//  TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;     //选择输出比较极性为高
//  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
 
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);                //清除标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);           //开定时器中断

  /* 使能 TIM2 */
  TIM_Cmd(TIM2, ENABLE); 
  
  //Configure NVIC: Preemption Priority = 0 and Sub Priority = 0
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  /*目前没有实现*/
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t rtimer_arch_now(void)
{
#if 0 
  rtimer_clock_t t1, t2;
  do {
    t1 = rtimer_clock;
    t2 = rtimer_clock;
  }while(t1 != t2);
#endif 
  return rtimer_clock;
}