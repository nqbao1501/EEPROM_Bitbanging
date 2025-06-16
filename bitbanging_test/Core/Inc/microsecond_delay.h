/*
 * microsecond_delay.h
 *
 *
 *      Author: Admin
 */

#ifndef INC_MICROSECOND_DELAY_H_
#define INC_MICROSECOND_DELAY_H_
#include "stm32f4xx_hal.h"

uint32_t DWT_Delay_Init(void);

__STATIC_INLINE void DWT_Delay_us(uint32_t microseconds)
{
  uint32_t initial_ticks = DWT->CYCCNT;
  uint32_t ticks_per_us = (SystemCoreClock / 1000000U);
  uint32_t target_ticks = microseconds * ticks_per_us;

  while ((DWT->CYCCNT - initial_ticks) < target_ticks);
}

__STATIC_INLINE void DWT_Delay_cycles(uint32_t cycles)
{
  uint32_t initial_ticks = DWT->CYCCNT;
  while ((DWT->CYCCNT - initial_ticks) < cycles);
}

#endif /* INC_MICROSECOND_DELAY_H_ */
