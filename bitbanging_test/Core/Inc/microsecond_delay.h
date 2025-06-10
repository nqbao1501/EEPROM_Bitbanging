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

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t ticks_per_us = (HAL_RCC_GetHCLKFreq() / 1000000U);
  uint32_t target_ticks = au32_microseconds * ticks_per_us;

  while ((DWT->CYCCNT - au32_initial_ticks) < target_ticks);
}

#endif /* INC_MICROSECOND_DELAY_H_ */
