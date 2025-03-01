/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
  //Enable TIM2
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  // Disable timer for configuration
  timer->CR1 &= ~TIM_CR1_CEN;

  // Reset timer configuration
  timer->CR1 = 0; // Default control register
  timer->CNT = 0; // Reset counter
  timer->PSC = 0; // Prescaler to 0 initially

  // Enable timer interrupt in the NVIC
  NVIC_SetPriority(TIM2_IRQn, 2); // Set interrupt priority
  NVIC_EnableIRQ(TIM2_IRQn);

  //Clear TIM2_IRQn update interrupt,
  TIM2->SR &= ~TIM_SR_UIF;

  //Enable the hardware interrupt.
  TIM2->DIER |= TIM_DIER_UIE;

  //Enable the timer.
  TIM2->CR1 |= TIM_CR1_CEN;


}

void timer_reset(TIM_TypeDef* timer)
{
  timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  timer->CR1 &= ~TIM_CR1_CEN;
  // Calculate clock frequency and prescaler
  int clock_freq = 8000000; // Default clock frequency 4 MHz
  int prescaler = (clock_freq / 1000) - 1; // Scale to milliseconds
  // now is 4000 tick per sec

  //TIM2 counter limit: 2,147,483,647   32bit

  // Set prescaler and auto-reload value
  timer->PSC = prescaler;
  timer->ARR = period_ms - 1;

  timer->CNT = 0;

  // Update registers
  timer->EGR |= TIM_EGR_UG;

  //Enable the timer.
  TIM2->CR1 |= TIM_CR1_CEN;
}

// timer->CR1 |= TIM_CR1_CEN    //enable TIM2
