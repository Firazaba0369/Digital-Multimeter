/*
 * timer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: firaz
 */
#include "main.h"
#include "timer.h"

void TIM2_init(void){
	//configure TIM2 clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	//set TIM2 to count up
	TIM2->CR1 &= ~(TIM_CR1_DIR);
	//set ARR clock
	TIM2->ARR = AC_SAMPLE_VAL;
	//set CCR1 to interrupt @ a 25% duty cycle
	TIM2->CCR1 = DC_SAMPLE_VAL;
	//enable update event interrupt in TIM2
	TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);
	//clear the flag before starting
	TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF);
	//start timer
	TIM2->CR1 |= TIM_CR1_CEN;
	//enable TIM2 in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	//enable interrupts globally
	__enable_irq();
}

