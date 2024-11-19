/*
 * UART.c
 *
 *  Created on: Nov 6, 2024
 *      Author: firaz
 */
#include "main.h"
#include "UART.h"

void UART_print(char *out_str){
	//check string
	if(out_str == NULL){
		return;
	}
	//check character isn't null terminator
	while(*out_str != '\0'){
		//wait for transmission flag
		while(!(USART2->ISR & USART_ISR_TXE));
		USART2->TDR = *out_str; //write string to USART
		out_str++; //increment string pointer
	}
}

void UART_init(void){
	//Enable clock for GPIOA and USART
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	//Set GPIOA2 & GPIOA3 to alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);

	//Enable alternate functionality
	GPIOA-> AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
	GPIOA-> AFR[0] |= ((0x7UL << GPIO_AFRL_AFSEL2_Pos) | (0x7UL << GPIO_AFRL_AFSEL3_Pos));

	//Set no PUPD
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);

	//Set GPIOA speed to high
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3);

	//USART setup
	//Disable UE to set M0, M1, BRR, STOP,
	USART2->CR1 &= ~(USART_CR1_UE);
	//Set M0, M1 for 8 bit word
	USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
	//Set stop bit to 1 bit
	USART2->CR2 &= ~(USART_CR2_STOP);
	//BRR set baud rate 24MHz/115.2Kbps
	USART2->BRR = (CLOCK_SPEED/BAUD_RATE);
	//RE - receiver enable
	USART2->CR1 |= (USART_CR1_RE);
	//Set oversampling mode
	USART2->CR1 &= ~(USART_CR1_OVER8);
	//UE - USART enable
	USART2->CR1 |= (USART_CR1_UE);
	//TE - transmit enable  RE - Receive enable
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
}
