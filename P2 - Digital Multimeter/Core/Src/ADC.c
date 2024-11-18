/*
 * ADC.c
 *
 *  Created on: Nov 5, 2024
 *      Author: firaz
 */
#include "main.h"
#include "ADC.h"

volatile uint16_t ADC_result = 0;
volatile uint8_t global_flag = 0;

void collect_samples(uint16_t *store_array, uint16_t sample_size){
	uint16_t idx = 0;
	while(idx < sample_size){
		 while(global_flag != FLAG){}; //wait for global flag
			store_array[idx] = ADC_result; //store the value in the array
			global_flag = NO_FLAG; //reset flag
			idx++;
	}
	idx = 0;
	return; //samples have been collected
}

/**
  * @brief Handle Interrupts
  * @retval None
  */
void ADC1_2_IRQHandler(void){
	 //check end of conversion flag
	 if (ADC1->ISR & ADC_ISR_EOC) {
	        ADC_result = ADC1->DR; //save digital conversion
	        ADC1->ISR |= ADC_ISR_EOC; //clear EOC flag
	        global_flag = FLAG;	//set global flag
	 }
}

/**
  * @brief Print Voltage
  * @retval None
  */
void print_voltage(uint16_t voltage_mv){
	char volt_str[5];

	//convert to digits
	uint8_t ones = voltage_mv / VDIV_ONES;
	uint8_t tenths = (voltage_mv % VDIV_ONES) / VDIV_TENTHS;
	uint8_t hundredths = ((voltage_mv % VDIV_TENTHS)+ROUND_FACTOR) / VDIV_HUNDREDTHS; //+5 rounding factor

	// Adjust for overflow caused by rounding
	if (hundredths > 9) {
		hundredths = 0;
		if (tenths < 9) {
			tenths++;
		} else {
			tenths = 0;
			ones++;
		}
	}
	//convert voltage to a string
	volt_str[0] = ('0' + ones);
	volt_str[1] = '.';
	volt_str[2] = ('0' + tenths);
	volt_str[3] = ('0' + hundredths);
	volt_str[4] = '\0';

	//print to UART
	UART_print(volt_str);
	return;
}

/**
  * @brief Get Avg of 20 Values
  * @retval uint16_t
  */
uint16_t get_avg(uint16_t *store_array, uint16_t sample_size){
	uint32_t sum = 0;
	for(uint16_t i = 0; i < sample_size; i++){
		sum += store_array[i];
	}
	uint32_t avg = sum/sample_size;
	//calibrate to get mV value
	avg = (avg*VREF)/BIT_SIZE;
	avg = (LR_SCALE*avg-LR_OFFSET)/LR_DIVISOR; //linear regression
	return avg;
}

/**
  * @brief Get Min of 20 Values
  * @retval uint16_t
  */
uint16_t get_min(uint16_t *store_array, uint16_t sample_size){
	uint32_t min = store_array[0];
		for(uint16_t i = 1; i < sample_size; i++){
			if(store_array[i]<min){
				min = store_array[i];
			}
		}
		//calibrate to get mV value
		min = (min*VREF)/BIT_SIZE;
		min = min != 0 ? (LR_SCALE*min-LR_OFFSET)/LR_DIVISOR : 0;//linear regression
		return min;
}

/**
  * @brief Get Max of 20 Values
  * @retval None
  */
uint16_t get_max(uint16_t *store_array, uint16_t sample_size){
	uint32_t max = store_array[0];
	for(uint16_t i = 1; i < sample_size; i++){
		if(store_array[i]>max){
			max = store_array[i];
		}
	}
	//calibrate to get mV value
	max = (max*VREF)/BIT_SIZE;
	max = (LR_SCALE*max-LR_OFFSET)/LR_DIVISOR; //linear regression
	return max;
}

/**
  * @brief Initialize ADC
  * @retval None
  */
void ADC_init(void){
	//Enable clocks
	RCC->AHB2ENR |= (RCC_AHB2ENR_ADCEN);
	// set ADC clock to HCLK / 1 synchronous
	ADC123_COMMON->CCR = (1 << ADC_CCR_CKMODE_Pos);

	//power on ADC
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |=  (ADC_CR_ADVREGEN);

	// wait for 20 us
	for(uint32_t i = 0; i < DELAY; i++);

	// Configure DifSel
	// Single ended mode for channel 5 (PA0)
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);

	// Calibrate the ADC
	// ensure the ADC is disabled and single ended calibration
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	// Enable ADC
	// clear ready bit with a 1
	ADC1->ISR |= (ADC_ISR_ADRDY);
	ADC1->CR  |= (ADC_CR_ADEN);

	// wait for ADRDY to be 1
	while (!(ADC1->ISR & ADC_ISR_ADRDY));

	// Configure Sequence
	// Single channel (5) in sequence
	ADC1->SQR1 = (5 << ADC_SQR1_SQ1_Pos);

	// Configure resolution / data alignment
	// Single conversion mode, 12-bit, right aligned
	ADC1->CFGR = 0;

	//configure sample time of 640.6 CC
	ADC1->SMPR1 &= ~(ADC_SMPR1_SMP0);
	ADC1->SMPR1 |= (ADC_SMPR1_SMP0); //set to 640.5 CC

	//configure interrupt
	ADC1->IER |= (ADC_IER_EOCIE);
	NVIC_EnableIRQ(ADC1_2_IRQn);

	/*-----configure GPIO pin (PA0)-----*/
	//configure GPIOA clock
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	//setup MODER to Analog input
	GPIOA->MODER |= (GPIO_MODER_MODE0);
	//no PUPD
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0);
	//set ASC
	GPIOA->ASCR |= (GPIO_ASCR_ASC0);

	//start a conversion
	ADC1->CR |= ADC_CR_ADSTART;
}

