/*
 * ADC.h
 *
 *  Created on: Nov 5, 2024
 *      Author: firaz
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_
#define FLAG 1
#define NO_FLAG 0
#define VREF 3300
#define BIT_SIZE 4095
#define LR_SCALE 10265
#define LR_OFFSET 13
#define LR_DIVISOR 10000
#define DELAY 50000
#define VDIV_ONES 1000
#define VDIV_TENTHS	100
#define VDIV_HUNDREDTHS 10
#define ROUND_FACTOR 5

/*-----Global Variables-----*/
//result of ADC conversion
extern volatile uint16_t ADC_result;
//interrupt flag
extern volatile uint8_t global_flag;

void ADC_init(void);
void collect_samples(uint16_t *store_array, uint16_t sample_size);
void ADC1_2_IRQHandler(void);
void print_voltage(uint16_t voltage_mv);
uint16_t get_avg(uint16_t *store_array, uint16_t sample_size);
uint16_t get_min(uint16_t *store_array, uint16_t sample_size);
uint16_t get_max(uint16_t *store_array, uint16_t sample_size);

#endif /* INC_ADC_H_ */
