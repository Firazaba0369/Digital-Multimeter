/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "ADC.h"
#include "timer.h"
#include "UART.h"
#define FFT_SIZE 2048
#define SAMPLE_RATE_HZ 2000
#define DIG_SIZE 6
#define DC_SAMPLE_SIZE 2000
#define SAMPLE 1
#define NO_SAMPLE 0
#define SCALED_SLOPE 10080
#define SCALED_INTERCEPT 1
#define SCALING_FACTOR 10000
#define REFINE_VAL 5



arm_rfft_fast_instance_f32 fftHandler;
//global variables
volatile float fft_in[FFT_SIZE];
volatile float fft_out[FFT_SIZE];
volatile float freq_mag[FFT_SIZE/2] = {0};
volatile uint8_t dc_sampling = NO_SAMPLE;

//function prototypes
void output_measurements(uint16_t freq, uint16_t dc_volt, uint16_t pp_volt, uint16_t rms_volt);
uint16_t refine_volt(uint16_t voltage, uint16_t *array, uint16_t array_size);
void compute_fft(uint16_t *samples, uint16_t dc_offset);
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  //initialize FFT, ADC, timer,and UART
  arm_rfft_fast_init_f32(&fftHandler, FFT_SIZE);
  ADC_init();
  TIM2_init();
  UART_init();

  //sampling arrays for AC and DC
  uint16_t ac_samples[FFT_SIZE] = {0};
  uint16_t dc_samples[DC_SAMPLE_SIZE] = {0};
  uint16_t peak_freq = 0;

  //reset screen and cursor
  UART_print("\x1B[2J");
  UART_print("\x1B[H");
  while (1)
  {
	  //collect AC and DC samples
	  //ac_sampling = SAMPLE;
	  collect_samples(ac_samples, FFT_SIZE);
	  //ac_sampling = NO_SAMPLE;
	  dc_sampling = SAMPLE; //still may not work. Double check this
	  collect_samples(dc_samples, DC_SAMPLE_SIZE);
	  dc_sampling = NO_SAMPLE;

	  //offset to remove for FFT calculation
	  uint16_t offset = get_avg(ac_samples, FFT_SIZE);

	  //compute fft and perform calculations
	  compute_fft(ac_samples, offset);
	  float max_magnitude = 0.0f;
	  for(int i = 0; i<FFT_SIZE/2;i++){
		  uint16_t cur_freq = (uint16_t)(i * SAMPLE_RATE_HZ / ((float)FFT_SIZE));
		  //get peak frequency through magnitude
		  if (freq_mag[i] > max_magnitude) {
			  max_magnitude = freq_mag[i];
			  peak_freq = cur_freq;
		  }
	  }

	  //compute and refine DC measurement
	  uint16_t vdc = (get_max(dc_samples, DC_SAMPLE_SIZE)+get_min(dc_samples, DC_SAMPLE_SIZE))/2;//get_avg(dc_samples, DC_SAMPLE_SIZE);
	  vdc = refine_volt(vdc,dc_samples,DC_SAMPLE_SIZE);

	  //compute and refine AC measurements
	  uint16_t ac_max = get_max(ac_samples, FFT_SIZE);
	  uint16_t ac_min = get_min(ac_samples, FFT_SIZE);
	  uint16_t vpp = (ac_max - ac_min);
	  vpp = vpp>99 ? refine_volt(vpp,ac_samples,FFT_SIZE): 0;
	  uint16_t vrms = (vpp/(2*sqrt(2)));
	  vrms = vpp > 0 ? refine_volt(vrms,ac_samples,FFT_SIZE): 0;

	  //calibrate frequency
	  uint16_t calibrated_freq = vpp > 10 ? ((SCALED_SLOPE * peak_freq)/ SCALING_FACTOR)+ SCALED_INTERCEPT: 0;

	  //output to terminal
	  output_measurements(calibrated_freq, vdc, vpp, vrms);
  }
}

/**
  * @brief Improve Accuracy of voltages
  * @retval None
  */
uint16_t refine_volt(uint16_t voltage, uint16_t *array, uint16_t array_size){
	uint16_t top_nums[REFINE_VAL] = {0};
	uint16_t idx = 0;
	for (int i = 0; i < array_size; i++) {
		if (abs(array[i] - voltage) < 10) {
	            if (idx < REFINE_VAL) {
	                top_nums[idx] = array[i];
	                idx++;
	            } else {
	                break;  // Stop once refine amount reached
	            }
	        }
	    }

		//add voltage to array if refine amount is not reached
	    while (idx < REFINE_VAL) {
	        top_nums[idx] = voltage;
	        idx++;
	    }

	    // Return the average of the top numbers closest to the voltage
		uint32_t sum = 0;
		for(uint16_t i = 0; i < REFINE_VAL; i++){
			sum += top_nums[i];
		}
	    return sum/REFINE_VAL;
}

/**
  * @brief Timer 2 interrupt handler
  * @retval None
  */
void TIM2_IRQHandler(void){
	//check for update event flag of ARR
	if (TIM2->SR & TIM_SR_UIF){
		if (!dc_sampling){ //figure this out
			ADC1->CR |= ADC_CR_ADSTART;
		}
		//clear update event interrupt flag
		TIM2->SR &= ~(TIM_SR_UIF);
	}
	else if (TIM2->SR & TIM_SR_CC1IF){
		if (dc_sampling){
			ADC1->CR |= ADC_CR_ADSTART;
		}
		//clear and update CCR1 flag
		TIM2->SR &= ~(TIM_SR_CC1IF);
	}
}

/**
  * @brief Compute FFT and update global array
  * @retval None
  */
void compute_fft(uint16_t *samples, uint16_t dc_offset){
	//
	//convert to float
	for (int i = 0; i < FFT_SIZE; i++) {

		fft_in[i] = (((((float)samples[i]*VREF)/BIT_SIZE)-dc_offset)/1000);
	 }
	 // Perform FFT
	 arm_rfft_fast_f32(&fftHandler, fft_in, fft_out, 0);
	 arm_cmplx_mag_f32(fft_out, freq_mag, FFT_SIZE/2);
}

/**
  * @brief output measurements
  * @retval None
  */
void output_measurements(uint16_t freq, uint16_t dc_volt, uint16_t pp_volt, uint16_t rms_volt){
	//convert to string
	char freq_str[DIG_SIZE];
	sprintf(freq_str, "%u", freq);

	//reset screen and cursor
	UART_print("\x1B[2J");
	UART_print("\x1B[H");

	//print frequency
	UART_print("Frequency: ");
	UART_print(freq_str);
	UART_print(" Hz\n");
	UART_print("\x1B[2;0H");

	//print DC voltage bar
	UART_print("Vdc  ");
	UART_print("#");
	uint16_t bar_idx = 100;
	while(dc_volt > bar_idx){
	  UART_print("#");
	  bar_idx += 100;
	}
	UART_print("\n");
	UART_print("\x1B[3;0H");

	//print Peak-to-Peak voltage
	UART_print("Vpp  ");
	UART_print("#");
	bar_idx = 100;
	while(pp_volt > bar_idx){
	  UART_print("#");
	  bar_idx += 100;
	}
	UART_print("\n");
	UART_print("\x1B[4;0H");

	//print RMS voltage
	UART_print("Vrms ");
	UART_print("#");
	bar_idx = 100;
	while(rms_volt > bar_idx){
	  UART_print("#");
	  bar_idx += 100;
	}
	UART_print("\n");
	UART_print("\x1B[5;6H");

	//print horizontal bar axis
	UART_print("|");
	for(int i = 0; i<6; i++){
	  UART_print("----|");
	}
	UART_print("\n");
	UART_print("\x1B[6;0H");

	//print bar values
	uint8_t first_dig = 0;
	uint8_t second_dig = 0;
	UART_print("\x1B[4C");
	for(int i = 0; i<7; i++){
	  //set digit values
	  if(i%2 == 0){
		  if(i!=0){
			  first_dig++;
		  }
		  second_dig = 0;
	  }
	  else{
		  second_dig = 5;
	  }
	  //format
	  char dig1[2] = {'0' + first_dig, '\0'};
	  char dig2[2] = {'0' + second_dig, '\0'};
	  UART_print(dig1);
	  UART_print(".");
	  UART_print(dig2);
	  if(i!=6){
		  UART_print("\x1B[2C");
	  }
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
