/*
 * timer.h
 *
 *  Created on: Nov 7, 2024
 *      Author: firaz
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#define AC_SAMPLE_VAL 11999 //24MHz x 500us (1/2KHz)
#define DC_SAMPLE_VAL 5999 //24MHz x 1m

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TIM2_init(void);

#endif /* INC_TIMER_H_ */
