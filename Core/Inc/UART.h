/*
 * UART.h
 *
 *  Created on: Nov 6, 2024
 *      Author: firaz
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#define CLOCK_SPEED 24000000
#define BAUD_RATE 115200

void UART_init(void);
void UART_print(char *out_str);


#endif /* INC_UART_H_ */
