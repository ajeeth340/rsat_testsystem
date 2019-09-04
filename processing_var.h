/*
 * processing_var.h
 *
 *  Created on: Jun 28, 2019
 *      Author: Ajeeth
 */

#ifndef SRC_PROCESSING_VAR_H_
#define SRC_PROCESSING_VAR_H_
//#define RW_DEBUG 1
#define RW_UI_DEBUG 0
#define TEST_BUFFER_SIZE			64                  		// Parser frame includes 64 bytes data
extern XUartNs550 RW1, RW2, RW3, RW4;  				/* The instance of the UartLite Device */
void timer_callback(XScuTimer * TimerInstance);
err_t gps(void);



#endif /* SRC_PROCESSING_VAR_H_ */
