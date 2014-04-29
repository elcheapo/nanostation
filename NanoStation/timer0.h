/*
 * timer0.h
 *
 *  Created on: Apr 3, 2013
 *      Author: francois
 */

#ifndef TIMER0_H_
#define TIMER0_H_

void init_timer0_tick(void);
void wait_tempo(uint8_t nb);
void set_timeout(uint8_t nb);
void set_loop_time(uint8_t nb);
uint8_t check_timeout(void);
uint8_t check_loop_time(void);

#endif /* TIMER0_H_ */
