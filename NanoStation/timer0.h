/*
 * timer0.h
 *
 *  Created on: Apr 3, 2013
 *      Author: francois
 */

#ifndef TIMER0_H_
#define TIMER0_H_

void init_timer0_tick(void);
void wait_tempo(uint16_t nb);
uint8_t check_timeout(void);
uint8_t check_loop_time(void);

extern volatile uint16_t timeout;
extern volatile uint16_t loop_time;
extern volatile uint16_t radio_timeout;

inline void set_timeout(uint16_t nb) {
	timeout = nb;
}

inline void set_loop_time(uint16_t nb) {
	loop_time = nb;
}

inline void set_radio_timeout(uint16_t nb) {
	radio_timeout = nb;
}

inline void wait_tempo(uint16_t nb) {
	timeout = nb;
	while (timeout != 0);
}

inline uint8_t check_timeout(void) {
	if (timeout == 0) return true;
	return false;
}

inline uint8_t check_loop_time(void) {
	if (loop_time == 0) return true;
	return false;
}

inline uint8_t check_radio_timeout(void) {
	if (radio_timeout == 0) return true;
	return false;
}
#endif /* TIMER0_H_ */
