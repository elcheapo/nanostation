/*
 * timer0.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: francois
 */

#include "elcheapo_remote.h"

volatile uint8_t timeout;
volatile uint8_t loop_time;

void init_timer0_tick(void) {
    // Timer/Counter 0 initialization
    // Clock source: System Clock / 256
    // Clock value: 4 ms per round
    // Mode: PWM
    // OC0 output: RETRO Eclairage lcd
    TCCR0A = (1 << COM0A1) // OC0A output
    		|(0 << COM0A0)
    		|(0 << COM0B1) // No OC0B output
    		|(0 << COM0B0)
    		|(1 << WGM01)	// WGM = 011: Fast PWM
    		|(1 << WGM00);
    TCCR0B = (0 << FOC0A)	// 0 = no forced compare match
    		|(0 << FOC0B)
    		|(0 << WGM02)
    		|(1 << CS02)	// 100 = div256
    		|(0 << CS01)
    		|(0 << CS00);
    TIMSK0 = (1<<TOV0); // interrupt on overflow every 4.096 ms
#ifdef RADIO1
    OCR0A = 0x30;   // 30/FF% duty cycle on lcd backlight ...
#else
    OCR0A = 0x40;   // 40/FF% duty cycle on lcd backlight ...
#endif
    OCR0B = 0; // not used
    TCNT0=0x00;
    timeout = 0;
}

void wait_tempo(uint8_t nb) {
	timeout = nb;
	while (timeout != 0);
}
void set_timeout(uint8_t nb) {
	timeout = nb;
}
void set_loop_time(uint8_t nb) {
	loop_time = nb;
}

uint8_t check_timeout(void) {
	if (timeout == 0) return true;
	return false;
}
uint8_t check_loop_time(void) {
	if (loop_time == 0) return true;
	return false;
}
ISR(TIMER0_OVF_vect) {
	if (timeout != 0) timeout--;
	if (loop_time != 0) loop_time--;
}
