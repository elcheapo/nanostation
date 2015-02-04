/*
 * timer0.cpp
 *
 *  Created on: Apr 3, 2013
 *      Author: francois
 */

#include "elcheapo_remote.h"
#include "timer0.h"

volatile uint16_t timeout;
volatile uint16_t loop_time;
volatile uint16_t radio_timeout;

void init_timer0_tick(void) {
    // Timer/Counter 0 initialization
    // Clock source: System Clock / 256
    // Clock value: 4 ms per round
    // Mode: PWM
    TCCR0A = (0 << COM0A1) // No OC0A output
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
    OCR0A = 0;
    OCR0B = 0; // not used
    TCNT0=0x00;
    timeout = 0;
}


ISR(TIMER0_OVF_vect) {
	if (timeout != 0) timeout--;
	if (loop_time != 0) loop_time--;
	if (radio_timeout != 0) radio_timeout--;
}
