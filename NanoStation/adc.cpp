/*
 * adc.c
 *
 *  Created on: 3 mars 2011
 *      Author: florrain
 */


#include "elcheapo_remote.h"
#include "config.h"
#include "adc.h"

const uint8_t analog_ref=0x40; // AVCC as ADC reference

void adc_init(void) {

	// set a2d prescale factor to 128
	// 16 MHz / 128 = 125 KHz, inside the desired 50-200 KHz range.
	ADCSRA = (1<<ADEN) // Enable ADC
			|(0<<ADSC) // Do not start conversion
			|(0<<ADATE)// No auto trigger
			|(0<<ADIF) // Interrupt flag
			|(0<<ADIE) // Interrupt Enable
			|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	// at 125 KHz, one conversion take 8µs * 13 = 104 µS
	ADCSRB = (0<<ACME) // Analog Comparator disabled
			|(0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0); // No Trigger
	ADMUX = analog_ref
			|(0<<ADLAR) // Right justified result
			|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	DIDR0 = 0x0f; // disable digital Input pin

}

int16_t get_adc(uint8_t channel){
	uint8_t temp;
	ADMUX = analog_ref | (0<<ADLAR) |(channel & 0x0f);
	ADCSRA |= (1<<ADSC); // start conversion
	while((ADCSRA & (1<<ADSC)) != 0);
	temp = ADCL;
	return ((ADCH << 8) | temp);

}


