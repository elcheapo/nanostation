/*
 * dcc_timer.cpp
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#include "elcheapo_remote.h"
#include "organizer.h"
#include "HardwareSerial.h"
//#include "adc.h"
#include "dcc_timer.h"


#undef DEBUG
#define DEBUG_IT

// Offsets from TCCRA for the other registers in the same timer
#define tccrb (tccra+1)
#define tccrc (tccra+2)
#define ocra ((volatile uint16_t *) (tccra+8))
#define ocrb ((volatile uint16_t *) (tccra+0xa))
//#define ocrc ((volatile uint16_t *) (tccra+0xc))
#define icr ((volatile uint16_t *) (tccra+6))
#define tcnt ((volatile uint16_t *) (tccra+4))



// Constructors ////////////////////////////////////////////////////////////////

DCC_timer::DCC_timer( volatile uint8_t *_tccra,
		volatile uint8_t *_timsk,
		volatile uint8_t *_tifr) {
	tccra = _tccra;
	timsk = _timsk;
	tifr = _tifr;
	pkt_ready=0;
}


// from config.h
//#define PB1_OC1A 		1		// out
//#define PB2_OC1B		2		// out
//#define PD_L298_IN3		4		// out
//#define PD_L298_IN4		5		// out
//#define PD_L298_IN1		6		// out
//#define PD_L298_IN2		7		// out

DCC_timer timer1 (&TCCR1A,&TIMSK1,&TIFR1); // Timer 1 controls DCC signal


ISR(TIMER1_OVF_vect) {
	timer1.timer_overflow_interrupt();
}
ISR(TIMER1_COMPB_vect) {
	timer1.match_B_interrupt();
}

inline void DCC_timer::do_send1(void) {
	*ocra = (F_CPU / 1000000L) * PERIOD_1;
	*ocrb = (F_CPU / 1000000L) * PERIOD_1 / 2;
}

inline void DCC_timer::do_send0(void) {
	*ocra = (F_CPU / 1000000L) * PERIOD_0;
	*ocrb = (F_CPU / 1000000L) * PERIOD_0 / 2;
}

void DCC_timer::match_B_interrupt(void) {
	// Clear all signals
	PORTD &= ~((1<<PD_L298_IN1) | (1<<PD_L298_IN2) |(1<<PD_L298_IN3) |(1<<PD_L298_IN4));
	// Set 2 & 4
	PORTD |= (1<<PD_L298_IN2) |(1<<PD_L298_IN4);
}


void DCC_timer::timer_overflow_interrupt(void) {
	// Clear all signals
	PORTD &= ~((1<<PD_L298_IN1) | (1<<PD_L298_IN2) |(1<<PD_L298_IN3) |(1<<PD_L298_IN4));
	// Set 1 & 3
	PORTD |= (1<<PD_L298_IN1) |(1<<PD_L298_IN3);

	// Uses timer x in fast PWM / OCRxA = TOP, OCRxB : TOV and match B toggle pins
	switch (_doi_packet.state) {
	case DOI_INTER_PACKET: {
		do_send1();
		/* Insure that we have 5 ms between packets as per DCC 9.2 */
		/* So we wait for 14 (+ 14 or 20) 0/1's before processing the next packet */
		/* 14 * 232µs * 14*112µs = 4.8 ms + packet loading time */
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0) _doi_packet.state = DOI_IDLE;
		if (pkt_abort != 0) {	// in case we need to stop repeating packet
			pkt_ready = 0;
			pkt_abort = 0;
		}
		break;
	}
	case DOI_IDLE: {
		do_send1();
		if (pkt_ready != 0) {
			if (_doi_packet.repeat_ctr >= current_message.repeat) {
				pkt_ready = 0; // DONE processing packet and repeat
				_doi_packet.repeat_ctr = 0;
			} else {
				// send / resend message
				_doi_packet.state = DOI_PREAMBLE;                           // current state
				_doi_packet.ibyte = 0;
				_doi_packet.bitcount = 0;
				_doi_packet.xor_byte = 0;
				if (current_message.type == is_prog)		// TODO: Update logic
					_doi_packet.bitcount = 25;   		// long preamble if service mode
				else
					_doi_packet.bitcount = 14;     	// regular preamble
			}
		}
		break;
	}
	case DOI_PREAMBLE: {
		do_send1();
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0) {
			_doi_packet.state = DOI_BSTART;
#ifdef DEBUG_IT
			Serial.println();
#endif
		}
		break;
	}
	case DOI_BSTART: {
		do_send0();
		if (current_message.size == _doi_packet.ibyte)	{ // message done, goto xor
			_doi_packet.cur_byte = _doi_packet.xor_byte;
			_doi_packet.state = DOI_XOR;
			_doi_packet.bitcount = 8;
		} else { // get next addr or data
			_doi_packet.cur_byte = current_message.dcc[_doi_packet.ibyte++];
			_doi_packet.xor_byte ^= _doi_packet.cur_byte;
			_doi_packet.state = DOI_BYTE;
			_doi_packet.bitcount = 8;
#ifdef DEBUG_IT
			Serial.print(_doi_packet.cur_byte,16);
			Serial.write('-');
#endif
		}
		break;
	}
	case DOI_BYTE:	{
		if (_doi_packet.cur_byte & 0x80) do_send1(); else do_send0();
		_doi_packet.cur_byte <<= 1;
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0)
			_doi_packet.state = DOI_BSTART;
		break;
	}
	case DOI_XOR: {
		if (_doi_packet.cur_byte & 0x80) do_send1(); else do_send0();
		_doi_packet.cur_byte <<= 1;
		_doi_packet.bitcount--;
		if (_doi_packet.bitcount == 0) {
			_doi_packet.state = DOI_LAST_BIT;
		}
		break;
	}
	case DOI_LAST_BIT: {
		do_send1();
		ack_ready = 1; // done sending the packet, if we are waiting for an ack, look for it now ...
		_doi_packet.state = DOI_INTER_PACKET;
		_doi_packet.bitcount = 1;
		_doi_packet.repeat_ctr ++;
		break;
	}
	default:
		while(1);
		break;
	}
}

void DCC_timer::abort_dcc(void){
	pkt_abort = 1;
}

void DCC_timer::send_dcc_packet(message * current){
	// MUST be called when pkt_ready is 0
	current_message = *current;
	pkt_ready = 1;
	ack_ready = 0;
}

void DCC_timer::begin(tmode mode){
	if (mode == digital) {
		_doi_packet.repeat_ctr = 0;
		// Setup the timer to the proper mode for DCC waveform generation
		*tccra = (1 << WGM10) | (1 << WGM11)
				| (0 << COM1A0)	| (0 << COM1A1)		// OCx Pin not used on atmega3328P
				| (0 << COM1B0)	| (0 << COM1B1);	// interrupts toogle the pins on Match and OVF

		*tccrb = (1<<WGM13) | (1 << WGM12)
				| (0<<CS12) | (0<<CS11) | (1<<CS10);// no prescaler, source = sys_clk

		// start with 0's
		*ocra = (F_CPU / 1000000L) * PERIOD_0 ;         // 58µs = 58*16 = 928 clocks
		*ocrb = (F_CPU / 1000000L) * PERIOD_0 / 2;		// use IT to toggle pins
		// Enable Timer Overflow and match on B Interrupts
		*timsk = ((1<<TOIE1) | (1<<OCIE1B));
		// Now set the I/O pins to start digital signal
		DDRD |= (1<<PD_L298_IN1) | (1<<PD_L298_IN2) |(1<<PD_L298_IN3) |(1<<PD_L298_IN4);
		PORTD |=  (1<<PD_L298_IN1) | (1<<PD_L298_IN3);
		DDRB |= (1<<PB1_OC1A) | (1<<PB2_OC1B);
		PORTB |= (1<<PB1_OC1A) | (1<<PB2_OC1B);
	} else { // Analog
		*tccra = (0 << WGM10) | (1 << WGM11)			// PWM Phase correct 9 bit 0-1FF
				| (1 << COM1A0)	| (1 << COM1A1)		// PWM signal on OCRA - Inverted, set at match with OCRA, cleared at bottom
				| (1 << COM1B0)	| (1 << COM1B1);	// PWM signal on OCRB - Inverted, set at match with OCRB, cleared at bottom

		*tccrb = (0<<WGM13)	| (0 << WGM12)
				| (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 511 = 125 Hz
		*timsk = 0; 				// no timer interrupt
		DDRD = (1<<PD_L298_IN1) | (1<<PD_L298_IN2) | (1<<PD_L298_IN3) | (1<<PD_L298_IN4) ;
		PORTD &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2)|(1<<PD_L298_IN3)|(1<<PD_L298_IN4)); // start with output IN1/IN2/IN3/IN4 deactivated
	}
}
void DCC_timer::end(void) {
	*timsk = 0; // disable timer interrupt
	*tccra = 0 << WGM10| 0 << WGM11
			| 0 << COM1A0	| 0 << COM1A1		// No output on OCxA
			| 0 << COM1B0	| 0 << COM1B1;		// No output on OCxB
//			| 0 << COM1C0	| 0 << COM1C1;		// No output on OCxC
	*tccrb = 0<<WGM13 | 0 << WGM12
			| (0<<CS12) | (0<<CS11) | (0<<CS10);// timer stopped, no clock
	DDRD = (1<<PD_L298_IN1) | (1<<PD_L298_IN2) | (1<<PD_L298_IN3) | (1<<PD_L298_IN4) ;
	PORTD &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2)|(1<<PD_L298_IN3)|(1<<PD_L298_IN4)); // start with output IN1/IN2/IN3/IN4 deactivated
}

void DCC_timer::analog_set_speed(uint8_t channel, uint16_t speed) {
//	*tccrb = (0<<WGM13) | (0 << WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);	//  prescaler / 64, source=16 MHz / 511 = 500 Hz
	*tccrb = (0<<WGM13) | (0 << WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 511 = 125 Hz
	if (channel == CHANNEL_1) {
		if (speed < 512)
			*ocra = 511-speed;
		else
			*ocra = 511;
	} else {
		if (speed < 512)
			*ocrb = 511-speed;
		else
			*ocrb = 511;
	}
}
uint16_t DCC_timer::analog_get_speed(uint8_t channel) {
	if (channel == CHANNEL_1)
		return 511-*ocra;
	else
		return 511-*ocrb;
}

void DCC_timer::analog_set_direction(uint8_t channel, tdirection direction) {
	if (channel == CHANNEL_1) {
		if (direction == off) {
			PORTD &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2));		// OCxB, OCxC = 0
		} else if (direction == forward) {
			PORTD &= ~(1<<PD_L298_IN2);
			PORTD |= (1<<PD_L298_IN1);
		} else {
			PORTD &= ~(1<<PD_L298_IN1);
			PORTD |= (1<<PD_L298_IN2);
		}
	} else {
		if (direction == off) {
			PORTD &= ~((1<<PD_L298_IN3)|(1<<PD_L298_IN4));		// OCxB, OCxC = 0
		} else if (direction == forward) {
			PORTD &= ~(1<<PD_L298_IN4);
			PORTD |= (1<<PD_L298_IN3);
		} else {
			PORTD &= ~(1<<PD_L298_IN3);
			PORTD |= (1<<PD_L298_IN4);
		}
	}
}
tdirection DCC_timer::analog_get_direction(uint8_t channel) {
	uint8_t temp;
	if (channel == CHANNEL_1) {
		temp = ( PORTD & ((1<<PD_L298_IN1)|(1<<PD_L298_IN2)));
		switch (temp) {
		default:
		case 0: return off;
		case (1<<PD_L298_IN1): return forward;
		case (1<<PD_L298_IN2): return backward;
		}
	} else {
		temp = ( PORTD & ((1<<PD_L298_IN3)|(1<<PD_L298_IN4)));
		switch (temp) {
		default:
		case 0: return off;
		case (1<<PD_L298_IN3): return forward;
		case (1<<PD_L298_IN4): return backward;

		}

	}
}

// Turn ON or OFF the two channels for DCC
// disabling one allows for programming operation

void DCC_timer::digital_on(uint8_t channel) {
	if (channel == CHANNEL_1) {
		PORTB |= (1<<PB1_OC1A);
	} else {
		PORTB |= (1<<PB2_OC1B);
	}
};
void DCC_timer::digital_off(uint8_t channel) {
	if (channel == CHANNEL_1) {
		PORTB &= ~(1<<PB1_OC1A);
	} else {
		PORTB &= ~(1<<PB2_OC1B);
	}

};
