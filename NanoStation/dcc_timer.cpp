/*
 * dcc_timer.cpp
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#include "elcheapo_remote.h"
#include "organizer.h"
//#include "adc.h"
#include "dcc_timer.h"


#undef DEBUG
#undef DEBUG_IT

// Offsets from TCCRA for the other registers in the same timer
#define tccrb (tccra+1)
#define tccrc (tccra+2)
#define ocra ((volatile uint16_t *) (tccra+8))
#define ocrb ((volatile uint16_t *) (tccra+0xa))
#define ocrc ((volatile uint16_t *) (tccra+0xc))
#define icr ((volatile uint16_t *) (tccra+6))
#define tcnt ((volatile uint16_t *) (tccra+4))



// Constructors ////////////////////////////////////////////////////////////////

DCC_timer::DCC_timer( volatile uint8_t *_tccra,
		volatile uint8_t *_timsk,
		volatile uint8_t *_tifr,
		volatile uint8_t *_ddr) {
	tccra = _tccra;
	timsk = _timsk;
	tifr = _tifr;
	ddr = _ddr;
//	direct = 1;
//	direct_ready = 0;
//	vSemaphoreCreateBinary(packet_sent);
//	vSemaphoreCreateBinary(ready_for_acknowledge);

}


#ifdef USE_TIMER1
DCC_timer timer1 (&TCCR1A,&TIMSK1,&TIFR1,&DDRD); // Voie 1 Gare

#if 0
ISR(TIMER1_OVF_vect) {
#ifdef DEBUG_IT
	ENTER_IT;
#endif
//	if (adc_mode == digital1) start_adc();
	timer1.timer_overflow_interrupt();
#ifdef DEBUG_IT
	EXIT_IT;
#endif
}
#endif
#endif



#ifdef USE_TIMER3
DCC_timer timer3 (&TCCR3A,&TIMSK3,&TIFR3,&DDRE); // Voie 1
ISR(TIMER3_OVF_vect) {
#ifdef DEBUG_IT
	ENTER_IT;
#endif
	if (adc_mode == digital3) start_adc();
	timer3.timer_overflow_interrupt();
#ifdef DEBUG_IT
	EXIT_IT;
#endif
}
#endif
#ifdef USE_TIMER4
DCC_timer timer4 (&TCCR4A,&TIMSK4,&TIFR4,&DDRH); // Voie 2
ISR(TIMER4_OVF_vect) {
#ifdef DEBUG_IT
	ENTER_IT;
#endif
	if (adc_mode == digital4) start_adc();
	timer4.timer_overflow_interrupt();
#ifdef DEBUG_IT
	EXIT_IT;
#endif
}
#endif
#ifdef USE_TIMER5
DCC_timer timer5 (&TCCR5A,&TIMSK5,&TIFR5,&DDRL); // Voie 2 Gare
ISR(TIMER5_OVF_vect) {
#ifdef DEBUG_IT
	ENTER_IT;
#endif
	if (adc_mode == digital5) start_adc();
	timer5.timer_overflow_interrupt();
#ifdef DEBUG_IT
	EXIT_IT;
#endif
}
#endif

inline void DCC_timer::do_send1(void) {
	*ocra = (F_CPU / 1000000L) * PERIOD_1;
	*ocrb = (F_CPU / 1000000L) * PERIOD_1 / 2;
	*ocrc = (F_CPU / 1000000L) * PERIOD_1 / 2;
}

inline void DCC_timer::do_send0(void) {
	*ocra = (F_CPU / 1000000L) * PERIOD_0;
	*ocrb = (F_CPU / 1000000L) * PERIOD_0 / 2;
	*ocrc = (F_CPU / 1000000L) * PERIOD_0 / 2;
}


void DCC_timer::timer_overflow_interrupt(void) {
	// Uses timer x in fast PWM / OCRxA = TOP, OCRxB : = toggle on match, OCRxC : inverted output
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
		if (_doi_packet.bitcount == 0)
			_doi_packet.state = DOI_BSTART;
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


#if 0
void DCC_timer::begin(tmode mode){
	if (mode == digital) {
		_doi_packet.repeat_ctr = 0;
		// Setup the timer to the proper mode for DCC waveform generation
		*tccra = 1 << WGM10| 1 << WGM11
				| 0 << COM1A0	| 0 << COM1A1
				| 0 << COM1B0	| 1 << COM1B1		// Non inverted output on OCxB
				| 1 << COM1C0	| 1 << COM1C1;		// Inverted output on OCxC

		*tccrb = 1<<WGM13 | 1 << WGM12
				| (0<<CS12) | (0<<CS11) | (1<<CS10);// no prescaler, source = sys_clk

		// start with 0's
		*ocra = (F_CPU / 1000000L) * PERIOD_0 ;         // 58µs = 58*16 = 928 clocks
		*ocrb = (F_CPU / 1000000L) * PERIOD_0 / 2;		 // Non inverted output
		*ocrc = (F_CPU / 1000000L) * PERIOD_0 / 2;		 // Inverted output
		// Enable Timer Overflow Interrupt
		*timsk = (1<<TOIE1);
		if (IS_TIMER1) {
			*ddr = T1_OCRA|T1_OCRB|T1_OCRC;
			*dcc_port &= ~T1_OCRA; // start with output OCRA deactivated
		} else {
			*ddr = T3_OCRA|T3_OCRB|T3_OCRC;
			*dcc_port &= ~T3_OCRA; // start with output OCRA deactivated
		}
	} else { // Analog
		*tccra = 0 << WGM10| 1 << WGM11			// PWM Phase correct 9 bit 0-1FF
				| 1 << COM1A0	| 1 << COM1A1		// PWM signal on OCRA - Inverted, set at match with OCRA, cleared at bottom
				| 0 << COM1B0	| 0 << COM1B1
				| 0 << COM1C0	| 0 << COM1C1;

		*tccrb = 0<<WGM13	| 0 << WGM12
				| (0<<CS12) | (1<<CS11) | (0<<CS10);	//  prescaler / 8, source=16 MHz / 511 = 3.9 KHz
		*timsk = 0; 				// no timer interrupt
		if (IS_TIMER1) {
			*ddr = T1_OCRA|T1_OCRB|T1_OCRC;
			*dcc_port &= ~(T1_OCRB|T1_OCRC); // start with output OCRB/C deactivated
		} else {
			*ddr = T3_OCRA|T3_OCRB|T3_OCRC;
			*dcc_port &= ~(T3_OCRB|T3_OCRC); // start with output OCRB/C deactivated
		}
	}
}

#endif

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

	*tccra = 0 << WGM10| 1 << WGM11			// PWM Phase correct 9 bit 0-1FF
			| 1 << COM1A0	| 1 << COM1A1		// PWM signal on OCRA - Inverted, set at match with OCRA, cleared at bottom
			| 1 << COM1B0	| 1 << COM1B1;		// PWM signal on OCRB - Inverted, set at match with OCRB, cleared at bottom

	*tccrb = 0<<WGM13	| 0 << WGM12
			| (0<<CS12) | (1<<CS11) | (1<<CS10);	//  prescaler / 64, source=16 MHz / 511 = 500 Hz
	*timsk = 0; 				// no timer interrupt
	*ddr = (1<<PD_L298_IN1) | (1<<PD_L298_IN2) | (1<<PD_L298_IN3) | (1<<PD_L298_IN4) ;
	*dcc_port &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2)|(1<<PD_L298_IN3)|(1<<PD_L298_IN4)); // start with output IN1/IN2/IN3/IN4 deactivated
}

void DCC_timer::end(void) {
	*timsk = 0; // disable timer interrupt
	*tccra = 0 << WGM10| 0 << WGM11
			| 0 << COM1A0	| 0 << COM1A1		// No output on OCxA
			| 0 << COM1B0	| 0 << COM1B1;		// No output on OCxB
//			| 0 << COM1C0	| 0 << COM1C1;		// No output on OCxC
	*tccrb = 0<<WGM13 | 0 << WGM12
			| (0<<CS12) | (0<<CS11) | (0<<CS10);// timer stopped, no clock
	*ddr = (1<<PD_L298_IN1) | (1<<PD_L298_IN2) | (1<<PD_L298_IN3) | (1<<PD_L298_IN4) ;
	*dcc_port &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2)|(1<<PD_L298_IN3)|(1<<PD_L298_IN4)); // start with output IN1/IN2/IN3/IN4 deactivated
}

void DCC_timer::analog_set_speed(uint8_t channel, uint16_t speed) {
	*tccrb = (0<<WGM13) | (0 << WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);	//  prescaler / 64, source=16 MHz / 511 = 500 Hz
	*tccrb = (0<<WGM13) | (0 << WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);	//  prescaler / 256, source=16 MHz / 511 = 125 Hz
	if (channel == 1) {
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
	if (channel == 1)
		return 511-*ocra;
	else
		return 511-*ocrb;
}

void DCC_timer::analog_set_direction(uint8_t channel, tdirection direction) {
	if (channel == 1) {
		if (direction == off) {
			*dcc_port &= ~((1<<PD_L298_IN1)|(1<<PD_L298_IN2));		// OCxB, OCxC = 0
		} else if (direction == forward) {
			*dcc_port &= ~(1<<PD_L298_IN2);
			*dcc_port |= (1<<PD_L298_IN1);
		} else {
			*dcc_port &= ~(1<<PD_L298_IN1);
			*dcc_port |= (1<<PD_L298_IN2);
		}
	} else {
		if (direction == off) {
			*dcc_port &= ~((1<<PD_L298_IN3)|(1<<PD_L298_IN4));		// OCxB, OCxC = 0
		} else if (direction == forward) {
			*dcc_port &= ~(1<<PD_L298_IN4);
			*dcc_port |= (1<<PD_L298_IN3);
		} else {
			*dcc_port &= ~(1<<PD_L298_IN3);
			*dcc_port |= (1<<PD_L298_IN4);
		}
	}
}
tdirection DCC_timer::analog_get_direction(uint8_t channel) {
	uint8_t temp;
	if (channel == 1) {
		temp = ((*dcc_port) & ((1<<PD_L298_IN1)|(1<<PD_L298_IN2)));
		switch (temp) {
		default:
		case 0: return off;
		case (1<<PD_L298_IN1): return forward;
		case (1<<PD_L298_IN2): return backward;
		}
	} else {
		temp = ((*dcc_port) & ((1<<PD_L298_IN3)|(1<<PD_L298_IN4)));
		switch (temp) {
		default:
		case 0: return off;
		case (1<<PD_L298_IN3): return forward;
		case (1<<PD_L298_IN4): return backward;

		}

	}
}
