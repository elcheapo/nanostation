/*
 * dcc_timer.cpp
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#include "elcheapo_remote.h"
//#include "organizer.h"
#include "adc.h"
#include "dcc_timer.h"


#undef LOCAL_DEBUG
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
	direct = 1;
	direct_ready = 0;
	vSemaphoreCreateBinary(packet_sent);
	vSemaphoreCreateBinary(ready_for_acknowledge);

}

#ifdef USE_TIMER1
DCC_timer timer1 (&TCCR1A,&TIMSK1,&TIFR1,&DDRB); // Voie 1 Gare
ISR(TIMER1_OVF_vect) {
#ifdef DEBUG_IT
	ENTER_IT;
#endif
	if (adc_mode == digital1) start_adc();
	timer1.timer_overflow_interrupt();
#ifdef DEBUG_IT
	EXIT_IT;
#endif
}
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
#ifdef LOCAL_DEBUG
	DCC_BIT_HIGH;
#endif
	*ocra = (F_CPU / 1000000L) * PERIOD_1;
	*ocrb = (F_CPU / 1000000L) * PERIOD_1 / 2;
	*ocrc = (F_CPU / 1000000L) * PERIOD_1 / 2;
}

inline void DCC_timer::do_send0(void) {
#ifdef LOCAL_DEBUG
	DCC_BIT_LOW;
#endif
	*ocra = (F_CPU / 1000000L) * PERIOD_0;
	*ocrb = (F_CPU / 1000000L) * PERIOD_0 / 2;
	*ocrc = (F_CPU / 1000000L) * PERIOD_0 / 2;
}


void DCC_timer::timer_overflow_interrupt(void) {
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
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
				xSemaphoreGiveFromISR (packet_sent, &xHigherPriorityTaskWoken); // tell the world about it ...
				_doi_packet.repeat_ctr = 0;
				if( xHigherPriorityTaskWoken != pdFALSE ) taskYIELD();
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
		_doi_packet.state = DOI_INTER_PACKET;
		_doi_packet.bitcount = 1;
		_doi_packet.repeat_ctr ++;
		xSemaphoreGiveFromISR(ready_for_acknowledge, &xHigherPriorityTaskWoken); // tell the world about it ...
		if( xHigherPriorityTaskWoken != pdFALSE ) taskYIELD();
		break;
	}
	default:
		while(1);
		break;
	}
}

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
void DCC_timer::abort_dcc(void){
	pkt_abort = 1;
}

void DCC_timer::send_dcc_packet(message * current){
	// MUST be called when pkt_ready is 0
	if (direct == 0) {
		current_message = *current;
		pkt_ready = 1;
	}
}

void DCC_timer::send_direct_dcc_packet(message * direct) {
	// MUST be called when pkt_ready is 0
	current_message = *direct;
	pkt_ready = 1;
}

void DCC_timer::end(void) {
	*timsk = 0; // disable timer interrupt
	*tccra = 0 << WGM10| 0 << WGM11
			| 0 << COM1A0	| 0 << COM1A1		// No output on OCxA
			| 0 << COM1B0	| 0 << COM1B1		// No output on OCxB
			| 0 << COM1C0	| 0 << COM1C1;		// No output on OCxC
	*tccrb = 0<<WGM13 | 0 << WGM12
			| (0<<CS12) | (0<<CS11) | (0<<CS10);// timer stopped, no clock
	if (IS_TIMER1) {
		*ddr = T1_OCRA|T1_OCRB|T1_OCRC;
		*dcc_port &= ~(T1_OCRA|T1_OCRB|T1_OCRC); // turn off all signals
	} else {
		*ddr = T3_OCRA|T3_OCRB|T3_OCRC;
		*dcc_port &= ~(T3_OCRA|T3_OCRB|T3_OCRC); // turn off all signals
	}

}

void DCC_timer::analog_set_speed(uint16_t speed) {
	// start PWM at low frequency at low speed then go to higher frequency
	if (speed > 300)
		*tccrb = 0<<WGM13 | 0 << WGM12 | (0<<CS12) | (1<<CS11) | (0<<CS10);	//  prescaler / 8, source=16 MHz / 511 = 3.9 KHz
	else
		*tccrb = 0<<WGM13 | 0 << WGM12 | (0<<CS12) | (1<<CS11) | (1<<CS10);	//  prescaler / 64, source=16 MHz / 511 = 500 Hz

	if (speed < 512)
		*ocra = 511-speed;
	else
		*ocra = 511;
}
uint16_t DCC_timer::analog_get_speed(void) {
	return 511-*ocra;
}

void DCC_timer::analog_set_direction(tdirection direction) {
	if (IS_TIMER1) {
		if (direction == off) {
			*dcc_port &= ~(T1_OCRB|T1_OCRC);		// OCxB, OCxC = 0
		} else if (direction == forward) {
			*dcc_port &= ~T1_OCRC; 	// OCxC = 0
			*dcc_port |= T1_OCRB; 	// OCxB = 1
		} else {
			*dcc_port &= ~T1_OCRB;	// OCxB = 0
			*dcc_port |= T1_OCRC;		// OCxC = 1
		}
	} else {
		if (direction == off) {
			*dcc_port &= ~(T3_OCRB|T3_OCRC);		// OCxB, OCxC = 0
		} else if (direction == forward) {
			*dcc_port &= ~T3_OCRC; 	// OCxC = 0
			*dcc_port |= T3_OCRB; 	// OCxB = 1
		} else {
			*dcc_port &= ~T3_OCRB;	// OCxB = 0
			*dcc_port |= T3_OCRC;		// OCxC = 1
		}
	}
}
tdirection DCC_timer::analog_get_direction(void) {
	uint8_t temp;
	if (IS_TIMER1)
		temp = ((*dcc_port) & (T1_OCRB|T1_OCRC)) >> 2;
	else
		temp = (*dcc_port) & (T3_OCRB|T3_OCRC);
	switch (temp) {
	default:
	case 0: return off;
	case 0x10: return forward;
	case 0x20: return backward;
	}
}

