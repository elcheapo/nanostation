/*
 * dcc_timer.h
 *
 *  Created on: 25 avr. 2012
 *      Author: florrain
 */

#ifndef DCC_TIMER_H_
#define DCC_TIMER_H_

// define which timers are used for DCC output
#define USE_TIMER1

// define some handy names for the states of the ISR
#define DOI_IDLE     (0)
#define DOI_PREAMBLE (1)
#define DOI_BSTART   (2)
#define DOI_BYTE     (3)
#define DOI_XOR      (4)
#define DOI_INTER_PACKET (5)
#define DOI_LAST_BIT (6)

typedef struct {
	uint8_t state;                            // current state
	uint8_t ibyte;                            // current index of byte in message
	uint8_t bitcount;							// current bit index n char
	uint8_t cur_byte;                         // current byte value
	uint8_t xor_byte;                         // actual check
	uint8_t repeat_ctr;						// current number of repeat
} doi;

/// This are timing definitions from NMRA
#define PERIOD_1   116L                  // 116us for DCC 1 pulse - do not change
//#define PERIOD_1   120L                  // 104us for DCC 1 pulse - do not change
#define PERIOD_0   232L                  // 232us for DCC 0 pulse - do not change
//#define PERIOD_0   200L                  // 200us for DCC 0 pulse - do not change

class DCC_timer
{
private:
	volatile uint8_t *tccra;
	volatile uint8_t *timsk;
	volatile uint8_t *tifr;

	doi _doi_packet;
	message current_message;
	volatile uint8_t pkt_abort;
	volatile uint8_t pkt_ready;
	volatile uint8_t ack_ready;

	void do_send0(void);
	void do_send1(void);

public:
	DCC_timer( volatile uint8_t *tccra,
			volatile uint8_t *timsk,
			volatile uint8_t *tifr
			);
	void begin(tmode mode);
	void end(void);
	void send_dcc_packet(message * current);
	void digital_on(uint8_t channel);
	void digital_off(uint8_t channel);
	void analog_set_speed(uint8_t channel, uint16_t speed);
	uint16_t analog_get_speed(uint8_t channel);
	void analog_set_direction(uint8_t channel, tdirection direction);
	tdirection analog_get_direction(uint8_t channel);

	void timer_overflow_interrupt(void);
	void match_B_interrupt(void);
	void abort_dcc(void);

	tmode get_mode(void);
	uint8_t dcc_is_powered(void);

	uint8_t dcc_busy(void);
	uint8_t dcc_ack_ready(void);
};

#define dcc_port (ddr+1)
//#define IS_TIMER1 ((uint16_t)(ddr) == 0x24)
// 328P only has one timer1

inline tmode DCC_timer::get_mode(void) {
	uint8_t mode = *tccra & 0x03;
	if (mode == 0x02) return analog;
	if (mode == 0x03) return digital;
	return dcc_off;
}

inline uint8_t DCC_timer::dcc_is_powered(void) {
	if ((PORTB & (1<<PB1_OC1A)) != 0)
		return true;
	return false;
}

inline uint8_t DCC_timer::dcc_busy(void) {return (pkt_ready);};


extern DCC_timer timer1;

#endif /* DCC_TIMER_H_ */
