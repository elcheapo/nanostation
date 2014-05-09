/*
 * main.c
 *
 *  Created on: Mar 1, 2014
 *      Author: francois
 *      Nano Receiver and PWM controller
 *      "NanoStation"
 */


#include "elcheapo_remote.h"
#include "timer0.h"
#include "dcc_timer.h"

#include "hal_nrf_reg.h"
#include "hal_nrf.h"

#include "radio.h"
#include "HardwareSerial.h"

int16_t adc_value;
uint8_t radio_data[RF_PAYLOAD_LENGTH];


#define POINT_MORT_MARGIN 16

void pot_to_speed (DCC_timer * timer, uint16_t pot) {
	if (pot > (512 + POINT_MORT_MARGIN)) {
		timer->analog_set_speed(pot - (512 + POINT_MORT_MARGIN) );
		timer->analog_set_direction(forward);
	} else if (pot < (512 - POINT_MORT_MARGIN)) {
		timer->analog_set_speed(512 - POINT_MORT_MARGIN - pot);
		timer->analog_set_direction(backward);
	} else {
		timer->analog_set_speed(0);
		timer->analog_set_direction(off);
	}
}

int main(void) {
	uint8_t status,ack;
	uint8_t count;
	uint16_t speed;

	// see config.h
	DDRB=PORTB_DIRECTION;
	DDRC=PORTC_DIRECTION;
	DDRD=PORTD_DIRECTION;
	//set output I/O to 1, Input to no pull-up
	PORTB = PORTB_DIRECTION;
	PORTD = PORTD_DIRECTION;


	Serial.begin(115200);
	init_timer0_tick();
	// enable interrupts
	__builtin_avr_sei ();


	radio_pl_init_prx();
	Serial.write('>');

	while (1) {
		CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver
		status = hal_nrf_get_status();
		if ((status & (1<<HAL_NRF_RX_DR)) != 0) { // a packet is available
			// get it
			count = hal_nrf_read_reg(R_RX_PL_WID);
			hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
			// clear IRQ source
			hal_nrf_get_clear_irq_flags();
#ifdef DEBUG
			Serial.write('.');
#endif
			switch (radio_data[0]) {
			case 3:
				speed = (radio_data[3] << 8) + radio_data[4];
				pot_to_speed(&timer1, speed);
				break;
//				hal_nrf_write_lcd_pload(ack_pipe, line , lcd->get_next_line(), 13);
			/* Ignore other stuff */
			default:
				break;
			}
		} else if ((status & (1<<HAL_NRF_MAX_RT)) != 0 ) { // Max Retry, flush TX
			hal_nrf_flush_tx(); 		// flush tx fifo, avoid fifo jam
			// TO BE CHECKED .... but does not seem to happen ...
		} else {
				// Reprogram radio
				CE_LOW();        // Set Chip Enable (CE) pin low during chip init
				radio_pl_init_prx ();
				Serial.write('R');
		};
		// Packet received and processed, wait for next packet for 1 sec
		set_radio_timeout(250);
		ack = 0;
		while (!check_radio_timeout()) {
			if (radio_activity()) {
				ack=1;
				break;
			}
		}
		if (ack == 0) {
			// No packet received for 1 sec - turn OFF outputs
			timer1.analog_set_speed(0);
			timer1.analog_set_direction(off);
			Serial.write('S');
		}
		// Packet should have been received let's try to see
	}
}
