/*
 * main.c
 *
 *  Created on: Mar 1, 2014
 *      Author: francois
 *      Nano Receiver and PWM controller
 *      "NanoStation"
 */


#include "elcheapo_remote.h"
#include "organizer.h"
#include "timer0.h"
#include "dcc_timer.h"

#include "hal_nrf_reg.h"
#include "hal_nrf.h"

#include "radio.h"
#include "HardwareSerial.h"

int16_t adc_value;
uint8_t radio_data[RF_PAYLOAD_LENGTH];
tmode mode;
uint16_t dcc_address_1;
uint16_t dcc_address_2;

#define POINT_MORT_MARGIN 16

void pot_to_speed (uint8_t channel, DCC_timer * timer, uint16_t pot) {
	if (pot > (512 + POINT_MORT_MARGIN)) {
		timer->analog_set_speed(channel, pot - (512 + POINT_MORT_MARGIN) );
		timer->analog_set_direction(channel, forward);
#ifdef DEBUG
		Serial.println(F("FORWARD"));
		Serial.print(F("DDRD="));
		Serial.println(DDRD,16);
		Serial.print(F("PORTD="));
		Serial.println(PORTD,16);
		Serial.print(F("PIND="));
		Serial.println(PIND,16);
#endif

	} else if (pot < (512 - POINT_MORT_MARGIN)) {
		timer->analog_set_speed(channel, 512 - POINT_MORT_MARGIN - pot);
		timer->analog_set_direction(channel, backward);
#ifdef DEBUG
		Serial.println(F("BACKWARD"));
		Serial.print(F("DDRD="));
		Serial.println(DDRD,16);
		Serial.print(F("PORTD="));
		Serial.println(PORTD,16);
		Serial.print(F("PIND="));
		Serial.println(PIND,16);
#endif
	} else {
		timer->analog_set_speed(channel,0);
		timer->analog_set_direction(channel,off);
	}
}

int main(void) {
	uint8_t status,ack,fifo_status;
	uint8_t count;
	uint16_t speed;

	// see config.h
	DDRB=PORTB_DIRECTION;
	DDRC=PORTC_DIRECTION;
	DDRD=PORTD_DIRECTION;
	//set output I/O to 1, Input to no pull-up
	PORTB = PORTB_DIRECTION;
	PORTD = PORTD_DIRECTION;
	ACSR = (1<<ACD) | (1<<ACIE); // Disable analog comparator

	DIDR0 = 0x3F; // Disable digital function input on ADC pîns (PORTC)
	DIDR1 = 0; //do not disable digital function on PD6/PD7

	Serial.begin(115200);
	init_timer0_tick();
	// enable interrupts
	__builtin_avr_sei ();

	dcc_address_1 = 3;
	dcc_address_2 = 4;

	radio_pl_init_prx();
	timer1.end();

	Serial.println(F("Nano Station"));
	Serial.write('>');

	CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver

	// Now decide if we want to do analog (POT to the right > 750) or digital (POT to the left < 250)
	while (1) {
		Serial.write('-');
		status = hal_nrf_get_status();
		fifo_status = hal_nrf_read_reg(FIFO_STATUS);
		if ((fifo_status & 0x01) == 0) { // a packet is available
			// get it
			count = hal_nrf_read_reg(R_RX_PL_WID);
			if (count != 0) {
				hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
				// clear IRQ source
				hal_nrf_get_clear_irq_flags();
				Serial.write('#');
			}
			speed = (radio_data[3] << 8) + radio_data[4];
			if (speed > 750 ) {
				mode = analog;
				break;
			}
			if (speed < 250 ){
				mode = digital;
				break;
			}
		}
		set_radio_timeout(1000/4);
		while (!check_radio_timeout()) {
			if (radio_activity()) {
				break;
			}
		}
	}

	if (mode == digital) {
		Serial.println(F("Digital"));

		timer1.begin(digital);
		timer1.digital_on(CHANNEL_1);
		timer1.digital_on(CHANNEL_2);
		new_loco(dcc_address_1);
		new_loco(dcc_address_2);
		//Switch on lights
		do_loco_func_grp0(dcc_address_1,1);
		do_loco_func_grp0(dcc_address_2,1);

		// Start in digital , send 30 RESET so the decoder switches to digital
		DCC_Reset.repeat = 30;
		timer1.send_dcc_packet(&DCC_Reset);
		DCC_Reset.repeat = 1;


		while (1) {
			status = hal_nrf_get_status();
			fifo_status = hal_nrf_read_reg(FIFO_STATUS);
			if ((fifo_status & 0x01) == 0) { // a packet is available
				// get it
				count = hal_nrf_read_reg(R_RX_PL_WID);
				if (count != 0) {
					hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
					// clear IRQ source
					hal_nrf_get_clear_irq_flags();
					switch (radio_data[0]) {
					/*
  				case 1:
				speed = (radio_data[11] << 8) + radio_data[12];
				pot_to_speed(&timer1, speed);
				break;
					 */
					case 2:
						// in case we turned off the output after radio link loss
						timer1.digital_on(CHANNEL_1);
						timer1.digital_on(CHANNEL_2);
						speed = (radio_data[3] << 8) + radio_data[4];
						if (speed > 512) {
							do_loco_speed(dcc_address_1, (speed - 512) / 4);
						} else {
							do_loco_speed(dcc_address_1, 0x80 | ((512 - speed) / 4) );
						}
						speed = (radio_data[5] << 8) + radio_data[6];
						if (speed > 512) {
							do_loco_speed(dcc_address_2, (speed - 512) / 4);
						} else {
							do_loco_speed(dcc_address_2, 0x80 | ((512 - speed) / 4) );
						}
						break;
					default:
						break;
					}
				}
			} else if ((status & (1<<HAL_NRF_MAX_RT)) != 0 ) { // Max Retry, flush TX
				hal_nrf_flush_tx(); 		// flush tx fifo, avoid fifo jam
				// TO BE CHECKED .... but does not seem to happen ...
			};
#if 0
			else {
				// Reprogram radio
				CE_LOW();        // Set Chip Enable (CE) pin low during chip init
				radio_pl_init_prx ();
				Serial.write('R');
			};
#endif


			// Do we have another packet ?
			fifo_status = hal_nrf_read_reg(FIFO_STATUS);
			if ((fifo_status &0x01) == 0) continue;
			// Packet received and processed, wait for next packet for 1 sec
			set_radio_timeout(1000/4);
			ack = 0;
			while (!check_radio_timeout()) {
				/* Now see if we need to send the next packet ... */
				run_organizer();
				/* otherwise, do we have someting going on for the radio */
				if (radio_activity()) {
					ack=1;
					Serial.println(F("#"));
					break;
				}
			}
			if (ack == 0) {
				// No packet received for 1 sec - turn OFF outputs
				timer1.digital_off(CHANNEL_1);
				timer1.digital_off(CHANNEL_2);
				Serial.write('S');
			}
		}
		// Packet should have been received let's try to see
	} else { // analog
		Serial.println(F("Analog"));
		timer1.begin(analog);
		while (1) {
			status = hal_nrf_get_status();

			fifo_status = hal_nrf_read_reg(FIFO_STATUS);
#ifdef DEBUG
			Serial.print(F("St:"));
			Serial.println(status,16);
			Serial.print(F("Fi:"));
			Serial.println(fifo_status,16);
#endif
			//		if ((status & (1<<HAL_NRF_RX_DR)) != 0) { // a packet is available
			if ((fifo_status & 0x01) == 0) { // a packet is available
				// get it
				count = hal_nrf_read_reg(R_RX_PL_WID);
				if (count != 0) {
					hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
					// clear IRQ source
					hal_nrf_get_clear_irq_flags();
#ifdef DEBUG
					Serial.write('.');
					for (uint8_t i=0; i<8; i++) {
						Serial.write(' ');
						Serial.print(radio_data[i],16);
					}
					Serial.println();
#endif
					switch (radio_data[0]) {
					/*				case 1:
					speed = (radio_data[11] << 8) + radio_data[12];
					pot_to_speed(&timer1, speed);
					break;
					 */
					case 2:
						speed = (radio_data[3] << 8) + radio_data[4];
						pot_to_speed(CHANNEL_1, &timer1, speed);
						speed = (radio_data[5] << 8) + radio_data[6];
						pot_to_speed(CHANNEL_2, &timer1, speed);
						break;
						//				hal_nrf_write_lcd_pload(ack_pipe, line , lcd->get_next_line(), 13);
						/* Ignore other stuff */
					default:
						break;
					}
				}
			} else if ((status & (1<<HAL_NRF_MAX_RT)) != 0 ) { // Max Retry, flush TX
				hal_nrf_flush_tx(); 		// flush tx fifo, avoid fifo jam
				// TO BE CHECKED .... but does not seem to happen ...
			};
#if 0
			else {
				// Reprogram radio
				CE_LOW();        // Set Chip Enable (CE) pin low during chip init
				radio_pl_init_prx ();
				Serial.write('R');
			};
#endif
			// Do we have another packet ?
			fifo_status = hal_nrf_read_reg(FIFO_STATUS);
			if ((fifo_status &0x01) == 0) continue;
			// Packet received and processed, wait for next packet for 1 sec
			set_radio_timeout(1000/4);
			ack = 0;
			while (!check_radio_timeout()) {
				if (radio_activity()) {
					ack=1;
					Serial.println(F("#"));
					break;
				}
			}
			if (ack == 0) {
				// No packet received for 1 sec - turn OFF outputs
				timer1.analog_set_speed(CHANNEL_1,512);
				timer1.analog_set_direction(CHANNEL_1,off);
				timer1.analog_set_speed(CHANNEL_2,512);
				timer1.analog_set_direction(CHANNEL_2,off);

				Serial.write('S');
			}
			// Packet should have been received let's try to see
		}
	}
}
