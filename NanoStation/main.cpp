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

// Re-initialize radio after n Seconds
#define MAX_TIMEOUTS 5

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
	uint8_t status;
	uint8_t count;
	uint16_t speed;
	bool powered;

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

	powered = false;

	dcc_address_1 = 3;
	dcc_address_2 = 4;

	timer1.end();

	Serial.println();
	Serial.println(F("Nano Station"));
	//	Serial.write('>');

	status = radio_pl_init_prx();
	if ((status & 0x80) == 0) {
		Serial.println(F("Radio OK"));
	} else {
		while (1);
		// If higher bit of radio status is not 0 - we have a wiring issue ...
	}
	CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver
	set_radio_timeout(5000/4);
	// Now decide if we want to do analog (POT to the right > 750) or digital (POT to the left < 250)
	while (1) {
		status = radio_get_packet(radio_data, &count);
		if (status == OK) {
			Serial.write('R');
			set_radio_timeout(5000/4);
			if (radio_data[0]==2) {
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
		} else { // Timeout receiving radio packet, re-initialize radio
			if (check_radio_timeout()) { // Re-initialize radio if we did not get anything after 5 sec...
				Serial.write('T');
				CE_LOW();
				radio_pl_init_prx();
				CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver
				set_radio_timeout(5000/4);
			}

		};
	}

	if (mode == digital) {
		Serial.println(F("Digital"));
		new_loco(dcc_address_1);
		new_loco(dcc_address_2);
	} else { // analog
		Serial.println(F("Analog"));
		timer1.begin(analog);
	}
	set_radio_timeout(1000/4);

	while (1) {
		status = radio_get_packet(radio_data, &count);
		if (status == OK ) {
			set_radio_timeout(1000/4);
			switch (radio_data[0]) {
			/*
			case 1:
			speed = (radio_data[11] << 8) + radio_data[12];
			pot_to_speed(&timer1, speed);
			break;
			 */
			case 2:
				if (mode == digital) {
					// in case we turned off the output after radio link loss
					if (powered == false) {
						timer1.begin(digital);
						timer1.digital_on(CHANNEL_1);
						timer1.digital_on(CHANNEL_2);
						//Switch on lights
						do_loco_func_grp0(dcc_address_1,1);
						do_loco_func_grp0(dcc_address_2,1);

						// Start in digital , send 30 RESET so the decoder switches to digital
						DCC_Reset.repeat = 30;
						timer1.send_dcc_packet(&DCC_Reset);
						DCC_Reset.repeat = 1;
						powered = true;
					}
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
				} else { // analog
					if (powered == false) {
						timer1.begin(analog);
						powered = true;
					}
					speed = (radio_data[3] << 8) + radio_data[4];
					pot_to_speed(CHANNEL_1, &timer1, speed);
					speed = (radio_data[5] << 8) + radio_data[6];
					pot_to_speed(CHANNEL_2, &timer1, speed);
				}
				break;
			default:
				break;
			}
		} else { // no radio packet
			if (mode == digital) {
				run_organizer();
			}
			if (check_radio_timeout()) { // Re-initialize radio if we did not get anything after 1 sec...
				Serial.write('T');
				CE_LOW();
				radio_pl_init_prx();
				CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver
				// No packet received for 1 sec - turn OFF outputs
				if (mode == digital) {
					timer1.digital_off(CHANNEL_1);
					timer1.digital_off(CHANNEL_2);
				} else { //  analog
					timer1.analog_set_speed(CHANNEL_1,512);
					timer1.analog_set_direction(CHANNEL_1,off);
					timer1.analog_set_speed(CHANNEL_2,512);
					timer1.analog_set_direction(CHANNEL_2,off);

				}
				powered = false;
				set_radio_timeout(5000/4);
			}
		};
	} // While
}
