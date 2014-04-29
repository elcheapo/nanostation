/*
 * main.c
 *
 *  Created on: Mar 31, 2013
 *      Author: francois
 */


#include "elcheapo_remote.h"
#include "timer0.h"

#include "hal_nrf_reg.h"
#include "hal_nrf.h"

#include "radio.h"
#include "radio_pl.h"

int16_t adc_value;
uint8_t radio_data[RF_PAYLOAD_LENGTH];



int main(void) {
	uint8_t key,i;
	bool connected;
	uint8_t status;
	uint8_t count;
	bool lcd_update;
	/* lcd_update can be disabled to monitor only SPI NRF traffic */
	bool extra_packet;

	// see config.h
	DDRB=PORTB_DIRECTION;
	DDRC=PORTC_DIRECTION;
	DDRD=PORTD_DIRECTION;
	//set output I/O to 1, Input to no pull-up
	PORTB = PORTB_DIRECTION;
	PORTD = PORTD_DIRECTION;
	// set CE low (transmit packet when CE pulsed high)
	CE_LOW();

	lcd = &radio_lcd2;
//	line = lcd->next_line();
	hal_nrf_write_lcd_pload(2, 1, lcd->get_next_line(), 13);

	while (1) {
		CE_HIGH();        // Set Chip Enable (CE) pin high to enable receiver
		status = hal_nrf_get_status();
		if ((status & 0x0e) != 0x0e) { // a packet is available
			// get it
			count = hal_nrf_read_reg(R_RX_PL_WID);
			hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, pload, count);
			// clear IRQ source
			hal_nrf_get_clear_irq_flags();
			EIMSK |= (1<<INT7); // Re-enable level sensitive interrupt

			switch ((status & 0x0e) >> 1) {
			case 1: // Radio1 H/W on channel 1
				kbd=&radio_kbd1;
				lcd=&radio_lcd1;
				/* increment radio_ok counter by 2 */
				if (kbd->radio_ok != NOT_PRESENT) kbd->radio_ok = NOT_PRESENT;
				/* decrement other radio count */
				if (radio_kbd2.radio_ok != 0) radio_kbd2.radio_ok--;
				ack_pipe = 1;
#ifdef DEBUG
				Serial3.write('a');
#endif
				break;
			case 2: // Radio2 H/W on channel 2
				kbd=&radio_kbd2;
				lcd=&radio_lcd2;
				/* increment radio_ok counter by 2 */
				if (kbd->radio_ok != NOT_PRESENT) kbd->radio_ok = NOT_PRESENT;
				/* decrement other radio count */
				if (radio_kbd1.radio_ok != 0) radio_kbd1.radio_ok--;
				ack_pipe = 2;
#ifdef DEBUG
				Serial3.write('b');
#endif
				break;
			default:
#ifdef DEBUG
				Serial3.write('M');
				Serial3.println(status,16);
#endif
				continue;
				break;
			}
			switch (pload[0]) {
			case 0:
				/* Just asking for an ack packet cuz sender didn't get one ... */
				break;
			case 1:
				/* Normal kbd/adc packet */
				kbd->scan_input(pload+1);
				kbd->pot[0]=(pload[5]<<8)+pload[6];
				kbd->pot[1]=(pload[7]<<8)+pload[8];
				kbd->pot[2]=(pload[9]<<8)+pload[10];
				kbd->pot[3]=(pload[11]<<8)+pload[12];
				// Now returns led / lines to display
				if ((status & 0x01) == 0) { // Make sure there is space in TX FIFO
				line = lcd->next_line();
				if (line == 0) {
					// send pseudo LED data
					hal_nrf_write_led_pload(ack_pipe, lcd->get_pseudo_led(), 11);
				} else  {
					hal_nrf_write_lcd_pload(ack_pipe, line , lcd->get_next_line(), 13);
				}
				}
				break;
			/* Ignore other stuff */
			default:
				break;
			}
		} else if ((status & (1<<HAL_NRF_MAX_RT)) != 0 ) { // Max Retry, flush TX
			hal_nrf_flush_tx(); 		// flush tx fifo, avoid fifo jam
			// TO BE CHECKED .... but does not seem to happen ...
#ifdef DEBUG
			Serial3.write('F');
#endif
		} else { // Wait for next packet for 2000 ms max
			if (xSemaphoreTake(radio_done,2000 / portTICK_RATE_MS ) != pdTRUE) {
				// Timeout on Radio reception
				radio_kbd1.radio_ok = false;
				radio_kbd2.radio_ok = false;
#ifdef DEBUG1
				Serial3.print(F("ST:"));Serial3.println(hal_nrf_get_status(),16);
#endif
				// Reprogram radio
				CE_LOW();        // Set Chip Enable (CE) pin low during chip init
				radio_pl_init (HAL_NRF_PRX);
				// and wait
#ifdef DEBUG
				Serial3.write('Z');
#endif
			} else {
				// Signal received from INT7
#ifdef DEBUG1
				Serial3.write('i');
#endif
			}
		}
	} // while
}
		/* don't resend a packet too fast */
		while (!check_loop_time());
	}
}

