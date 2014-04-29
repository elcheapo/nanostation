/*
 * main.c
 *
 *  Created on: Mar 31, 2013
 *      Author: francois
 */


#include "elcheapo_remote.h"
#include "i2c_keyboard.h"
#include "nokia5510.h"
#include "TWI_Master.h"
#include "adc.h"
#include "timer0.h"

#include "hal_nrf_reg.h"
#include "hal_nrf.h"

#include "radio.h"
#include "radio_pl.h"

int16_t adc_value;
uint8_t radio_data[RF_PAYLOAD_LENGTH];
#ifdef DEBUG
uint8_t lcd_update;
#endif


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

	lcd_reset();
	lcd1.begin();
	TWI_Master_Initialise();
	adc_init();
	init_timer0_tick();
	// enable interrupts
	__builtin_avr_sei ();

	lcd_update = true;

	lcd1.clear();
	lcd1.go(0,1);
	//            123456789012
	lcd1.print(F("  EL CHEAPO "));
	lcd1.go(0,2);
#ifdef RADIO1
	lcd1.print(F("Remote Ctrl1"));
#endif
#ifdef RADIO2
	lcd1.print(F("Remote Ctrl2"));
#endif
	lcd1.update();
	connected=false;
	radio_pl_init (NRF_address, HAL_NRF_PTX);

	while (1) {
		lcd1.update();
#ifdef RADIO1
		set_loop_time(80/4);
#endif
#ifdef RADIO2
		set_loop_time(90/4);
#endif
		kbd1.scan();

		if (!connected) {
			key = kbd1.get_key();
			if (key != 0) {
				lcd1.go(5,5);
				lcd1.write(key);
				lcd1.pseudo_led(0,0);
				if (key == 'A') lcd_update = !lcd_update;
			} else {
				lcd1.pseudo_led(0,1);
			}
		}

		radio_data[0] = 1; // Normal LCD + ADC packet

		radio_data[1]=kbd1.get_column(0);
		radio_data[2]=kbd1.get_column(1);
		radio_data[3]=kbd1.get_column(2);
		radio_data[4]=kbd1.get_column(3);

		adc_value = get_adc(0);
		radio_data[5]=adc_value >> 8;
		radio_data[6]=adc_value & 0xff;
		if (!connected) {
			lcd1.go(5,3); lcd1.print(F("     "));lcd1.go(5,3); lcd1.print(adc_value,10);
		}

		adc_value = get_adc(1);
		radio_data[7]=adc_value >> 8;
		radio_data[8]=adc_value & 0xff;
		if (!connected) {
			lcd1.go(0,3); lcd1.print(F("     "));lcd1.go(0,3); lcd1.print(adc_value,10);
		}

		adc_value = get_adc(6);
		radio_data[9]=adc_value >> 8;
		radio_data[10]=adc_value & 0xff;
		if (!connected) {
			lcd1.go(5,4); lcd1.print(F("     "));lcd1.go(5,4); lcd1.print(adc_value,10);
		}

		adc_value = get_adc(7);
		radio_data[11]=adc_value >> 8;
		radio_data[12]=adc_value & 0xff;
		if (!connected) {
			lcd1.go(0,4); lcd1.print(F("     "));lcd1.go(0,4); lcd1.print(adc_value,10);
		}

		if (lcd_update == 0)
			lcd1.update();

		hal_nrf_get_clear_irq_flags();
		radio_send_packet(radio_data, 13);
		set_timeout(70);
		lcd1.pseudo_led(9,0);
		extra_packet=false;

		ready_to_receive:
		while (!radio_activity()) {
			if (check_timeout()) break;
		};
		status = hal_nrf_get_status();
		switch (status & 0x70) {
		case (1<<HAL_NRF_TX_DS):
			/* Tx packet sent, ack received but no ack packet payload ... */
			hal_nrf_get_clear_irq_flags();
			/* Sent right away a quick packet to try to get the ACK payload but only once*/
			if (!extra_packet) {
				extra_packet = true;
				radio_data[0] = 0;
				radio_send_packet(radio_data, 1);
				goto ready_to_receive;
			}
			break;
		case ((1<<HAL_NRF_TX_DS)|(1<<HAL_NRF_RX_DR)):
			/* Tx done, Ack packet received */
			// get it
			count = hal_nrf_read_reg(R_RX_PL_WID);
			hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
			// clear IRQ source
			hal_nrf_get_clear_irq_flags();
			connected = true;
			lcd1.pseudo_led(9,1);
			switch (radio_data[0]) {
			case 0x01: /* LCD data */
				key = radio_data[1];
				if ((key > 0) && (key < 6)) {
					lcd1.go(0,key);
					for (uint8_t i = 2; i<14; i++)
						lcd1.write(radio_data[i]);
				} else {
					lcd1.go(0,1);
					lcd1.print(F("Invalid Pkt"));
				}
				break;
			case 0x02: /* Pseudo_led data */
				for (i=0; i<8; i++)
					lcd1.pseudo_led(i, radio_data[i+2]);
				break;
			default:
				/* Ignore junk */
				break;
			}
			break;
		case (1<<HAL_NRF_MAX_RT):
			hal_nrf_get_clear_irq_flags();
			/* Max Retry reached */
			hal_nrf_flush_tx(); 						// flush tx fifo, avoid fifo jam
			lcd1.go(0,1);
			lcd1.print(F("NO ANSWER   "));
			connected = false;
			break;
		default:
			hal_nrf_get_clear_irq_flags();
			break;
		}
		/* don't resend a packet too fast */
		while (!check_loop_time());
	}
}

