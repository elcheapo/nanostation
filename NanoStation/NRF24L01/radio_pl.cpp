/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2185 $
 */ 

/** @ingroup PL 
 * @file
 * Initialise the radio in Enhanced ShockBurst mode with Bidirectional data. 
 * This is done by opening @b pipe0 with auto ACK and with auto retransmits. 
 * It also opens for the use of ACK payload (@b hal_nrf_enable_ack_pl()) and 
 * dynamic payload width (@b hal_nrf_enable_dynamic_pl() for general enabling
 * and @b hal_nrf_setup_dyn_pl() to enable on specific pipes).
 *
 * @author Per Kristian Schanke
 */

#include "elcheapo_remote.h"

#include "hal_nrf.h"
#include "radio_pl.h"
#include "radio.h"
#include "timer0.h"

#define ACK_PL
#define AUTO_ACK

void radio_pl_init (const uint8_t *address, hal_nrf_operation_mode_t operational_mode) {

#ifdef DEBUG
	uint8_t buffer[6];
#endif

	CE_LOW();        // Set Chip Enable (CE) pin low during chip init

	hal_nrf_write_reg(EN_RXADDR, 0);	 // First close all radio pipes
	hal_nrf_write_reg(EN_AA, 0);

	// Pipe 0, 1 open with autoack
	hal_nrf_write_reg(EN_RXADDR, 0x03);
	hal_nrf_write_reg(EN_AA, 0x03);

	hal_nrf_write_reg(SETUP_AW, HAL_NRF_AW_5BYTES - 2); // 5 bytes address width
	hal_nrf_write_reg(SETUP_RETR, (((RF_RETRANS_DELAY/250)-1)<<4) | RF_RETRANSMITS);
	hal_nrf_write_reg(RF_CH, RF_CHANNEL);
	// Frequency = 2400 + RF_CHANNEL
	hal_nrf_write_reg(RF_SETUP, 0x0e) ;
	//2 Mbits - not test PLL - 0dBm - default settings

	// Write addresses LSB first
	hal_nrf_write_multibyte_reg(HAL_NRF_PIPE0, address, HAL_NRF_AW_5BYTES);
	hal_nrf_write_multibyte_reg(HAL_NRF_TX, address, HAL_NRF_AW_5BYTES);
	hal_nrf_write_reg(RX_PW_P0, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P1, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P2, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P3, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P4, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P5, RF_PAYLOAD_LENGTH);
//	hal_nrf_lock_unlock ();                 // Activate features
	hal_nrf_write_reg(DYNPD, 0x3f);			// Sets up dynamic payload on all data pipes.
	hal_nrf_write_reg(FEATURE, 0x06);  // Enable dynamic payload, enable ack payload
	if(operational_mode == HAL_NRF_PTX) {           // Mode dependent settings
		hal_nrf_write_reg(CONFIG, 0b00001110);
		// IRQ on, EN_CRC, 2 bytes CRC, PWR UP, PTX
	} else {
		hal_nrf_write_reg(CONFIG, 0b00001111);
		// IRQ on, EN_CRC, 2 bytes CRC, PWR UP, PRX
	}
	hal_nrf_write_reg(STATUS, 0x70);
	// Clear pending IRQ
#ifdef DEBUG
	Serial.print(F("EN_RXADDR = 0x"));
	Serial.println(hal_nrf_read_reg(EN_RXADDR),16);
	Serial.print(F("EN_AA = 0x"));
	Serial.println(hal_nrf_read_reg(EN_AA),16);
	Serial.print(F("SETUP_RETR = 0x"));
	Serial.println(hal_nrf_read_reg(SETUP_RETR),16);
	Serial.print(F("RF_CH = "));
	Serial.println(hal_nrf_read_reg(RF_CH),10);
	Serial.print(F("RF_SETUP = 0x"));
	Serial.println(hal_nrf_read_reg(RF_SETUP),16);
	Serial.print(F("DYNPD = 0x"));
	Serial.println(hal_nrf_read_reg(DYNPD),16);
	Serial.print(F("FEATURE = 0x"));
	Serial.println(hal_nrf_read_reg(FEATURE),16);
	Serial.print(F("ADR0 = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_PIPE0, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("ADR1 = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_PIPE1, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("ADR2 = "));
	Serial.println(hal_nrf_read_reg(HAL_NRF_PIPE2),16);
	Serial.print(F("TXADR = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_TX, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("CONFIG = 0x"));
	Serial.println(hal_nrf_read_reg(CONFIG),16);
#endif

	wait_tempo(2);

	// start_timer(RF_POWER_UP_DELAY);                // Wait for the radio to
	// wait_for_timer();                              // power up

//	radio_set_status (RF_IDLE);                    // Radio now ready
}    
