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
 * $LastChangedRevision: 2310 $
 */ 

/** @file
 * @ingroup Main
 * Radio functions.
 *
 * This file handles all radio communication for the example application, i.e. 
 * radio_init, radio_send_packet and radio_interrupt function.
 *
 * @author Per Kristian Schanke
 */

#include "elcheapo_remote.h"

#include "hal_nrf.h"
#include "radio.h"

/** The address of the radio. Parameter to the radio init */
#ifdef RADIO1
const uint8_t NRF_address[HAL_NRF_AW_5BYTES] = {0xa5,'h','c','l','E'};
#endif
#ifdef RADIO2
const uint8_t NRF_address[HAL_NRF_AW_5BYTES] = {0x55,'h','c','l','E'};
#endif

void radio_send_packet(uint8_t *packet, uint8_t length) {
	hal_nrf_write_tx_payload(packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
/*
void radio_send_packet_no_ack(uint8_t *packet, uint8_t length) {
	hal_nrf_write_multibyte_reg(W_TX_PAYLOAD_NO_ACK, packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
*/
