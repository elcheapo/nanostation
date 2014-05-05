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
 * $LastChangedRevision: 2132 $
 */ 

/** @file
 * Header file defining the hardware dependent interface of the Arduino
 *
 *
 */

#ifndef HAL_NRF_HW_H__
#define HAL_NRF_HW_H__

#include "nordic_common.h"
#include "hal_nrf.h"

inline uint8_t hal_nrf_rw(uint8_t value){
	uint8_t data;
	SPDR = value;
	while ((SPSR & (1<<SPIF)) == 0);
	data= SPDR; /* Clear SPIF */
	return data;
}

inline void hal_spi_init(uint32_t spi_speed){
	// Already initialized for LCD
}

// See config.h for hardware definition

/** Macro that set radio's CSN line LOW.
 *
 */
inline void CSN_LOW(void) {
	PORTB &= ~(1<<PB_NRF_CSN);
}

/** Macro that set radio's CSN line HIGH.
 *
 */
inline void CSN_HIGH(void){
	PORTB |= (1<<PB_NRF_CSN);
}

/** Macro that set radio's CE line LOW.
 *
 */
inline void CE_LOW(void) {
	PORTD &= ~(1<<PD_NRF_CE);
}

/** Macro that set radio's CE line HIGH.
 *
 */
inline void CE_HIGH(void) {
	PORTD |= (1<<PD_NRF_CE);
}

/**
 * Pulses the CE to nRF24L01 for at least 10 us
 */
inline void CE_PULSE(void) {
	CE_HIGH();
	__builtin_avr_delay_cycles(250);
	CE_LOW();
}

inline uint8_t hal_nrf_read_reg(uint8_t reg) {
	uint8_t temp;
	CSN_LOW();
	hal_nrf_rw(reg);
	temp = hal_nrf_rw(0);
	CSN_HIGH();
	return temp;
}

inline uint8_t hal_nrf_write_reg(uint8_t reg, uint8_t value) {
	uint8_t retval;
	reg &= 0x1F;
	CSN_LOW();
	retval = hal_nrf_rw(W_REGISTER + reg);
	hal_nrf_rw(value);
	CSN_HIGH();
	return retval;
}

inline uint8_t hal_nrf_nop(void) {
	uint8_t retval;
	CSN_LOW();
	retval = hal_nrf_rw(NOP);
	CSN_HIGH();
	return retval;
}

inline uint8_t hal_nrf_get_status(void) {
	return hal_nrf_nop();
}

inline void hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(reg);
	while(length--) {
		*pbuf++ = hal_nrf_rw(0);
	}
	CSN_HIGH();
}

inline void  hal_nrf_write_multibyte_reg(uint8_t reg, uint8_t const *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(W_REGISTER + reg);
	while(length--) {
		hal_nrf_rw(*pbuf++);
	}
	CSN_HIGH();
}

inline void  hal_nrf_write_tx_payload(uint8_t const *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(W_TX_PAYLOAD);
	while(length--) { hal_nrf_rw(*pbuf++); }
	CSN_HIGH();
}

inline void  hal_nrf_write_tx_payload_no_ack(uint8_t const *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(W_TX_PAYLOAD_NO_ACK);
	while(length--) { hal_nrf_rw(*pbuf++); }
	CSN_HIGH();
}

inline uint8_t hal_nrf_get_clear_irq_flags(void) {
	return hal_nrf_write_reg(STATUS, 0x70);
}

inline void  hal_nrf_write_ack_pload(uint8_t pipe, uint8_t *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(W_ACK_PAYLOAD+pipe);
	while(length--) { hal_nrf_rw(*pbuf++); }
	CSN_HIGH();
}

// optimization for LCD display Packets
inline void  hal_nrf_write_lcd_pload(uint8_t pipe, uint8_t line, uint8_t *pbuf, uint8_t length) {
	CSN_LOW();
	hal_nrf_rw(W_ACK_PAYLOAD+pipe);
	hal_nrf_rw(line);
	length --;
	while(length--) { hal_nrf_rw(*pbuf++); }
	CSN_HIGH();
}
inline void  hal_nrf_reuse_tx(void) {
	CSN_LOW();
	hal_nrf_rw(REUSE_TX_PL);
	CSN_HIGH();
}

inline void  hal_nrf_flush_rx(void) {
	CSN_LOW();
	hal_nrf_rw(FLUSH_RX);
	CSN_HIGH();
}

inline void hal_nrf_flush_tx(void) {
	CSN_LOW();
	hal_nrf_rw(FLUSH_TX);
	CSN_HIGH();
}

#endif /* HAL_NRF_HW_H__ */
