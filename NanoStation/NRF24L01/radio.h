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

#ifndef RADIO_H__
#define RADIO_H__
 
 /** @file
 * @ingroup main
 * Radio header file for the nRF24LU1 example application
 * 
 * @author Per Kristian Schanke
 */

/** Defines the channel the radio should operate on*/
#define RF_CHANNEL 72

/** Defines the time it takes for the radio to come up to operational mode */
#define RF_POWER_UP_DELAY 2

/** Defines the payload length the radio should use */
#define RF_PAYLOAD_LENGTH 32

/** Defines how many retransmits that should be performed */
#define RF_RETRANSMITS 1

/** Defines the retransmit delay. Should be a multiple of 250. If the 
 * RF_PAYLOAD_LENGTH is larger than 18, a higher retransmit delay need to
 * be set. This is because both the original package and ACK payload will
 * be of this size. When the ACK payload exceeds 18 byte, it will not be able
 * to receive the full ACK in the ordinary 250 microseconds, so the delay
 * will need to be increased. */
#if (RF_PAYLOAD_LENGTH <= 18)
#define RF_RETRANS_DELAY 250
#else
#define RF_RETRANS_DELAY 500
#endif

extern const uint8_t NRF_address[];

/** Enumerates the different states the radio may
 * be in.
 */
typedef enum {
  RF_IDLE,    /**< Radio is idle */
  RF_MAX_RT,  /**< Maximum number of retries have occurred */
  RF_TX_DS,   /**< Data is sent */
  RF_RX_DR,   /**< Data received */
  RF_TX_AP,   /**< Ack payload received */
  RF_BUSY     /**< Radio is busy */
} radio_status_t;

/** This function load the data to be sent into the radio, sends it, and waits for
 * the response.
 * @param packet The data to send. Maximum 2 byte
 * @param length The length of the data
*/
void radio_send_packet(uint8_t *packet, uint8_t length);
void radio_send_packet_no_ack(uint8_t *packet, uint8_t length);

/** This function reads the interrupts. It does the work
 * of a interrupt handler by manually reading the interrupt
 * flags and act on them.
 */
//void radio_irq (void);

inline uint8_t radio_activity(void) {
	if ( (PIND & (1<<NRF_IRQ)) == 0)
		return true;
	else
		return false;
}

#endif
