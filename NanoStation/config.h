/*

 * 	El Cheapo Remote control unit
 *
  * 	Copyright (c) 2013 Francois Lorrain (francois.lorrain@gmail.com)
 *
 * This source file is subject of the GNU general public license 2 or later,
 * available on the web at : http://www.gnu.org/licenses/gpl.txt
 *
 * This is meant to be a fun project to construct you own DCC command station
 * with an Arduino board and some standard boards.
 * The code is freely available under the GPL so it can be easily customized
 * to your own needs.
 * Hopefully it contains some great ideas, and you are welcome to add your own ...
 *
 *
 */

/*
 * File : config.h
 * Purpose : All global definitions and hardware related defines
 *
*/
//-----------------------------------------------------------------
#ifndef __CONFIG_H__
#define __CONFIG_H__


//========================================================================
// 1. Processor Definitions
//========================================================================
//
// Nano Station uses a ATMega328P on an arduino nano


#undef DEBUG

//========================================================================
// 2. Port Definitions
//========================================================================
// Port A

// Port B

//#define PortB0			0		// out
#define PB_NRF_CSN		0		// out
#define PB1_OC1A 		1		// out
//#define PB_NRF_CSN		2		// out
#define PB2_OC1B		2		// out
#define MOSI			3		// out
#define MISO			4		// in
#define SCK				5		// out
#define PortB6			6		// out (XTAL1)
#define PortB7			7		// out (XTAL2)

#define PORTB_DIRECTION	((uint8_t)0xef)   	// Output PINs for PORTB


// Port C
#define ADC0		0
//#define ADC1 		1
#define PC1_POT		1		// out
#define ADC2		2
//#define ADC3		3
#define PC3_POT		3		// out
#define SDA			4
#define SCL			5
#define ADC6		6
#define	ADC7		7


#define PORTC_DIRECTION	((uint8_t)0x0A)   	// Output PINs for PORTC

// Port D
#define PortD0			0		// in (RX) - Buzzer out
#define PortD1 			1		// out (TX)
#define PD_NRF_IRQ		2		// in (INT0)
#define PD_NRF_CE		3		// out
#define PD_L298_IN3		4		// out
#define PD_L298_IN4		5		// out
#define PD_L298_IN1		6		// out
#define PD_L298_IN2		7		// out

#define PORTD_DIRECTION	((uint8_t)0xfb)   	// Output PINs for PORTD

//========================================================================
// 3. System Definitions
//========================================================================

#ifndef FALSE
#define FALSE       0
#endif
#ifndef TRUE
#define TRUE        (!FALSE)
#endif

#define OK TRUE
#define NOK FALSE

typedef enum {dcc_off, analog, digital} tmode;
typedef enum {off, forward, backward} tdirection;


#define CHANNEL_1 0
#define CHANNEL_2 1


//------------------------------------------------------------------------
// define a structure for DCC messages inside OpenDCC
//------------------------------------------------------------------------
#define   MAX_DCC_SIZE  5

typedef enum {is_void,
              is_stop,
              is_loco,
              is_acc,
              is_feedback,
              is_prog,
              is_prog_ack}  tmessage;


#define NUM_DCC_SPEED_REPEAT   2        // Speed commands are repeated this number
#define NUM_DCC_ACC_REPEAT     2        // Accessory Commands are repeated this number
#define NUM_DCC_FUNC_REPEAT    1        // Function Commands are repeated this number
#define NUM_DCC_POM_REPEAT     3        // Program on the main are repeated this number

#define SIZE_LOCOBUFFER      4       // no of simult. active locos (6 bytes each entry)


#endif   // config.h
