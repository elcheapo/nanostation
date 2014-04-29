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
// El cheapo remote uses a seeeduino with the ATMega328P

//#define RADIO1 Set in project properties
//#define RADIO2

#if defined(RADIO1) & defined(RADIO2)
#error only one of RADIO1 or RADIO2 must be defined
#endif

#if  !defined(RADIO1) & !defined(RADIO2)
#error one of RADIO1 or RADIO2 must be defined
#endif

#undef DEBUG

//========================================================================
// 2. Port Definitions
//========================================================================
// Port A

// Port B

#define LCD_D_C		0		// out
#define LCD_RST 	1		// out
#define LCD_SCE		2		// out
#define MOSI		3		// out
#define MISO		4		// in
#define SCK			5		// out
#define PortB6		6		// out
#define PortB7		7		// out

#define PORTB_DIRECTION	((uint8_t)0xef)   	// Output PINs for PORTB


// Port C
#define ADC0		0
#define ADC1 		1
#define ADC2		2
#define ADC3		3
#define SDA			4
#define SCL			5
#define ADC6		6
#define	ADC7		7


#define PORTC_DIRECTION	((uint8_t)0x00)   	// Output PINs for PORTC

// Port D
#define PortD0		0		// out
#define PortD1 		1		// out
#define NRF_IRQ		2		// in
#define NRF_CSN		3		// out
#define NRF_CE		4		// out
#define PortD5		5		// out
#define RETRO		6		// out (OC0A
#define PortD7		7		// out

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

#endif   // config.h
