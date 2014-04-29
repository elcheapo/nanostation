/*
 * 	El Cheapo DCC command station 
 * 
 * 	Copyright (c) 2006 Wolfgang Kufer (kufer@gmx.de)
 * 	Copyright (c) 2012 Francois Lorrain (francois.lorrain@gmail.com)
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
 * File : 
 * Author :
 * Purpose :
 * Upstream interface :
 *  
*/
/* Scheduler include files. */
//#include <FreeRTOS.h>
//#include <task.h>
//#include <queue.h>
//#include <semphr.h>
//#include <timers.h>

#include <Arduino.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/builtins.h>
#define nop()  __asm__ __volatile__("nop")
#include "config.h"

