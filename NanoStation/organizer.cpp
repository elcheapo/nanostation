#include "elcheapo_remote.h"
//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006, 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      organizer.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de

#undef DEBUG_UART3


#include "organizer.h"
#include "dcc_timer.h"



//#include "programmer.h"


/* this queue contains the messages to transmit at hand, will be filled
 * out of the locobuffer and whenever there is a modification, then we
 * send the message rapidly
 */

// Replace with FreeRTOS equivalent queues
// message message_queue[SIZE_MESSAGE_QUEUE];

locomem locobuffer[SIZE_LOCOBUFFER];
uint8_t current_loco;
uint8_t current_level;
uint8_t send_BC_flag; // if 0: no BC, if 1: send STOP,( if 2 send BRAKE)
// ---------------------------------------------------------------------
// predefined messages
//
// stored in bss, copied at start to sram
//                     {rep, size, type, data}
message DCC_Reset    = {1,   2,    is_void, {0x00, 0x00}};    // DCC-Reset-Paket
message DCC_Idle     = {1,   2,    is_void, {0xFF, 0x00}};    // DCC-Idle-Paket
message DCC_BC_Stop  = {1,   2,    is_stop, {0x00, 0x71}};    // Broadcast Motor off:
// 01DC000S :D=x, C=1 (ignore D)
message DCC_BC_Brake = {1,   2,    is_stop, {0x00, 0x70}};    // Broadcast Slow down
// if S=0: slow down
message DCC_Factory_Reset = {1, 3, is_prog, {0x7F, 0x08, 0x77}};



//--------------------------------- routines to convert speed from and to DCC14 and DCC28


uint8_t convert_speed_to_rail(uint8_t speed128, t_format format)
{
	uint8_t retval, myspeed, direction;
	retval = speed128;             // by default
	myspeed = speed128 & 0x7F;    // mask direction
	direction = speed128 & 0x80;

	switch(format) {
	case DCC14:
		if (myspeed > 1) {
			retval = (myspeed - 2) / 9 + 2;
			retval |= direction;
		}
		break;
	case DCC27: // !!! not implemented -> same as DCC28.
	case DCC28:
		if (myspeed > 1) {
			retval = (myspeed - 2) * 2 / 9 + 2;
			retval |= direction;
		}
		break;
	case DCC128:
		break;
	}
	return(retval);
}


uint8_t convert_speed_from_rail(uint8_t speed, t_format format)
{
	uint8_t retval, myspeed, direction;
	retval = speed;             // by default
	myspeed = speed & 0x7F;    // mask direction
	direction = speed & 0x80;

	switch(format)
	{
	case DCC14:
		if (myspeed > 1) {
			retval = (myspeed - 2) * 9 + 2;
			retval |= direction;
		}
		break;
	case DCC27: // !!! not implemented -> same as DCC28.
	case DCC28:
		if (myspeed > 1) {
			retval = ((myspeed - 2) * 9 + 1 )/ 2 + 2;
			retval |= direction;
		}
		break;
	case DCC128:
		break;
	}
	return(retval);
}


//=====================================================================================
//--------------------------------- routines to build DCC messages - see NMRA RP 9.2.1


/// build short address and 14 speed steps; neg. speed = forward, pos speed = revers
/// 0AAAAAAA 01DUSSSS
///
void build_loko_14s(uint16_t nr, uint8_t speed, message * new_message) {
	// build short address and 14 speed steps; neg. speed = forward, pos speed = revers
	// 0AAAAAAA 01DUSSSS
	uint8_t mydata;
	uint8_t index=0;

	new_message->repeat = NUM_DCC_SPEED_REPEAT;
	new_message->type = is_loco;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	// build up data: -> 01DUSSSS
	mydata = speed & 0x0F;
	mydata |= (speed & 0x80)>>2;
	mydata |= 0b01000000;                     // mark command
	new_message->dcc[index++] = mydata;
	new_message->size = index;
}

/// build short address and 28 speed steps; neg. speed = forward, pos speed = revers
/// 0AAAAAAA 01DCSSSS

void build_loko_28s(uint16_t nr, uint8_t speed, message *new_message)
{
	// build address and 28 speed steps; neg. speed = forward, pos speed = revers
	// 0AAAAAAA 01DCSSSS
	uint8_t mydata;
	uint8_t index = 0;

	new_message->repeat = NUM_DCC_SPEED_REPEAT;
	new_message->type = is_loco;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	// build up data: -> 01DCSSSS
	if ((speed & 0x1F) == 0) {
		mydata = 0;
	} else {
		if ((speed & 0x1F) == 1) {
			mydata = 1;    // emergency stop
		} else {
			mydata = (((speed & 0x1F) + 2) >> 1) | ((speed & 0x01) << 4);    // beim LSB kein +2, ist egal
			// intern ist speed 1 = nothalt
		}
	}
	mydata |= (speed & 0x80)>>2;
	mydata |= 0b01000000;                     // mark command
	new_message->dcc[index++] = mydata;
	new_message->size = index;
}

void build_loko_128s(uint16_t nr, uint8_t speed, message *new_message) {
	uint8_t index = 0;
	// message 0AAAAAAA 00111111 DSSSSSSS    ; Speed comes msb first

	new_message->repeat = NUM_DCC_SPEED_REPEAT;
	new_message->type = is_loco;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}

	new_message->dcc[index++] = 0b00111111;
	// build up data: -> DSSSSSSS
	new_message->dcc[index++] = speed;
	new_message->size = index;
}

void build_nmra_basic_accessory(uint16_t nr, char output, char activate, message *new_message)
{
	// Message: 10AAAAAA 1aaaBCCC
	// parameters: nr: turnout [0000-4095]
	//             output: coil (red, green) [0,1]
	//             activate: on off, [0,1]; note: intellibox only sends on, never off :-o
	//
	// Notes:   10111111 1000BCCC is broadcast
	//          aaa is bit 7 to 9 of address, but is transmitted inverted!

	uint16_t address   = 0;     // of the decoder
	uint8_t pairnr   = 0;     // decoders have pair of outputs, range [0-3]

	// calc real address of the decoder and the pairnr of the switch

	address = ((nr) / 4) + 1;  /* valid decoder addresses: 1..1023 */
	pairnr  = (nr) % 4;             // was nr-1

	new_message->repeat = NUM_DCC_ACC_REPEAT;
	new_message->type = is_acc;
	new_message->size = 2;
	new_message->dcc[0] = 0x80 | (address & 0x3F);
	new_message->dcc[1] = 0x80 | ( ((address / 0x40) ^ 0x07) * 0x10 );    // shift down, invert, shift up
	new_message->dcc[1] = new_message->dcc[1] | ((activate & 0x01) * 0x08);   // add B
	new_message->dcc[1] = new_message->dcc[1] | (pairnr * 2) | (output & 0x01);
}


void build_nmra_extended_accessory(uint16_t nr, char aspect, message *new_message)
{
	// Message: 10AAAAAA 0aaa0AA1 000sssss
	// parameters: nr: turnout [0001-4096]
	//             aspect: signal or state to execute (5 bits)
	//
	// Notes:   10111111 10000111 000sssss is broadcast
	//          aaa is msb address, but is transmitted inverted!

	uint16_t address   = 0;     // of the decoder
	uint8_t pairnr   = 0;     // decoders have pair of outputs, range [0-3]

	// calc real address of the decoder and the pairnr of the switch

	address = ((nr-1) / 4) + 1;  /* valid decoder addresses: 1..1023 */
	pairnr  = (nr-1) % 4;

	new_message->repeat = NUM_DCC_ACC_REPEAT;
	new_message->type = is_acc;
	new_message->size = 3;
	new_message->dcc[0] = 0x80 | (address & 0x3F);
	new_message->dcc[1] = 0x80 | ( ((address / 0x40) ^ 0x07) * 0x10 );    // shift down, invert, shift up
	new_message->dcc[1] = new_message->dcc[1] | (pairnr * 2) | (0x01);
	new_message->dcc[2] = aspect;
}

/* New function with correct handling for 7 / 14 bit addresses */

void build_function_grp1(int nr, uint8_t func, message *new_message){
	// Message: 0AAAAAAA or 11AAAAAA AAAAAAAA - 100FFFFF
	// FFFFF => FL, F4, F3, F2, F1
	uint8_t index;

	new_message->repeat = NUM_DCC_FUNC_REPEAT;
	new_message->type = is_void;
	index = 0;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	// build up data: -> 100FFFFF
	new_message->dcc[index++] = 0b10000000 | (func & 0x1F);
	new_message->size = index;
}

void build_function_grp2(int nr, uint8_t func, message *new_message) {
	// Message: 0AAAAAAA or 11AAAAAA AAAAAAAA - 1011FFFF
	// FFFFF => F8, F7, F6, F5
	uint8_t index;

	new_message->repeat = NUM_DCC_FUNC_REPEAT;
	new_message->type = is_void;
	index = 0;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	new_message->dcc[index++] = 0b10110000 | (func & 0x0F);
	new_message->size = index;
}

void build_function_grp3(int nr, uint8_t func, message *new_message) {
	// Message: 0AAAAAAA or 11AAAAAA AAAAAAAA - 1010FFFF
	// FFFF => F12, F11, F10, F9
	uint8_t index;

	new_message->repeat = NUM_DCC_FUNC_REPEAT;
	new_message->type = is_void;
	index = 0;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	new_message->dcc[index++] = 0b10100000 | (func & 0x0F);
	new_message->size = index;
}

void build_function_grp4(int nr, uint8_t func, message *new_message) {
	// Message: 0AAAAAAA or 11AAAAAA AAAAAAAA - 11110000 FFFFFFFF
	// FFFFFFFF => F20, F19, F18, F17, F16, F15, F14, F13
	uint8_t index;

	new_message->repeat = NUM_DCC_FUNC_REPEAT;
	new_message->type = is_void;
	index = 0;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	new_message->dcc[index++] = 0b11011110;
	new_message->dcc[index++] = func;
	new_message->size = index;
}

void build_function_grp5(int nr, uint8_t func, message *new_message) {
	// Message: 0AAAAAAA 11111000 FFFFFFFF
	// FFFFFFFF => F28, F27, F26, F25, F24, F23, F22, F21
	uint8_t index;

	new_message->repeat = NUM_DCC_FUNC_REPEAT;
	new_message->type = is_void;
	index = 0;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	new_message->dcc[index++] = 0b11011111;
	new_message->dcc[index++] = func;
	new_message->size = index;
}

/*---------------------------------------------------------------------------*/

void build_pom(int nr, uint16_t cv, uint8_t data, message *new_message){
	// message 11AAAAAA AAAAAAAA 111xxxxx xxxxxxxx
	// short form:               1111CCCC DDDDDDDD
	//                               0000 -------- unused
	//                               0010 xxxxxxxx = Acceleration Value (CV#23)
	//                               0011 xxxxxxxx = Deceleration Value (CV#24)
	//
	// long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
	//                               CC=00 Reserved for future use
	//                               CC=01 Verify byte
	//                               CC=11 Write byte
	//                               CC=10 Bit manipulation
	// here: only long form, only write byte
	//
	uint16_t cv_adr;
	uint8_t index = 0;

	cv_adr = cv-1;

	new_message->repeat = NUM_DCC_POM_REPEAT;
	new_message->type = is_prog;
	if (nr > 127) {
		new_message->dcc[index++] = 0xC0 | ( (uint8_t)(nr / 256) & 0x3F);
		new_message->dcc[index++] = (uint8_t)(nr & 0xFF);
	} else {
		new_message->dcc[index++] = (nr & 0x7F);
	}
	// build up data: -> 1110CCAA
	new_message->dcc[index++] = 0b11100000 | 0b00001100 | (uint8_t)((cv_adr >> 8) & 0b11);
	new_message->dcc[index++] = (uint8_t)(cv_adr & 0xFF);
	new_message->dcc[index++] = data;
	new_message->size = index;
}

void build_pom_accessory(int nr, uint16_t cv, uint8_t data, message *new_message) {
	// message 10AAAAAA 1aaaCDDD 111xxxxx xxxxxxxx
	//                      0000= whole decoder  1xxx -> output xxx
	// long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
	//                               CC=00 Reserved for future use
	//                               CC=01 Verify byte
	//                               CC=11 Write byte
	//                               CC=10 Bit manipulation
	// here: only long form, only write byte
	//
	uint16_t cv_adr;

	cv_adr = cv-1;

	new_message->repeat = NUM_DCC_POM_REPEAT;
	new_message->type = is_prog;
	new_message->size = 5;

	new_message->dcc[0] = 0x80 | (nr & 0x3F);
	new_message->dcc[1] = 0x80 | ( ((nr / 0x40) ^ 0x07) * 0x10 );    // shift down, invert, shift up
	// build up data: -> 1110CCAA
	new_message->dcc[2] = 0b01110000 | 0b00001100 | (uint8_t)((cv_adr >> 8) & 0b11);
	new_message->dcc[3] = (uint8_t)(cv_adr & 0xFF);
	new_message->dcc[4] = data;
}


//============================================================================
//
// Routines for locobuffer
//
//============================================================================
//
// purpose:   creates a flexible refresh of loco speeds
//
// how:       every speed command is entered to locobuffer.
//            search_locobuffer return a message of the loco
//            to be refreshed.
//            "younger" locos are refreshed more often.
//
// interface:
//			  enter_loco (int addr, char speed)
//            init_locobuffer ()
//            get_loco_format(uint16_t addr)
//            store_loco_format(uint16_t addr, t_format format)
//
//------------------------------------------------------------------------
// 
// Up to 64 locos may have different format (ESIZE_LOCO_FORMAT)
// please add following command to linker:
//
//   -Wl,--section-start=.ee_loco=0x810080

//
// see also: http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial#EEPROM
// is it default format?
// -> no:   search loco, if found: replace it
//          if not found: search empty and store it
//          if no empty found: error too many locos with extra format -> unhandled - Code 0
// -> yes:  search loco, if found: clear entry


// local static var to locobuffer

//uint8_t cur_i;             // this locobuffer entry is currently used
//uint8_t cur_ref_level;     // level = 0


//-----------------------------------------------------------------------------------
// neue Addr und Function in Locobuffer eintragen
// index : index into locobuff
// funct: 4 bits (lsbs) of function
// grp:   0 = light (only one bit, lsb)
//        1 = f1 - f4
//        2 = f5 - f8
//        3 = f9 - f12
//		  4 = f20 - f13
//		  5 = f28 - f21
// return:  void

void enter_func_to_locobuffer(uint8_t index, uint8_t funct, uint8_t grp) {
	if (grp == 0) 	locobuffer[index].fl = funct;
	else if (grp == 1) 	locobuffer[index].f4_f1 = funct & 0x0F;
	else if (grp == 2) 	locobuffer[index].f8_f5 = funct & 0x0F;
	else if (grp == 3) 	locobuffer[index].f12_f9 = funct & 0x0F;
	else if (grp == 4) 	locobuffer[index].f20_f13 = funct;
	else if (grp == 5) 	locobuffer[index].f28_f21 = funct;
}

// This routine creates DCC messages from the data stored in locobuffer
// Notes: Address handling: 1...127: DCC short address
//                          128 ... 10239 long address

// Note on speed handling:

void build_speed_message(uint8_t address, t_format format, uint8_t speed, uint8_t light, message * mess)
{
	uint8_t railspeed;

	railspeed = convert_speed_to_rail(speed, format);

	switch(format) {
	case DCC128:
		build_loko_128s(address, railspeed, mess);
		return;
		break;
	case DCC27: // Implmentierungsl�cke: DCC27 wird nicht unterst�tzt !!! dann halt idle ...
	case DCC28:
		build_loko_28s(address, railspeed, mess);
		return;
		break;
	case DCC14:
		build_loko_14s(address, railspeed, mess);
		// in DCC14, we have to add the light bit - taking in account long / short addresses
		if (light != 0) {
			mess->dcc[mess->size - 1] |= 0x10;
		}
		return;
		break;
	}
	return;
}


///-----------------------------------------------------------------------------------
// search_locobuffer returns pointer to dcc message
// The search result depends on the current_level:
// level < 3 : refresh locos speed (and light if DCC14)
// level 4: refresh grp1 and light
// level 5: refresh grp2
// level 6: refresh grp3
// level 7: refresh grp4
// level 8: refresh grp5
//
//

void search_locobuffer(message * temp) {
	// look for the next loco
	while ( (locobuffer[current_loco].address == 0) && (current_loco < SIZE_LOCOBUFFER)) {
		current_loco++;
	}
	// at the end of locobuffer, send idle
	if (current_loco == SIZE_LOCOBUFFER) {

		* temp = DCC_Idle;
		current_loco = 0;
		current_level ++;
	} else {
		// build message for loco
		if (current_level < 3) {
			build_speed_message(locobuffer[current_loco].address, locobuffer[current_loco].format,\
					locobuffer[current_loco].speed, locobuffer[current_loco].fl, temp);
		} else if (current_level == 3) {
			build_function_grp1(locobuffer[current_loco].address, locobuffer[current_loco].fl<<4 | locobuffer[current_loco].f4_f1, temp);
		} else if (current_level == 4) {
			build_function_grp2(locobuffer[current_loco].address, locobuffer[current_loco].f4_f1, temp);
		}else if (current_level == 5) {
			build_function_grp3(locobuffer[current_loco].address, locobuffer[current_loco].f8_f5, temp);
		}else if (current_level == 6) {
			build_function_grp4(locobuffer[current_loco].address, locobuffer[current_loco].f12_f9, temp);
		}else if (current_level == 7) {
			build_function_grp5(locobuffer[current_loco].address, locobuffer[current_loco].f20_f13, temp);
		}
		current_loco ++;
	}
	if (current_level >= 8)  current_level = 0;
}




//==========================================================================================
//
// COMMAND_ORGANIZER
//
// purpose:   receives the parsed command as dcc-message, checks for the
//            type of message and builds up the sequence of DCC messages
//            sent to DCC_OUT (only for normal run mode)
//

void init_organizer(void) {
	uint8_t i;

	send_BC_flag = 0;
	current_loco = 0;
	current_level = 0;

	for (i=0; i<SIZE_LOCOBUFFER; i++) {
		locobuffer[i].address = 0;
		locobuffer[i].speed = 0;
		locobuffer[i].control_by = 0;
		locobuffer[i].format = DCC128;
		locobuffer[i].fl = 0;
		locobuffer[i].f4_f1 = 0;
		locobuffer[i].f8_f5 = 0;
		locobuffer[i].f12_f9 = 0;
		locobuffer[i].f20_f13 = 0;
		locobuffer[i].f28_f21 = 0;
	}
}
#if 0
// return TRUE if message enqueued, return false, if not
bool enqueue (const message * new_message) {
	return xQueueSendToBack(dcc_queue, new_message, portMAX_DELAY);
}
#endif
//----------------------------------------------------------------------------------
// downstream
//----------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
// run_organizer: select the next message to put on the rail
//
// depending on the opendcc_state (fehlt noch!!!):
//
// RUN_OKAY:    run all track queues and refreshes
//
//xRUN_STOP:    only queue_hp, only queue_lp,                  // DCC Running, all Engines Emergency Stop
//              no repeatbuffer, no locobuffer
//
// RUN_OFF:     only idle (we dont want to loose a command)    // Output disabled (2*Taste, PC)
//
// RUN_SHORT:   only idle (we dont want to loose a command)    // Kurzschluss;
//
// RUN_PAUSE:   run queues and refresh, but set speed 0        // DCC Running, all Engines Speed 0
//
// PROG_OKAY:   only queue_prog,
//              no repeatbuffer, no locobuffer                 // Prog-Mode running
//
// PROG_SHORT:  only idle                                      // Prog-Mode + Kurzschluss
//
// PROG_OFF:    only idle                                      // Prog-Mode, abgeschaltet
//
// PROG_ERROR:  only queue_prog                                // Prog-Mode, Fehler beim Programmieren
//              no repeatbuffer, no locobuffer

// Achtung: organizer läuft zur Zeit bei RUN_OKAY


void run_organizer(void) {
	message search_message;
	if ( !timer1.dcc_busy()) {
		search_locobuffer(&search_message);
		if (send_BC_flag == 1) {
			timer1.send_dcc_packet(&DCC_BC_Stop);
		} else {
			timer1.send_dcc_packet(&search_message);
		}
	}

#ifdef DEBUG_UART3
	Serial3.print('K');
#endif
}




//=======================================================================================
//  Upstream Interface
//
//  Notes:
//      prior to any command to the organizer, init_organizer must be called
//      all commands return true, if there is still space in the queues
//
//  Summary of commands to organizer:
//      init_organizer(void)
//      organizer_ready(void)
//      do_loco_speed_f(addr, speed, format)   give speed and format for a loco
//      do_loco_speed(addr, speed)             give speed for a loco
//      do_loco_func_grp0(addr, funct)         light
//      do_loco_func_grp1(addr, funct)         f1-f4
//      do_loco_func_grp2(addr, funct)         f5-f8
//      do_loco_func_grp3(addr, funct)         f9-f12
//      do_loco_func_grp4(addr, funct)         f20-f13
//      do_loco_func_grp5(addr, funct)         f28-f21
//      do_accessory(addr, output, activate)   turnout
//      do_all_stop(void)                      Halt all locos
//
//---------------------------------------------------------------------------------------

uint16_t cache_addr;
int8_t cache_index;


int8_t scan_locobuffer(uint16_t addr) { // search locobuffer
	int8_t i;

	// we might call this routine quite a few time for the same loco,
	// speed up by caching last result
	if (addr == cache_addr) return cache_index;

	for (i=0; i<SIZE_LOCOBUFFER; i++) {
		if (locobuffer[i].address == addr) {
			cache_addr = addr;
			cache_index = i;
			return(i);
		}
	}
	cache_addr = 0;
	return (-1);   // (-1) if not found
}


/* Create a new Loco, must not exist */
locomem * new_loco(uint16_t addr) {
	uint8_t i;
	for (i=0; i<SIZE_LOCOBUFFER; i++) {
		if (locobuffer[i].address == 0) {
			locobuffer[i].address = addr;
			locobuffer[i].control_by = 0;
			locobuffer[i].format = DCC128;
			locobuffer[i].speed = 0;
			locobuffer[i].fl = 1;
			locobuffer[i].f4_f1 = 0;
			locobuffer[i].f8_f5 = 0;
			locobuffer[i].f12_f9 = 0;
			locobuffer[i].f20_f13 = 0;
			locobuffer[i].f28_f21 = 0;
			return(&locobuffer[i]);
		}
	}
	return NULL;   // NULL if no space
}

/*
 * Return pointer to loco from control_by field, NULL if not found
 */
locomem * find_control(int8_t control_by) {
	uint8_t i;
	for (i=0; i<SIZE_LOCOBUFFER; i++) {
		if (locobuffer[i].control_by == control_by) {
			return(&locobuffer[i]);
		}
	}
	return NULL;
}

/*
 * Return pointer to loco from address field, NULL if not found
 */
locomem * get_loco (uint16_t address) {
	int8_t index;
	index = scan_locobuffer(address);
	if (index == -1) return NULL; // Nobody at this address
	return &locobuffer[index];
}


// Speed einstellen: dieses Kommando geht auch in die high priority queue falls gebremst wird;
// immer in low priority queue, von dort wird es nach dem Ausgeben in repeatbuffer
// �bernommen.
// return false, if full; return 1 if there is still space


void do_loco_speed(uint16_t addr, uint8_t speed) {
	int8_t index;
	index = scan_locobuffer(addr);
	if (index == -1) return; // Loco not known ...
	locobuffer[index].speed = speed;
}

void do_loco_speed_relative(uint16_t addr, int8_t delta_speed) {
	int8_t index;
	uint8_t i;
	int16_t j;
	index = scan_locobuffer(addr);
	if (index == -1) return; // Loco not known ...
	// get out of e-stop "normally"
	if (locobuffer[index].speed == 1) locobuffer[index].speed=0;
	// do not touch direction
	i = locobuffer[index].speed & 0x80;
	// make sure we dont get over 128 or negative
	j = (locobuffer[index].speed & 0x7f) + delta_speed;
	if (j > 125) j = 125;
	if (j < 0) j = 0;
	locobuffer[index].speed = i | (j & 0x7F);
}


// entering loco functions

void do_loco_func_grp0(uint16_t addr, uint8_t funct) {
	int8_t index;
	index = scan_locobuffer(addr);
	if (index == -1) return;
	enter_func_to_locobuffer(index, funct, 0);
	// grp 0 = light
}


void do_loco_func_grp1(uint16_t addr, uint8_t funct) {
	int8_t index;
	index = scan_locobuffer(addr);
	if (index == -1) return; // Loco not known ...
	enter_func_to_locobuffer(addr, funct, 1);
}

void do_loco_func_grp2(uint16_t addr, uint8_t funct) {
	int8_t index;
	index = scan_locobuffer(addr);
	if (index == -1) return; // Loco not known ...
	enter_func_to_locobuffer(addr, funct, 1);
}
void do_loco_func_grp3(uint16_t addr, uint8_t funct) {
	int8_t index;
	index = scan_locobuffer(addr);
	if (index == -1) return; // Loco not known ...
	enter_func_to_locobuffer(addr, funct, 1);
}

//
// parameters: addr:     turnout decoder [0000-4095]
//             output:   coil (red, green) [0,1]
//             activate: off, on = [0,1];
// special:    if addr >= virtual_decoder_offset, then remap the call to dmx

void do_accessory(uint16_t addr, uint8_t output, uint8_t activate) {
	message temp;
	build_nmra_basic_accessory(addr, output, activate, &temp);
	timer1.send_dcc_packet( &temp);
}

// programming on the main (locos)
//
// parameters: addr:     loco
//             cv:       config var in this loco
//             data:     value to be written

void do_pom_loco(uint16_t addr, uint16_t cv, uint8_t data) {
	message temp;
	build_pom(addr, cv, data, &temp);
	timer1.send_dcc_packet( &temp);
}


// programming on the main (accessory)
//
// parameters: addr:     accessory decoder addr (not turnout!)
//             cv:       config var in this accessory
//             data:     value to be written

void do_pom_accessory(uint16_t addr, uint16_t cv, uint8_t data) {
	message temp;
	build_pom_accessory(addr, cv, data, &temp);
	timer1.send_dcc_packet( &temp);
}



void do_all_stop(void){
	// repeat abschalten
	DCC_BC_Brake.repeat = 10;
	timer1.send_dcc_packet(&DCC_BC_Brake);
	send_BC_flag = 1;      // this forces organizer to only send stop packet
}

void restart_all(void) {
	send_BC_flag = 0;
}
