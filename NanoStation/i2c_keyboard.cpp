/*
 * i2c_keyboard.cpp
 *
 *  Created on: 20 juin 2012
 *      Author: florrain
 */
#include "elcheapo_remote.h"

#include "TWI_Master.h"
#include "i2c_keyboard.h"

I2c_Keyboard kbd1(0x40);

I2c_Keyboard::I2c_Keyboard(uint8_t i2c_address){
	_i2c_address = i2c_address;
	for (uint8_t i=0; i<4; i++) {
		column[i]=0;
	}
//	state = 0;
//	last_key = 0;
}

uint8_t I2c_Keyboard::scan(void) {
	uint8_t buffer [4];
	uint8_t temp1, temp2;
	// Keyboard connection is 	C4 C3 C2 C1 L4 L3 L2 L1
	// I2C Pin					P7 P6 P5 P4 P3 P2 P1 P0
	// Don't play around if I2C still busy.

	// scan column 1
	buffer[0] = _i2c_address|TWI_WRITE;
	buffer[1] = 0xef; // 11101111 C1 = 0
	if (!TWI_Start_Transceiver_With_Data( buffer , 2 )) return FALSE;
	buffer[0] = _i2c_address|TWI_READ;
	if (!TWI_Start_Transceiver_With_Data( buffer , 3 )) return FALSE;
	if (!TWI_Get_Data_From_Transceiver(buffer,3)) return FALSE;
	temp1 = (~buffer[1]) & 0x0f; // extract Line info
	temp2 = (~buffer[2]) & 0x0f; // extract Line info
	temp1 = temp1 & (~(temp1 ^ temp2)); // remove bits which have changed
	column[0] = temp1; // 0000 L4 L3 L2 L1 format

	// scan column 2
	buffer[0] = _i2c_address|TWI_WRITE;
	buffer[1] = 0xdf; // 11011111 C2 = 0
	if (!TWI_Start_Transceiver_With_Data( buffer , 2 )) return FALSE;
	buffer[0] = _i2c_address|TWI_READ;
	if (!TWI_Start_Transceiver_With_Data( buffer , 3 )) return FALSE;
	if (!TWI_Get_Data_From_Transceiver(buffer,3)) return FALSE;
	temp1 = (~buffer[1]) & 0x0f; // extract Line info
	temp2 = (~buffer[2]) & 0x0f; // extract Line info
	temp1 = temp1 & (~(temp1 ^ temp2)); // remove bits which have changed
	column[1] = temp1; // 0000 L4 L3 L2 L1 format

	// scan column 3
	buffer[0] = _i2c_address|TWI_WRITE;
	buffer[1] = 0xbf; // 10111111 C3 = 0
	if (!TWI_Start_Transceiver_With_Data( buffer , 2 )) return FALSE;
	buffer[0] = _i2c_address|TWI_READ;
	if (!TWI_Start_Transceiver_With_Data( buffer , 3 )) return FALSE;
	if (!TWI_Get_Data_From_Transceiver(buffer,3)) return FALSE;
	temp1 = (~buffer[1]) & 0x0f; // extract Line info
	temp2 = (~buffer[2]) & 0x0f; // extract Line info
	temp1 = temp1 & (~(temp1 ^ temp2)); // remove bits which have changed
	column[2] = temp1; // 0000 L4 L3 L2 L1 format

	// scan column 4
	buffer[0] = _i2c_address|TWI_WRITE;
	buffer[1] = 0x7f; // 01111111 C4 = 0
	if (!TWI_Start_Transceiver_With_Data( buffer , 2 )) return FALSE;
	buffer[0] = _i2c_address|TWI_READ;
	if (!TWI_Start_Transceiver_With_Data( buffer , 3 )) return FALSE;
	if (!TWI_Get_Data_From_Transceiver(buffer,3)) return FALSE;
	temp1 = (~buffer[1]) & 0x0f; // extract Line info
	temp2 = (~buffer[2]) & 0x0f; // extract Line info
	temp1 = temp1 & (~(temp1 ^ temp2)); // remove bits which have changed
	column[3] = temp1; // 0000 L4 L3 L2 L1 format

	return TRUE;
}

#if 0
uint8_t I2c_Keyboard::get_long(uint8_t column_id) {
	uint8_t temp = long_press[column_id & 0x03];
	// only return long key press once
	if (temp!=0) {
		long_press[column_id & 0x03] = 0;
		temp_time[column_id & 0x03]=0;
	}
	return temp;
}
#endif

int8_t I2c_Keyboard::get_key(void) {
	if (column[0] != 0) {
		switch (column[0]) {
		case 0x08: return '*';
		case 0x04: return '7';
		case 0x02: return '4';
		case 0x01: return '1';
		default: return 0;
		}
	}
	if (column[1] != 0) {
		switch (column[1]) {
		case 0x08: return '0';
		case 0x04: return '8';
		case 0x02: return '5';
		case 0x01: return '2';
		default: return 0;
		}
	}
	if (column[2] != 0) {
		switch (column[2]) {
		case 0x08: return '#';
		case 0x04: return '9';
		case 0x02: return '6';
		case 0x01: return '3';
		default: return 0;
		}
	}
	if (column[3] != 0) {
		switch (column[3]) {
		case 0x08: return 'D';
		case 0x04: return 'C';
		case 0x02: return 'B';
		case 0x01: return 'A';
		default: return 0;
		}
	}
	return 0;
}

int8_t I2c_Keyboard::get_key_debounced(uint8_t & last) {
	/* Only return key presses once */
	int8_t key=get_key();
	if (key == last)
		return 0;
	last = key;
	return key;
}


uint8_t I2c_Keyboard::get_star_line(void) {
	uint8_t temp=0;
	if ((column[0] & 0x08) != 0 ) temp |= 0x08;
	if ((column[1] & 0x08) != 0 ) temp |= 0x04;
	if ((column[2] & 0x08) != 0 ) temp |= 0x02;
	if ((column[3] & 0x08) != 0 ) temp |= 0x01;
	return temp;
}

int8_t I2c_Keyboard::get_star_line_debounced(uint8_t & last) {
	/* Only return key presses once */
	int8_t key=get_star_line();
	if (key == last)
		return 0;
	last = key;
	return key;
}
