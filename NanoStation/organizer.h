//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      organizer.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//------------------------------------------------------------------------
// define the structures for DCC messages
//------------------------------------------------------------------------
// SIZE_... : see config.h
//-------------------------------------- primary command buffer (fifo)

typedef struct {
    uint8_t repeat;             // counter for repeat or refresh (depending)
    uint8_t size;               // 2 .. 5
    tmessage type;              // enum: isvoid, isloco, accessory, ...
    uint8_t dcc[MAX_DCC_SIZE];  // the dcc content
} message;

// define a structure for the loco memeory (6 bytes)

typedef enum {DCC14, DCC27, DCC28, DCC128} t_format;

typedef struct
  {
	uint8_t control_by;
    uint16_t address;       // address (either 7 or 14 bits)
    uint8_t speed;          // this is in effect a bitfield:
                                    // msb = direction (1 = forward, 0=revers)
                                    // else used as integer, speed 1 ist NOTHALT
                                    // this is i.e. for 28 speed steps:
                                    // 0=stop
                                    // 1=emergency stop
                                    // 2..127: speed steps 1..126
                                    // speed is always stored as 128 speed steps
                                    // and only converted to the according format
                                    // when put on the rails (30.09.2006)
    t_format format;        // 00 = 14, 01=27, 10=28, 11=128 speed steps.
    uint8_t fl:1;
    uint8_t f4_f1:4;
    uint8_t f8_f5:4;
    uint8_t f12_f9:4;
    uint8_t f20_f13;
    uint8_t f28_f21;
  } locomem;

// Note on speed coding:
//
// Speed is always stored as 0..127.
// Speedentries from IBOX are handled directly.
// Speedentries from LENZ are converted (speed_from_rail) when they are put to
//                                      (speed_to_rail) or read from locobuffer.
// When a message is put on the rails, speed is converted according to format.


//extern message message_queue[SIZE_MESSAGE_QUEUE];

// ----------------------------------------------------------------
// predefined messages
//
// stored in bss, copied at start to sram

extern message DCC_Reset;    	// DCC-Reset-Paket
extern message DCC_Idle;    	// DCC-Idle-Paket
extern message DCC_Factory_Reset;
extern message DCC_BC_Stop ;    // Broadcast Motor off: // 01DC000S :D=x, C=1 (ignore D)
extern message DCC_BC_Brake ;    // Broadcast Slow down // if S=0: slow down

//------------------------------------------------------------------
// Upstream Interface for parser
//------------------------------------------------------------------

void init_organizer(void);                                      // must be called once at program start

void run_organizer(void);                                       // must be called in a loop!

// -- routines for command entry
bool organizer_ready(void);                                     // true if command can be accepted

uint8_t convert_speed_to_rail(uint8_t speed128, t_format format);
uint8_t convert_speed_from_rail(uint8_t speed, t_format format);

locomem * get_loco (uint16_t address);
void do_loco_speed(uint16_t address, uint8_t speed);     // eine Lok eintragen (speed 1-127), will be converted to format
void do_loco_speed_relative(uint16_t addr, int8_t delta_speed);
void do_loco_func_grp0(uint16_t address, uint8_t funct);
void do_loco_func_grp1(uint16_t address, uint8_t funct);
void do_loco_func_grp2(uint16_t address, uint8_t funct);
void do_loco_func_grp3(uint16_t address, uint8_t funct);
void do_loco_func_grp4(uint16_t address, uint8_t funct);
void do_loco_func_grp5(uint16_t address, uint8_t funct);

void do_all_stop(void);
void restart_all(void);
void do_accessory(uint16_t address, uint8_t output, uint8_t activate);      // turnout
void do_pom_loco(uint16_t addr, uint16_t cv, uint8_t data);                     // program on the main
void do_pom_accessory(uint16_t addr, uint16_t cv, uint8_t data);                // program on the main

void build_loko_14s(uint16_t nr, int speed, message * new_message);
void build_loko_28s(uint16_t nr, uint8_t speed, message * new_message);
void build_loko_128s(uint16_t nr, uint8_t speed, message * new_message);
 
void build_nmra_basic_accessory(uint16_t nr, char output, char activate, message * * new_message);
void build_nmra_extended_accessory(uint16_t nr, char aspect, message * * new_message);

//--------------------------------------------------------------------------
// Routines for locobuffer
//--------------------------------------------------------------------------

void enter_speed_to_locobuffer(uint8_t index, uint8_t speed);                      // returns index
void enter_func_to_locobuffer(uint8_t index, uint8_t funct, uint8_t grp);   // returns index
void search_locobuffer(message * loco_message);   // fills in loco_message and returns it
locomem * find_control(int8_t control_by);


int8_t scan_locobuffer(uint16_t addr);
locomem * new_loco(uint16_t addr);

