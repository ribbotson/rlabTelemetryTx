/*____________________________________________________________________________*/
/* File: cmnd.h
 *
 * Created: 22/05/2014 16:34:57
 * Revised  08/07/2014 16:23
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
 *  This source module implements the "host command interface" (HCI) and a
 *  set of "debug" commands.
 *
 *  A command string is composed of a 2-letter command "name" and a number of
 *  user-supplied "arguments" (parameters). Some commands have no arguments.
 *  A single space must be inserted between command buffer arguments, where
 *  there is more than one (including the 2-letter command name).
*/
/*____________________________________________________________________________*/
/*
 * License and attribution:
 * This is a derived work and a modified version of the HCI interface created by 
 * M.J. Bauer ( 2007-2011) [Ref:  www.mjbauer.biz]
 *
 * Copyright (c) 2014 Richard Ibbotson (richard.ibbotson@btinternet.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2.1 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*____________________________________________________________________________*/



#ifndef CMND_H_
#define CMND_H_
#include "avr/pgmspace.h"
#include "ctype.h"
#include "asf.h"
#define  CMD_MSG_SIZE      (63)     // Maximum command string length


//#define FALSE 0
//#define TRUE  1

//#define  NEW_LINE          {  }
#define  BUILD_VER_MAJOR  1
#define  BUILD_VER_MINOR  0
#define  BUILD_VER_DEBUG  00





/*_______________________  F U N C T I O N   P R O T O T Y P E S  ______________________*/
void   hci_init(void);                              // initialises HCI variables
void   hci_service( void );                     // checks for data received from HCI stream
void   hci_process_input( char c );             // builds a command message
void   hci_exec_command( void );                // executes host command
void   hci_clear_command( void );               // clears command msg buffer; resets pointer
void   hci_put_resp_term( void );               // Outputs the termination (prompt) chars
void   hci_put_cmd_error(void);                     // Outputs "! Command Error" (interactive only)

void   null_cmd( void );                        // Command functions
void   list_cmd( void );
void   version_cmd( void );
void   default_params_cmd( void );
void   adc_cmd( void );
void   dac_cmd( void );
void   set_tx_channel_cmd(void);
void   write_tx_channel_cmd(void);
void   beacon_cmd( void );
void   show_status(void);
void   show_config(void);
void   read_eeprom(void);
void   write_eeprom(void);
void   set_tx_trim_cmd(void);
void   set_cw_cmd(void);
void   show_telemetry_cmd(void);
void   hci_clear_command( void );               // clears command msg buffer; resets pointer
void   hci_put_resp_term( void );               // Outputs the termination (prompt) chars

void   dt_format( char * , long );
void   angle_format( char * , signed long );

unsigned char  dectobin( char c );                      // convert dec ASCII digit to binary
unsigned int decatoi( char * pnac, char bNdigs );     // convert dec ASCII string to integer
unsigned char  hexctobin( char c );                     // convert hex ASCII digit to binary
unsigned int hexatoi( char * s );                     // convert hex ASCII string to integer
char   isHexDigit( char c );                    // Rtn TRUE if char is hex ASCII digit
char  isDecDigit( char c );						// Rtn TRUE if char is dec ASCII digit



#endif /* CMND_H_ */