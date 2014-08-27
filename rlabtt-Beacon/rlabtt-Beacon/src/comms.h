/*____________________________________________________________________________*/
/* File: comms.h
 *
 * Created: 24/07/2014 16:29:06
 * Revised  
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
 *  This header file implements the communications interfaces of the rlab
 *  Telemetry Transmitter
 *
 *  This module supports the following interfaces
 *		: USB serial
 *		: Ext serial
 *		: GPS serial
 *		: NTX2B serial
 *		: NTX2B Modulation
 *  
*/
/*____________________________________________________________________________*/
/*
 * Copyright, License and Attribution:
 *
 * 
 *
 * 
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



#ifndef COMMS_H_
#define COMMS_H_
#include <stdio.h>
#include "asf.h"

#define RTTY50	0
#define RTTY300 1
#define RTTY300_SPACE 2565
#define RTTY300_MARK 1525 // mark is lower tone, idle and '1' bit
#define RTTY300_CARRIER 2048

#define GPS_USART_SERIAL                 &USARTC0
#define GPS_USART_SERIAL_BAUDRATE        9600
#define GPS_USART_SERIAL_CHAR_LENGTH     USART_CHSIZE_8BIT_gc
#define GPS_USART_SERIAL_PARITY          USART_PMODE_DISABLED_gc
#define GPS_USART_SERIAL_STOP_BIT        false

#define NTX2B_USART_SERIAL               &USARTD0
#define NTX2B_USART_SERIAL_BAUDRATE      4800
#define NTX2B_USART_SERIAL_CHAR_LENGTH   USART_CHSIZE_8BIT_gc
#define NTX2B_USART_SERIAL_PARITY        USART_PMODE_DISABLED_gc
#define NTX2B_USART_SERIAL_STOP_BIT      false

#define EXT_USART_SERIAL                 &USARTE0
#define EXT_USART_SERIAL_BAUDRATE        115200
#define EXT_USART_SERIAL_CHAR_LENGTH     USART_CHSIZE_8BIT_gc
#define EXT_USART_SERIAL_PARITY          USART_PMODE_DISABLED_gc
#define EXT_USART_SERIAL_STOP_BIT        false

enum {TRANSMIT_OFF, TRANSMIT_CW, TRANSMIT_BEACON};
	
void comms_init(void);
void comms_cdc_set_dtr(uint8_t port, bool b_enable);
void comms_cdc_rx_notify(uint8_t port);
void ntx2b_uart_init(void);
int ntx2b_ser_putchar(char c, FILE *stream);
void ntx2bPutch(char value);
void  ntx2bPutHexDigit( unsigned char d );
bool comms_cdcgotchar(void);
void comms_cdc_clr_gotchar(void);
void  ntx2bPutstr( char * pstr );
void  ntx2Putstr_P( PGM_P pksz );
void  ntx2bPutHexDigit( unsigned char d );
void  ntx2bPutHexByte( unsigned char b );
void  ntx2b_mod_init(void);
void  ntx2b_mod_enable( char);
void  ntx2b_mod_disable( void);
void  ntx2b_mod_transmit( void);
bool  ntx2b_mod_busy(void);

int comms_usb_putchar(char c, FILE *stream);
int ext_ser_putchar(char c, FILE *stream);
void dac_init(void);

void gps_uart_init(void);

void ext_uart_init(void);


#endif /* COMMS_H_ */