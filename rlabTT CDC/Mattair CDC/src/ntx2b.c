/*____________________________________________________________________________*/
/*
 * File: ntx2b.c
 *
 * Created: 07/06/2014 16:01:06
 * Revised  14/07/2014 16:55
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter 
 */
/*____________________________________________________________________________*/
/*
 * 
 *  This source module implements the driver functions for the NTX2B-FA
 *  434MHz transmitter
 *
 *
*/
/*____________________________________________________________________________*/
/*
 * Copyright, License and Attribution:
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
/*____________________________________________________________________________*/
#include <ntx2b.h>

void ntx2b_uart_init(void)
{
	static usart_rs232_options_t NTX2B_USART_SERIAL_OPTIONS = {
		.baudrate = NTX2B_USART_SERIAL_BAUDRATE,
		.charlength = NTX2B_USART_SERIAL_CHAR_LENGTH,
		.paritytype = NTX2B_USART_SERIAL_PARITY,
		.stopbits = NTX2B_USART_SERIAL_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_D, PR_USART0_bm);
	usart_init_rs232(NTX2B_USART_SERIAL, &NTX2B_USART_SERIAL_OPTIONS);
	usart_rx_disable(NTX2B_USART_SERIAL);

	
}


void ntx2bPutch(char value)
{
	while(!usart_data_register_is_empty(NTX2B_USART_SERIAL));
	usart_putchar(NTX2B_USART_SERIAL, value);
}
/*
|  Output a NUL-terminated string to the NTX2 serial port.
|
*/
void  ntx2bPutstr( char * pstr )
{
	char   c;

	while ( (c = *pstr++) != 0 )
	{
		if ( c == '\n' )
		{
			ntx2bPutch( '\r' );
			ntx2bPutch( '\n' );
		}
		else   ntx2bPutch( c );
	}
	
}

/*
|  Output a NUL-terminated string to the NTX2B serial port.
|  The string is expected to be in the program code (flash) memory space.
|
*/
void  ntx2Putstr_P( PGM_P pksz )
{
	char   c;

	while ( (c = pgm_read_byte(pksz)) != 0 )
	{
		if ( c == '\n' )
		{
			ntx2bPutch( '\r' );
			ntx2bPutch( '\n' );
		}
		else   ntx2bPutch( c );
		pksz++ ;
	}
	
}

/*
|  Output 4 LS bits of a byte as Hex (or BCD) ASCII char.
|
|
*/
void  ntx2bPutHexDigit( unsigned char d )
{
	d &= 0x0F;
	if ( d < 10 )  ntx2bPutch ( '0' + d );
	else  ntx2bPutch ( 'A' + d - 10 );
}

/*
|  Output byte as 2 Hex ASCII chars, MSD first.
|
|
*/
void  ntx2bPutHexByte( unsigned char b )
{
	ntx2bPutHexDigit( b >> 4 );
	ntx2bPutHexDigit( b );
}
