/*____________________________________________________________________________*/
/*
 * File: ntx2b.h
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


#ifndef NTX2B_H_
#define NTX2B_H_

#include <avr/pgmspace.h> 
#include <asf.h>

void ntx2b_uart_init(void);
void ntx2bPutch(char value);
void  ntx2bPutHexDigit( unsigned char d );
void  ntx2bPutstr( char * pstr );
void  ntx2Putstr_P( PGM_P pksz );
void  ntx2bPutHexDigit( unsigned char d );
void  ntx2bPutHexByte( unsigned char b );


#endif /* NTX2B_H_ */