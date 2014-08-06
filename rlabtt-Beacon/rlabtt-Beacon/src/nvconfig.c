/*____________________________________________________________________________*/
/* File: nvconfig.c
 *
 * Created: 03/08/2014 10:01:06
 * Revised  03/08/2014 16055
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 *  This source module manages the storage of configuration data
 *
 *  Config is stored in non volatile eeprom
 */  
/*____________________________________________________________________________*/
/*
 * Copyright, License and Attribution:
 *
 * This is a derived work and a modified version of the TinyGPS++ Library created by 
 * Mikal Hart (2008-2013)
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
/*____________________________________________________________________________*/

#include "nvconfig.h"

// External configuration
extern struct NVCONFIG nvconfig;

void get_nvconfig(void)
{
	nvconfig.uart_on_startup = START_UART_USB;
	nvconfig.beacon_repeat_time = 10;  // units of 500ms beacon every 5 seconds
	nvconfig.payload_id[10] = "rlabtt";
	
}