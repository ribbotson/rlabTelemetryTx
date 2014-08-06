/*____________________________________________________________________________*/
/* File: nvconfig.h
 *
 * Created: 03/08/2014 10:29:06
 * Revised  03/08/2014 10:29:06
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
  *  This include file manages the storage of configuration data
  *
  *  Config is stored in non volatile eeprom
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


#ifndef NVCONFIG_H_
#define NVCONFIG_H_


#define		START_UART_USB	0

// Configuration Data structure
struct NVCONFIG {
	char		uart_on_startup;
	unsigned char beacon_repeat_time; // units of 500ms beacon every 5 seconds
	char payload_id[10];
	char		modulation_mode;
};


void get_nvconfig(void);



#endif /* NVCONFIG_H_ */