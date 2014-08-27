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
#include "asf.h"

#define		START_UART_USB	0
#define		START_UART_EXT	1
#define		START_UART_AUTO 2

#define		MAGIC_EEPROM_PAGE	0
#define		CONFIG_EEPROM_PAGE	1
#define		MAGIC_EEPROM_ADDRESS (MAGIC_EEPROM_PAGE * EEPROM_PAGE_SIZE)
#define		CONFIG_EEPROM_ADDRESS (CONFIG_EEPROM_PAGE * EEPROM_PAGE_SIZE)


// Magic Number structure
struct MAGIC {
	long			magic_number;
	int				config_version;
};


// Configuration Data structure
struct CONFIG {
	char			uart_on_startup;
	unsigned char	beacon_repeat_time; // units of 500ms 
	char			payload_id[10];
	char			modulation_mode;
	char			tx_channel;
	char			tx_trim;
};


void get_nvconfig(struct CONFIG *); // get the config structure from EEPROM
void set_nvconfig(struct CONFIG *); // save the config structure to EEPROM
void get_nvmagic(struct MAGIC *); // get magic number structure from EEPROM
void set_nvmagic(struct MAGIC *); // set magic number structure to EEPROM


#endif /* NVCONFIG_H_ */