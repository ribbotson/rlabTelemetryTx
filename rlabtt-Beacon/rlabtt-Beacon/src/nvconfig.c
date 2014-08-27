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
#include "string.h"
// Buffers for eeprom operations
uint8_t eeprom_write_page[EEPROM_PAGE_SIZE];
uint8_t eeprom_read_page[EEPROM_PAGE_SIZE];

void get_nvconfig(struct CONFIG * nvconfig)
{	
	nvm_eeprom_read_buffer(CONFIG_EEPROM_ADDRESS, eeprom_read_page, EEPROM_PAGE_SIZE);	
	memcpy(nvconfig, eeprom_read_page, sizeof(struct CONFIG));	
}



void set_nvconfig(struct CONFIG * nvconfig) // save the config structure to EEPROM
{
	memset(eeprom_write_page, 0, EEPROM_PAGE_SIZE);
	memcpy(eeprom_write_page, nvconfig, sizeof(struct CONFIG));
	nvm_eeprom_load_page_to_buffer(eeprom_write_page);
	nvm_eeprom_atomic_write_page(CONFIG_EEPROM_PAGE);	
}


void get_nvmagic(struct MAGIC * nvmagic) // get magic number structure from EEPROM
{
	nvm_eeprom_read_buffer(MAGIC_EEPROM_ADDRESS, eeprom_read_page, EEPROM_PAGE_SIZE);
	memcpy(nvmagic, eeprom_read_page, sizeof(struct MAGIC));
}


void set_nvmagic(struct MAGIC * nvmagic) // set magic number structure to EEPROM
{
	memset(eeprom_write_page, 0, EEPROM_PAGE_SIZE);
	memcpy(eeprom_write_page, nvmagic, sizeof(struct MAGIC));
	nvm_eeprom_load_page_to_buffer(eeprom_write_page);
	nvm_eeprom_atomic_write_page(CONFIG_EEPROM_PAGE);
}