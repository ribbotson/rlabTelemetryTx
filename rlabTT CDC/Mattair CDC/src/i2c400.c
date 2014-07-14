/*____________________________________________________________________________*/
/*
 * File: i2c400.c
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
 *  This source module implements the I2C interface running at 400kHz
 *
 *  This module support the I2C devices on this bus including GPS, SE95 Temperature
 * sensor and MPL115A2 altitude sensor
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

#include <asf.h>
#include "i2c400.h"

twi_options_t m_options = {		.speed     = I2C400_SPEED,
							    .chip      = I2C400_ADDRESS, 
								
};

unsigned char i2c_buffer[8];

//Package to initialise SE95 Temperature Sensor
twi_package_t se95_init_package = {
	.addr_length = 0,
	.addr		 = {0x00, 0x00, 0x00},
	.chip        = SE95I2C_ADDRESS,
	.buffer      = (void *) &i2c_buffer[0],
	.length      = 0,
	.no_wait     = false
};

//Package to read SE95 Temperature Sensor
twi_package_t se95_read_package = {
	.addr_length = 1,
	.addr		 = {0x00, 0x00, 0x00},
	.chip        = SE95I2C_ADDRESS,
	.buffer      = (void *) &i2c_buffer[0],
	.length      = 2,
	.no_wait     = false
};								

//Package to make MPL115 do conversion
twi_package_t mpl115_convert_package = {
	.addr_length = 0,
	.addr		 = {0x00, 0x00, 0x00},
	.chip        = MPL115I2C_ADDRESS,
	.buffer      = (void *) &i2c_buffer[0],
	.length      = 2,
	.no_wait     = false
};

//Package to read MPL115
twi_package_t mpl115_read_package = {
	.addr_length = 1,
	.addr		 = {0x00, 0x00, 0x00},
	.chip        = MPL115I2C_ADDRESS,
	.buffer      = (void *) &i2c_buffer[0],
	.length      = 12,
	.no_wait     = false
};
								
void i2c400_init(void)
{
	m_options.speed_reg = TWI_BAUD(sysclk_get_cpu_hz(),m_options.speed);

	sysclk_enable_peripheral_clock(&TWI_I2C400);

	twi_master_init(&TWI_I2C400,&m_options);
	
}


void se95_init(void)
{
	i2c_buffer[0] = 1;
	i2c_buffer[1] = 0;
	twi_master_write(&TWI_I2C400, &se95_init_package);
	
}

void mpl115_convert(void)
{
	i2c_buffer[0] =  0x12;
	i2c_buffer[1] = 0;
	twi_master_write(&TWI_I2C400, & mpl115_convert_package);
}
signed int se95_read(void)
{
	twi_master_read(&TWI_I2C400, &se95_read_package);
	return (i2c_buffer[0] << 8) | i2c_buffer[1];
}

signed int mpl115_read_pressure(void)
{
	unsigned int Padc, Tadc;
	signed int a0, b1, b2, c12;
	signed long lt1, lt2, lt3;
	signed long c12x2, a1, a1x1, y1, a2x2, PComp;
	twi_master_read(&TWI_I2C400, &mpl115_read_package);
	Padc = (i2c_buffer[0] << 8) | i2c_buffer[1];
	Tadc = (i2c_buffer[2] << 8) | i2c_buffer[3];
	a0 = (i2c_buffer[4] << 8) | i2c_buffer[5];
	b1 = (i2c_buffer[6] << 8) | i2c_buffer[7];
	b2 = (i2c_buffer[8] << 8) | i2c_buffer[9];
	c12 = (i2c_buffer[10] << 8) | i2c_buffer[11];
	
	// Do compensation
	Padc>>=6; 
	Tadc>>=6;
	
	//******* STEP 1 : c12x2 = c12 * Tadc
	lt1 = c12; // s(16,24) // c12 is s(14,13)+9 zero pad = s(16,15)+9 => s(16,24) left justified
	lt2 = (signed int)Tadc; // u(10,0)
	lt3 = lt1 * lt2; // s(26,24) = c12 * Tadc
	c12x2 = lt3 >> 11; // s(15,13) ? EQ 3 = c12x2
	//******* STEP 2 : a1 = b1 + c12x2
	lt1 = (signed int)b1; // s(16,13)
	lt2 = c12x2; // s(15,13)
	lt3 = lt1 + lt2; // s(16,13) = b1 + c12x2
	a1 = lt3; // s(16,13) ? EQ 4 = a1
	//******* STEP 3 : a1x1 = a1 * Padc
	lt1 = a1; // s(16,13)
	lt2 = (signed int)Padc; // u(10,0)
	lt3 = lt1 * lt2; // s(26,13) = a1 * Padc
	a1x1 = lt3; // s(26,13) ? EQ 5 = a1x1
	//******* STEP 4 y1 = a0 + a1x1
	lt1 = ((signed long)a0) << 10; // s(26,13) shifted to match a1x1 F value to add. So s(16,3)<<10 = s(26,13)
	lt2 = a1x1; // s(26,13)
	lt3 = lt1 + lt2; // s(26,13) = a0 + a1x1
	y1 = lt3; // s(26,13) ? EQ 6 = y1
	//******* STEP 5 : a2x2 = b2 * Tadc
	lt1 = (signed long)b2; // s(16,14)
	lt2 = (signed long)Tadc; // u(10,0)
	lt3 = lt1 * lt2; // s(26,14) = b2 * Tadc
	a2x2 = lt3 >> 1; // s(25,13) ? EQ 7 = a2x2
	//******* STEP 6 : PComp = y1 + a2x2
	lt1 = y1; // s(26,13)
	lt2 = a2x2; // s(25,13)
	lt3 = lt1 + lt2; // s(26,13) = y1 + a2x2
	PComp = lt3 >> 9; // s(17,4) ? EQ 8 = PComp
	return (signed int)PComp; // By calibration this is less than 16 bits
	
}



