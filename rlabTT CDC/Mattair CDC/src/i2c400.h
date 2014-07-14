/*____________________________________________________________________________*/
/*
 * File: i2c400.h
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



#ifndef I2C400_H_
#define I2C400_H_

#define TWI_I2C400          TWIC
#define GPSI2C_ADDRESS		0x42
#define MPL115I2C_ADDRESS	0x60
#define SE95I2C_ADDRESS		0x48
#define I2C400_ADDRESS	    0x00
#define I2C400_SPEED        400000ul


void i2c400_init(void);
void se95_init(void);
void mpl115_convert(void);
signed int se95_read(void);
signed int mpl115_read_pressure(void);


#endif /* I2C400_H_ */