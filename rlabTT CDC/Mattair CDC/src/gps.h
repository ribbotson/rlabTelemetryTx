/*____________________________________________________________________________*/
/* File: gps.h
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
 *  This source module implements the Global Positioning System (GPS) Interface
 *
 *  This module communicates with the ublox Max7q GPS module over an I2C two wire interface
 *  NMEA messages are parsed as received to global parameters for use in telemetry messages.
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



#ifndef GPS_H_
#define GPS_H_

#include <asf.h>

#define GPS_BUFFER_SIZE		64
enum {GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER};
	
 enum {
	 GPS_INVALID_AGE = 0xFFFFFFFF,      GPS_INVALID_ANGLE = 999999999,
	 GPS_INVALID_ALTITUDE = 999999999,  GPS_INVALID_DATE = 0,
	 GPS_INVALID_TIME = 0xFFFFFFFF,		 GPS_INVALID_SPEED = 999999999,
	 GPS_INVALID_FIX_TIME = 0xFFFFFFFF, GPS_INVALID_SATELLITES = 0xFF,
	 GPS_INVALID_HDOP = 0xFFFFFFFF
 };
 	
void gps_init(void);
void get_gps_data(void);
int gps_from_hex(unsigned char);
int gpsstrcmp(const unsigned char *, const unsigned char *);
long gpsatol(const unsigned char *);
bool gpsisdigit(unsigned char c);
unsigned long gps_parse_decimal(void);
unsigned long gps_parse_degrees(void);
unsigned long get_gps_date(void);
unsigned long get_gps_time(void);
unsigned long get_gps_longitude(void);
unsigned long get_gps_latitude(void);
unsigned long get_gps_altitude(void);
void parse_gps_term(void);
#endif /* GPS_H_ */