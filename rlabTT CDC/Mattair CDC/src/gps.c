/*____________________________________________________________________________*/
/* File: gps.c
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
/*____________________________________________________________________________*/

#include "gps.h"
#include "i2c400.h"
#include "cmnd.h"


#define GPRMC_TERM   "GPRMC"
#define GPGGA_TERM   "GPGGA"

//Storage for Parsed GPS Information
unsigned long gps_time = GPS_INVALID_TIME, gps_new_time;
unsigned long gps_date = GPS_INVALID_DATE, gps_new_date;
long gps_latitude = GPS_INVALID_ANGLE, gps_new_latitude;
long gps_longitude = GPS_INVALID_ANGLE, gps_new_longitude;
long gps_altitude = GPS_INVALID_ALTITUDE, gps_new_altitude;
unsigned long  gps_speed = GPS_INVALID_SPEED, gps_new_speed;
unsigned long gps_course = GPS_INVALID_ANGLE, gps_new_course;
unsigned long  gps_hdop = GPS_INVALID_HDOP, gps_new_hdop;
unsigned short gps_numsats = GPS_INVALID_SATELLITES, gps_new_numsats;

unsigned char gps_term[15];
unsigned char gps_term_offset, gps_term_number;

bool gps_checksum_term, gps_data_good;

unsigned long gps_last_time_fix, gps_new_time_fix;
unsigned long gps_last_position_fix, gps_new_position_fix;
unsigned char gps_sentence_type, gps_parity;
unsigned char gpsbuf[GPS_BUFFER_SIZE];

//Package to read GPS bytes available count
twi_package_t getgps_count = {
	.addr_length = 1, 
	.addr		 = {0xfd, 0x00, 0x00},
	.chip        = GPSI2C_ADDRESS, 
	.buffer      = (void *) &gpsbuf[0],
	.length      = 2,
	.no_wait     = false
};

//Package to read GPS data bytes

twi_package_t getgps_data = {
	.addr_length = 1,
	.addr		 = {0xff, 0x00, 0x00},
	.chip        = GPSI2C_ADDRESS,
	.buffer      = (void *) &gpsbuf[0],
	.length      = GPS_BUFFER_SIZE,
	.no_wait     = false
};

void gps_init(void)
{
	// Stop NMEA messages we don't need
	
	
}
	

void get_gps_data(void) {
	status_code_t gpsreturn;
	
	unsigned int bytes2Read;
	unsigned char gps_raw_pointer, gps_raw_data;
	
	
	gpsreturn = twi_master_read(&TWI_I2C400, &getgps_count);
	if (gpsreturn) return; // got some TWI error. Return
	
	bytes2Read =  (gpsbuf[0] << 8) | gpsbuf[1];
	if (!bytes2Read) return; // GPS not ready to send data. Return
	if (bytes2Read > 64) bytes2Read = 64;
	getgps_data.length = bytes2Read;
	gpsreturn = twi_master_read(&TWI_I2C400, &getgps_data );
	
	for(gps_raw_pointer =0; gps_raw_pointer < bytes2Read; gps_raw_pointer++)
	{
		gps_raw_data = gpsbuf[gps_raw_pointer];
//		cdcPutch(gps_raw_data);
		switch(gps_raw_data)
		{
			case ',': // term terminators				
			case '\r':
			case '\n':
			case '*':
				if (gps_term_offset < sizeof(gps_term))
				{
					gps_term[gps_term_offset] = 0;
					parse_gps_term();
				}
				++gps_term_number;
				gps_term_offset = 0;
				gps_checksum_term = (gps_raw_data == '*');
				if(gps_raw_data == ',') gps_parity ^= gps_raw_data;
				break;
			
			case '$': // sentence begin
				gps_term_number = gps_term_offset = 0;
				gps_parity = 0;
				gps_sentence_type = GPS_SENTENCE_OTHER;
				gps_checksum_term = false;
				gps_data_good = false;
				break;
				
			default:
				// ordinary characters
				if (gps_term_offset < sizeof(gps_term) - 1)
				gps_term[gps_term_offset++] = gps_raw_data;
				if (!gps_checksum_term)
				gps_parity ^= gps_raw_data;
		}

		
	}
	
}

int gps_from_hex(unsigned char a)
{
	if (a >= 'A' && a <= 'F')
	return a - 'A' + 10;
	else if (a >= 'a' && a <= 'f')
	return a - 'a' + 10;
	else
	return a - '0';
}


int gpsstrcmp(const unsigned char *str1, const unsigned char *str2)
{
	while (*str1 && *str1 == *str2)
	++str1, ++str2;
	return *str1;
}

bool gpsisdigit(unsigned char c)
{
	 return c >= '0' && c <= '9';
 }

long gpsatol(const unsigned char *str)
{
	long ret = 0;
	while (gpsisdigit(*str))
	ret = 10 * ret + *str++ - '0';
	return ret;
}

unsigned long gps_parse_decimal()
{
	unsigned char *p = gps_term;
	bool isneg = *p == '-';
	if (isneg) ++p;
	unsigned long ret = 100UL * gpsatol(p);
	while (gpsisdigit(*p)) ++p;
	if (*p == '.')
	{
		if (gpsisdigit(p[1]))
		{
			ret += 10 * (p[1] - '0');
			if (gpsisdigit(p[2]))
			ret += p[2] - '0';
		}
	}
	return isneg ? -ret : ret;
}

// Parse a string in the form ddmm.mmmmmmm...
unsigned long gps_parse_degrees()
{
	unsigned char *p;
	unsigned long left_of_decimal = gpsatol(gps_term);
	unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
	for (p=gps_term; gpsisdigit(*p); ++p);
	if (*p == '.')
	{
		unsigned long mult = 10000;
		while (gpsisdigit(*++p))
		{
			hundred1000ths_of_minute += mult * (*p - '0');
			mult /= 10;
		}
	}
	return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

/* Parse a NMEA GPS string in the GPS string buffer

*/

void parse_gps_term(void)
{
	 if (gps_checksum_term)
	 {
		 unsigned char checksum = 16 * gps_from_hex(gps_term[0]) + gps_from_hex(gps_term[1]);
		 if (checksum == gps_parity)
		 {
			 if (gps_data_good)
			 {
				gps_last_time_fix = gps_new_time_fix;
				gps_last_position_fix = gps_new_position_fix;

				 switch(gps_sentence_type)
				 {
					 case GPS_SENTENCE_GPRMC:
					 gps_time      = gps_new_time;
					 gps_date      = gps_new_date;
					 gps_latitude  = gps_new_latitude;
					 gps_longitude = gps_new_longitude;
					 gps_speed     = gps_new_speed;
					 gps_course    = gps_new_course;
					 break;
					 case GPS_SENTENCE_GPGGA:
					 gps_altitude  = gps_new_altitude;
					 gps_time      = gps_new_time;
					 gps_latitude  = gps_new_latitude;
					 gps_longitude = gps_new_longitude;
					 gps_numsats   = gps_new_numsats;
					 gps_hdop      = gps_new_hdop;
					 break;
				 }

				 return;
			 }
		 }

		 
		 return;
	 }

	 // the first term determines the sentence type
	 if (gps_term_number == 0)
	 {
		 if (!gpsstrcmp(gps_term, GPRMC_TERM))
		 gps_sentence_type = GPS_SENTENCE_GPRMC;
		 else if (!gpsstrcmp(gps_term, GPGGA_TERM))
		 gps_sentence_type = GPS_SENTENCE_GPGGA;
		 else
		 gps_sentence_type = GPS_SENTENCE_OTHER;
		 return;
	 }

	 if (gps_sentence_type != GPS_SENTENCE_OTHER && gps_term[0])
	 switch(COMBINE(gps_sentence_type, gps_term_number))
	 {
		 case COMBINE(GPS_SENTENCE_GPRMC, 1): // Time in both sentences
		 case COMBINE(GPS_SENTENCE_GPGGA, 1):
		 gps_new_time = gps_parse_decimal();
		 gps_new_time_fix = main_get_msclock_ticks();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 2): // GPRMC validity
		 gps_data_good = gps_term[0] == 'A';
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 3): // Latitude
		 case COMBINE(GPS_SENTENCE_GPGGA, 2):
		 gps_new_latitude = gps_parse_degrees();
		 gps_new_position_fix =  main_get_msclock_ticks();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 4): // N/S
		 case COMBINE(GPS_SENTENCE_GPGGA, 3):
		 if (gps_term[0] == 'S')
		 gps_new_latitude = -gps_new_latitude;
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 5): // Longitude
		 case COMBINE(GPS_SENTENCE_GPGGA, 4):
		 gps_new_longitude = gps_parse_degrees();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 6): // E/W
		 case COMBINE(GPS_SENTENCE_GPGGA, 5):
		 if (gps_term[0] == 'W')
		 gps_new_longitude = -gps_new_longitude;
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
		 gps_new_speed = gps_parse_decimal();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
		 gps_new_course = gps_parse_decimal();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
		 gps_new_date = gpsatol(gps_term);
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
		 gps_data_good = gps_term[0] > '0';
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
		 gps_new_numsats = (unsigned char)atoi(gps_term);
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 8): // HDOP
		 gps_new_hdop = gps_parse_decimal();
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
		 gps_new_altitude = gps_parse_decimal();
		 break;
	 }

}

unsigned long get_gps_date(void)
{
	return gps_date;
}

unsigned long get_gps_time(void)
{
	return gps_time;
}
	
unsigned long get_gps_longitude(void)
{
	return gps_longitude;
}
unsigned long get_gps_latitude(void)
{
	return gps_latitude;
}

unsigned long get_gps_altitude(void)
{
	return gps_altitude;
}