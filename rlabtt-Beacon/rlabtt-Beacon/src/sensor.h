/*____________________________________________________________________________*/
/* File: sensor.h
 *
 * Created: 07/06/2014 16:01:06
 * Revised  02/08/2014 16:55
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
  *  This include file implements the sensor for rlab Telemetry Transmitter
  * including the Global Positioning System (GPS) Interface
  *
  *  This module communicates with the ublox Max7q GPS module over an I2C two wire interface
  *  NMEA messages are parsed as received to global parameters for use in telemetry messages.
  *  This module supports the other devices on the I2C bus, temperature and pressure sensor.
  *  Also support the Analog to digital converters for battery monitoring
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
#include <compiler.h>

#define GPS_BUFFER_SIZE		64
enum {GPS_SENTENCE_GPGGA, GPS_SENTENCE_GPRMC, GPS_SENTENCE_OTHER};
enum {POWER_UNKNOWN, POWER_USB, POWER_BATTERY, POWER_BOTH};
	
		
#define TWI_I2C400          TWIC
#define GPSI2C_ADDRESS		0x42
#define MPL115I2C_ADDRESS	0x60
#define SE95I2C_ADDRESS		0x48
#define I2C400_ADDRESS	    0x00
#define I2C400_SPEED        400000ul

#define POWER_OK_VOLTS 3750 // power OK level in ADC counts
#define ADC_TO_VOLTS

void i2c400_init(void);
void se95_init(void);
void mpl115_convert(void);
signed int se95_read(void);
int mpl115_read_pressure(void);

void adc_init(void);

void check_power_source(void);


struct TGPS {
	 unsigned long	Time; // UTC Time in 100ths of sec
	 long			Date;
	 signed long	Latitude;
	 signed long	Longitude;
	 long			Altitude;
	 long			Speed;
	 signed long	Direction;
	 char			Satellites;
	 signed int		Temperature; // in 100th deg C
	 int			Pressure; // in 10th kPa
	 int			Battery;
	 
 };
void sensor_init(void);
void SendUBX(unsigned char *MSG, int len);
void SetFlightMode(void);
void get_gps_data(void);
int gps_from_hex(unsigned char);
int gpsstrcmp(const unsigned char *, const unsigned char *);
long gpsatol(const unsigned char *);
bool gpsisdigit(unsigned char c);
long gps_parse_decimal(void);
long gps_parse_degrees(void);
long get_gps_date(void);
long get_gps_time(void);
long get_gps_longitude(void);
long get_gps_latitude(void);
long get_gps_altitude(void);
long get_gps_speed(void);
long get_gps_direction(void);	
char get_gps_satellites(void);
signed int get_gps_temperature(void);
int get_gps_pressure(void);
int get_gps_battery(void);
void set_gps_temperature(signed int);
void set_gps_pressure(int);
void set_gps_battery(int);
void parse_gps_term(void);
void i2c400_init(void);
void se95_init(void);
void mpl115_convert(void);
signed int se95_read(void);
int mpl115_read_pressure(void);
#endif /* GPS_H_ */