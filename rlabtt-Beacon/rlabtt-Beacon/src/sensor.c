/*____________________________________________________________________________*/
/* File: sensor.c
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
 *  This source module implements the sensor for rlab Telemetry Transmitter
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
/*____________________________________________________________________________*/

#include "sensor.h"
#include "cmnd.h"
#include "timer.h"


#define GPRMC_TERM   "GPRMC"
#define GPGGA_TERM   "GPGGA"

enum {GPS_NO_SAT, GPS_1D, GPS_3D};

//Storage for Parsed GPS Information

struct TGPS	new_gps_data; // structure to build new GPS data in
struct TGPS valid_gps_data; // pointer to where good GPS data is to be put

unsigned char gps_term[15];
unsigned char gps_term_offset, gps_term_number;

bool gps_checksum_term, gps_data_good;

unsigned long gps_last_time_fix, gps_new_time_fix;
unsigned long gps_last_position_fix, gps_new_position_fix;
unsigned char gps_sentence_type, gps_parity;
unsigned char gpsbuf[GPS_BUFFER_SIZE];
char gps_status = GPS_NO_SAT; // status of GPS data

char power_status = POWER_UNKNOWN; // source of power (unknown, USB, battery, both)

twi_options_t m_options = {		.speed     = I2C400_SPEED,
	.chip      = I2C400_ADDRESS,
	
};

unsigned char i2c_buffer[16];

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

//Package to write UBX GPS data bytes

twi_package_t send_ubx = {
	.addr_length = 0,
	.addr		 = {0x00, 0x00, 0x00},
	.chip        = GPSI2C_ADDRESS,
	.buffer      = (void *) NULL,
	.length      = 0,
	.no_wait     = false
};

void adc_init(void)
{    struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	
	adc_read_configuration(&MY_ADC, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_BANDGAP);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_FREERUN_SWEEP, 2, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&MY_ADC, &adc_conf);
	
	adcch_read_configuration(&MY_ADC,VBATT_ADC_CH, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN0, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC, VBATT_ADC_CH, &adcch_conf);
	
	adcch_read_configuration(&MY_ADC,VUSB_ADC_CH, &adcch_conf);
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1);
	adcch_write_configuration(&MY_ADC, VUSB_ADC_CH, &adcch_conf);
}

void check_power_source(void)
{
	
	// check what is the power source
	if (adc_get_result(&MY_ADC, VBATT_ADC_CH) > POWER_OK_VOLTS)
	{
		if (adc_get_result(&MY_ADC, VUSB_ADC_CH) > POWER_OK_VOLTS) power_status = POWER_BOTH;
		power_status = POWER_BATTERY;
	}
	else if (adc_get_result(&MY_ADC, VUSB_ADC_CH) > POWER_OK_VOLTS) power_status = POWER_USB;
	else power_status = POWER_UNKNOWN;
}

void sensor_init(void)
{
	// start the ADC
	adc_init();
	
	// Initialise the I2C bus
	i2c400_init();
	
	// Initialise the valid  GPS structure
	
	valid_gps_data.Time = 0;
	valid_gps_data.Date = 0;
	valid_gps_data.Latitude = 0;
	valid_gps_data.Longitude = 0;
	valid_gps_data.Altitude = 0;
	valid_gps_data.Speed = 0;
	valid_gps_data.Direction = 0;
	valid_gps_data.Satellites = 0;
	valid_gps_data.Temperature = 0;
	valid_gps_data.Pressure = 0;
	valid_gps_data.Battery = 0;
	
	// Send our GPS Flight mode string
	SetFlightMode();
	
	// Initialise other I2C devices
	se95_init();	// Thermometer
}
	
void SendUBX(unsigned char *MSG, int len)
{
	send_ubx.buffer = MSG;
	send_ubx.length = len;
	
	twi_master_write(&TWI_I2C400, &send_ubx);
}

void SetFlightMode(void)
{
	// Send navigation configuration command
	unsigned char setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
	SendUBX(setNav, sizeof(setNav));
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

long gps_parse_decimal()
{
	unsigned char *p = gps_term;
	bool isneg = *p == '-';
	if (isneg) ++p;
	long ret = 100UL * gpsatol(p);
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
long gps_parse_degrees()
{
	unsigned char *p;
	long left_of_decimal = gpsatol(gps_term);
	long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
	for (p=gps_term; gpsisdigit(*p); ++p);
	if (*p == '.')
	{
		long mult = 10000;
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
					 valid_gps_data.Time =  new_gps_data.Time;
					 valid_gps_data.Date =  new_gps_data.Date;
					 valid_gps_data.Latitude =  new_gps_data.Latitude;
					 valid_gps_data.Longitude =  new_gps_data.Longitude;
					 valid_gps_data.Speed =  new_gps_data.Speed;
					 valid_gps_data.Direction =  new_gps_data.Direction;
					 break;
					 
					 case GPS_SENTENCE_GPGGA:
					 valid_gps_data.Altitude =  new_gps_data.Altitude;
					 valid_gps_data.Time =  new_gps_data.Time;
					 valid_gps_data.Latitude =  new_gps_data.Latitude;
					 valid_gps_data.Longitude =  new_gps_data.Longitude;
					 valid_gps_data.Satellites =  new_gps_data.Satellites;
					 valid_gps_data.Pressure = mpl115_read_pressure();
					 valid_gps_data.Temperature = se95_read();
					 valid_gps_data.Battery = adc_get_result(&MY_ADC, VBATT_ADC_CH);
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
		 new_gps_data.Time = gps_parse_decimal();
		 gps_new_time_fix = main_get_msclock_ticks();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 2): // GPRMC validity
		 gps_data_good = gps_term[0] == 'A';
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 3): // Latitude
		 case COMBINE(GPS_SENTENCE_GPGGA, 2):
		 new_gps_data.Latitude = gps_parse_degrees();
		 gps_new_position_fix =  main_get_msclock_ticks();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 4): // N/S
		 case COMBINE(GPS_SENTENCE_GPGGA, 3):
		 if (gps_term[0] == 'S')
		 new_gps_data.Latitude = -new_gps_data.Latitude;
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 5): // Longitude
		 case COMBINE(GPS_SENTENCE_GPGGA, 4):
		 new_gps_data.Longitude = gps_parse_degrees();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 6): // E/W
		 case COMBINE(GPS_SENTENCE_GPGGA, 5):
		 if (gps_term[0] == 'W')
		 new_gps_data.Longitude = -new_gps_data.Longitude;
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
		 new_gps_data.Speed = gps_parse_decimal();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 8): // Direction (GPRMC)
		 new_gps_data.Direction = gps_parse_decimal();
		 break;
		 case COMBINE(GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
		 new_gps_data.Date = gpsatol(gps_term);
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
		 gps_data_good = gps_term[0] > '0';
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
		 new_gps_data.Satellites = (unsigned char)atoi(gps_term);
		 break;
		 case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
		 new_gps_data.Altitude = gps_parse_decimal();
		 break;
	 }

}
long get_gps_date(void)
{
	return valid_gps_data.Date;
}

long get_gps_time(void)
{
	return valid_gps_data.Time;
}
	
long get_gps_longitude(void)
{
	return valid_gps_data.Longitude;
}
long get_gps_latitude(void)
{
	return valid_gps_data.Latitude;
}

long get_gps_altitude(void)
{
	return valid_gps_data.Altitude;
}

long get_gps_direction(void)
{
	return valid_gps_data.Direction;
}
long get_gps_speed(void)
{
	return valid_gps_data.Speed;
}


char get_gps_satellites(void)
{
	return valid_gps_data.Satellites;
}

signed int get_gps_temperature(void)
{
	return valid_gps_data.Temperature;
}

int get_gps_pressure(void)
{
	return valid_gps_data.Pressure;
}

int get_gps_battery(void)
{
	return valid_gps_data.Battery;
}

void set_gps_temperature(signed int temperature)
{
	valid_gps_data.Temperature = temperature;
}

void set_gps_pressure(int pressure)
{
	valid_gps_data.Pressure = pressure;
}

void set_gps_battery(int battery)
{
	valid_gps_data.Battery = battery;
}

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
	signed long temperature = ((((signed long)((i2c_buffer[0] << 8) | i2c_buffer[1])) * 100l)  / 256l);
	return (signed int) temperature;
}

int mpl115_read_pressure(void)
{
	unsigned int Padc, Tadc;
	signed int a0, b1, b2, c12;
	signed long lt1, lt2, lt3;
	signed long c12x2, a1, a1x1, y1, a2x2, PComp;

	mpl115_convert();
	delay_ms(4);
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
	PComp = ((PComp  * 1041) >> 14) + 800;
	PComp = (PComp * 10) / 16;
	return (int)PComp; // By calibration this is less than 16 bits
	
}
