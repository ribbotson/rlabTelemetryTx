/*____________________________________________________________________________*/
/*
 * File: main.c
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
 *  This source module implements the main functions of the rlab Telemetry
 *  Transmitter
 *
 *
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


#include "asf.h"
#include "string.h"  	// String function definitions
#include "stdio.h"
#include "avr/pgmspace.h"
#include "main.h"
#include "conf_usb.h"
#include "cmnd.h"
#include "sensor.h"
#include "comms.h"
#include "timer.h"
#include "nvconfig.h"

// Global Data to describe the current firmware build
int build_major = 0;
int build_minor = 1;
int build_release = 0;
char * build_date =  (char *) __DATE__ ;

// mapping of files for fprintf
FILE usbout = FDEV_SETUP_STREAM(comms_usb_putchar, NULL,_FDEV_SETUP_WRITE);
FILE extout =FDEV_SETUP_STREAM(ext_ser_putchar, NULL,_FDEV_SETUP_WRITE);
FILE ntx2bout =FDEV_SETUP_STREAM(ntx2b_ser_putchar, NULL,_FDEV_SETUP_WRITE);

// external Status Variables

extern char transmit_state;
extern char power_status;
// Global configuration
struct MAGIC magic = {.magic_number= EEPROM_MAGIC_NUMBER, .config_version = EEPROM_CONFIG_VERSION};
	
struct CONFIG config = {.uart_on_startup = START_UART_AUTO,
						.beacon_repeat_time = 10,
						.payload_id = "rlabtt",
						.modulation_mode = RTTY300,
						.tx_channel = 1,
						.tx_trim = 0x40,
						.cutdown_altitude = 10000.0 };

// String to be sent as beacon and its length
char		beacon_string[100];
unsigned char	beacon_length;

char serial_port = SERIAL_NONE;

// External Task request flags
extern volatile bool    b500msecTaskReq;
extern volatile bool	 bBeaconTaskReq;


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
		
	irq_initialize_vectors();
	cpu_irq_enable();
	
	
	pmic_init();

	// Initialize the sleep manager
	sleepmgr_init();
	
	
	
	sysclk_init();
	
	// set the board to an initial safe state
	board_init();
	
	// get the configuration from eeprom if there else write it
	
	get_nvmagic(&magic); // get magic number structure from EEPROM
	if (magic.magic_number == EEPROM_MAGIC_NUMBER && magic.config_version == EEPROM_CONFIG_VERSION)
	{
		get_nvconfig(&config);
	}
	else
	{
		set_nvmagic(&magic);
		set_nvconfig(&config);
	}
	

	//Initialise the delay system
	delay_init(sysclk_get_cpu_hz())
	
	// default led to off
	LED_Off(LED0_GPIO);
	
	// Start the ADC for power monitoring
	adc_enable(&MY_ADC);
	
	// Initialise GPS and I2C devices
	sensor_init();
	
	// Start all the comms ports
	comms_init();
	
	
	// find out source of power
	check_power_source();
	
	if(config.uart_on_startup == START_UART_USB)
	{
		if(power_status == POWER_USB || power_status == POWER_BOTH)
		{
			// Start USB stack
			udc_start(); // should check if really started
			serial_port = SERIAL_USB;
			// map stdout
			stdout = &usbout;
		}
	}
	
	else if(config.uart_on_startup == START_UART_AUTO)
	{
		if(power_status == POWER_USB || power_status == POWER_BOTH)
		{
			// Start USB stack
			udc_start();
			serial_port = SERIAL_USB;
			// map stdout
			stdout = &usbout;
		}
		else
		{
			serial_port = SERIAL_EXT;
			// map stdout
			stdout = &extout;
			
		}
	}
	
	else if(config.uart_on_startup == START_UART_EXT)
	{
		serial_port = SERIAL_EXT;
		// map stdout
		stdout = &extout;
	}		
	
	
	
	
	// for testing startup
		
	LED_On(LED0_GPIO);
	
	
	

	
	
	
	
	
	// start the timers
	timer_init();
	
	// The main loop 
	
	while (true) {
		if(comms_cdcgotchar()){
			while(udi_cdc_is_rx_ready()){
				 hci_process_input( udi_cdc_getc());
				 
			}				 
			comms_cdc_clr_gotchar();
		}
		if(b500msecTaskReq)
		{
			b500msecTaskReq = false;
			LED_Toggle(LED0_GPIO);
			check_power_source();		
		}
		
		if(bBeaconTaskReq)
		{
			bBeaconTaskReq = false;
			if(transmit_state == TRANSMIT_BEACON)
			{
				if(!ntx2b_mod_busy())
				{
					
					 BuildSentence(beacon_string);
					 printf("Beacon String: %s", beacon_string); 
					 ntx2b_mod_transmit();
				}
				
			}
		}
		get_gps_data();
		sleepmgr_enter_sleep();
	}
}








void BuildSentence(char *beacon_string)
{
	int Count, i, j;
	unsigned int myCRC;
	char TimeBuffer[10];
	static unsigned long SentenceCounter;
	dt_format(TimeBuffer, get_gps_time());
	sprintf(beacon_string, "$$%s,%ld,%s,%08ld,%08ld,%08ld,%08ld,%08ld,%d,%d, %d, %d",
	config.payload_id,
	SentenceCounter++,
	TimeBuffer,
	get_gps_latitude(),
	get_gps_longitude(),
	get_gps_altitude(),
	(get_gps_speed() * 13) / 7,
	get_gps_direction(),
	get_gps_satellites(),
	get_gps_temperature(),
	get_gps_pressure(),
	get_gps_battery());
	
	Count = strlen(beacon_string);

	myCRC = 0xffff;           // Seed
	
	
	for (i = 2; i < Count; i++)
	{   // For speed, repeat calculation instead of looping for each bit
		myCRC ^= (((unsigned int)beacon_string[i]) << 8);
		for (j=0; j<8; j++)
		{
			if (myCRC & 0x8000)
			myCRC = (myCRC << 1) ^ 0x1021;
			else
			myCRC <<= 1;
		}
	}

	beacon_string[Count++] = '*';
	beacon_string[Count++] = Hex((myCRC >> 12) & 15);
	beacon_string[Count++] = Hex((myCRC >> 8) & 15);
	beacon_string[Count++] = Hex((myCRC >> 4) & 15);
	beacon_string[Count++] = Hex(myCRC & 15);
	beacon_string[Count++] = '\n';
	beacon_string[Count++] = '\0';

	beacon_length = strlen( (char *)beacon_string);
	
}

char Hex(unsigned char Character)
{
	char HexTable[] = "0123456789ABCDEF";
	
	return HexTable[Character];
}




