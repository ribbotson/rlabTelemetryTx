/*____________________________________________________________________________*/
/* File: comms.c
 *
 * Created: 24/07/2014 16:29:06
 * Revised  
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
 *  This source module implements the communications interfaces of the rlab
 *  Telemetry Transmitter
 *
 *  This module supports the following interfaces
 *		: USB serial
 *		: Ext serial
 *		: GPS serial
 *		: NTX2B serial
 *		: NTX2B Modulation
 *  
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
#include "asf.h"
#include <stdio.h>
#include "comms.h"
#include "udi_cdc.h"
#include "cmnd.h"
#include "main.h"
#include "timer.h"

extern volatile bool	ntx2b_mod_busy_flag;
extern volatile unsigned char mod_state;
volatile bool	cdcRxChar;
extern FILE usbout;
char transmit_state = TRANSMIT_OFF; // state of transmitter (off, CW, beacon)



//Initialise all the communications ports

void comms_init(void)
{
	cdcRxChar = false;
	
	// Start the UARTS
	gps_uart_init();	
	ntx2b_uart_init();
	ext_uart_init();
	
	// Start the DAC
	dac_enable(&MY_DAC);
	dac_init();
	
	// Initialise the NTX2B Modulation
		
	ntx2b_mod_enable(RTTY300);

	
	
}

bool comms_cdcgotchar()
{
	return cdcRxChar;
}

void comms_cdc_clr_gotchar(void)
{
	cdcRxChar = false;
}

int comms_usb_putchar(char c, FILE *stream)
{

	if (c == '\n')
	comms_usb_putchar('\r', stream);
	udi_cdc_putc(c);
	return 0;
}

void comms_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		// Startup message to CDC Port
		printf("\nrlab.org.uk : HAB Monitor : Free as in air: ");      // output msg to serial port
		hci_put_resp_term();        // prompt
		hci_clear_command();
	}else{
		// Host terminal has close COM
		
	}
}

void comms_cdc_rx_notify(uint8_t port)
{
	cdcRxChar = true;
}

void dac_init(void){
	struct dac_config dac_conf;
	
	dac_read_configuration(&MY_DAC, &dac_conf);
	dac_set_conversion_parameters(&dac_conf, DAC_REF_BANDGAP, DAC_ADJ_RIGHT);
	dac_set_active_channel(&dac_conf, NTX2B_DAC, 0);
	dac_set_conversion_trigger(&dac_conf, 0, 1);
	dac_write_configuration(&MY_DAC, &dac_conf);
	
}

void gps_uart_init(void)
{
	static usart_rs232_options_t GPS_USART_SERIAL_OPTIONS = {
		.baudrate = GPS_USART_SERIAL_BAUDRATE,
		.charlength = GPS_USART_SERIAL_CHAR_LENGTH,
		.paritytype = GPS_USART_SERIAL_PARITY,
		.stopbits = GPS_USART_SERIAL_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_C, PR_USART0_bm);
	usart_init_rs232(GPS_USART_SERIAL, &GPS_USART_SERIAL_OPTIONS);
}



void ext_uart_init(void)
{
	static usart_rs232_options_t EXT_USART_SERIAL_OPTIONS = {
		.baudrate = EXT_USART_SERIAL_BAUDRATE,
		.charlength = EXT_USART_SERIAL_CHAR_LENGTH,
		.paritytype = EXT_USART_SERIAL_PARITY,
		.stopbits = EXT_USART_SERIAL_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_E, PR_USART0_bm);
	usart_init_rs232(EXT_USART_SERIAL, &EXT_USART_SERIAL_OPTIONS);
}



void ntx2b_uart_init(void)
{
	static usart_rs232_options_t NTX2B_USART_SERIAL_OPTIONS = {
		.baudrate = NTX2B_USART_SERIAL_BAUDRATE,
		.charlength = NTX2B_USART_SERIAL_CHAR_LENGTH,
		.paritytype = NTX2B_USART_SERIAL_PARITY,
		.stopbits = NTX2B_USART_SERIAL_STOP_BIT
	};
	sysclk_enable_module(SYSCLK_PORT_D, PR_USART0_bm);
	usart_init_rs232(NTX2B_USART_SERIAL, &NTX2B_USART_SERIAL_OPTIONS);
	usart_rx_disable(NTX2B_USART_SERIAL); // we don't use the receiver
	ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN ); // set enable low
	
}

int ntx2b_ser_putchar(char c, FILE *stream)
{

	if (c == '\n')
	ntx2b_ser_putchar('\r', stream);
	while(!usart_data_register_is_empty(NTX2B_USART_SERIAL));
	usart_putchar(NTX2B_USART_SERIAL, c);
	return 0;
}


void  ntx2b_mod_init(void)
{
	
}


void  ntx2b_mod_enable( char mode)
{
	
	// Set DAC to Space frequency
	dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_MARK);
	
	// set DAC to update on events
	struct dac_config dac_conf;
	
	dac_read_configuration(&MY_DAC, &dac_conf);
	dac_set_conversion_parameters(&dac_conf, DAC_REF_BANDGAP, DAC_ADJ_RIGHT);
	dac_set_active_channel(&dac_conf, NTX2B_DAC, 0);
	dac_set_conversion_trigger(&dac_conf, 0, 0);
	dac_write_configuration(&MY_DAC, &dac_conf);
	
	// Set DAC to Space frequency
	dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_MARK);
	
	ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	
	
}
void  ntx2b_mod_disable( void)
{
	
	// wait till end transmission
	while(ntx2b_mod_busy_flag);
	
	// set the DAC to update when data sent
	struct dac_config dac_conf;
	
	dac_read_configuration(&MY_DAC, &dac_conf);
	dac_set_conversion_parameters(&dac_conf, DAC_REF_BANDGAP, DAC_ADJ_RIGHT);
	dac_set_active_channel(&dac_conf, NTX2B_DAC, 0);
	dac_set_conversion_trigger(&dac_conf, 0, 0);
	dac_write_configuration(&MY_DAC, &dac_conf);
	
	ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
}

void  ntx2b_mod_transmit( void)
{
	ntx2b_mod_busy_flag = true;
	mod_state = 0;
	
	// start interrupts
	tc_set_overflow_interrupt_level(&MOD_BAUD_TIMER, TC_INT_LVL_LO);
}

bool  ntx2b_mod_busy(void)
{
	return ntx2b_mod_busy_flag;
}

