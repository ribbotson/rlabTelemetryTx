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


#include <asf.h>
#include "conf_usb.h"
#include  "cmnd.h"
#include "gps.h"
#include "ntx2b.h"
#include "i2c400.h"

#include <avr/pgmspace.h> 

static volatile bool main_b_cdc_enable = false;

// Welcome message

const  char  psWelcome[] PROGMEM = "\nrlab.org.uk : HAB Monitor : Free as in air: ";
/**
 * \brief Timer Counter Overflow interrupt callback function
 *
 * This function is called when an overflow interrupt has occurred on
 * MILLISEC_TIMER .
 */
static void millisec_ovf_interrupt_callback(void)
{
	millisecTimer();
}

bool    b5msecTaskReq;
bool    b50mSecTaskReq;
bool    b500msecTaskReq;
bool	cdcRxChar;
static  unsigned long  ulClockTicks;     // General-purpose "tick" counter


/*! \brief Main function. Execution starts here.
 */
int main(void)
{
	irq_initialize_vectors();
	cpu_irq_enable();

	// Initialize the sleep manager
	sleepmgr_init();
	adc_init();
	dac_init();
	sysclk_init();
	board_init();
	i2c400_init();
	
	LED_Off(LED0_GPIO);
	
	// start the millisecond timer
	tc_enable(&MILLISEC_TIMER);
	tc_set_overflow_interrupt_callback(&MILLISEC_TIMER,
	millisec_ovf_interrupt_callback);
	tc_set_wgm(&MILLISEC_TIMER, TC_WG_NORMAL);
	tc_write_period(&MILLISEC_TIMER, MILLISEC_TIMER_PERIOD); //timer is 1ms
	tc_set_overflow_interrupt_level(&MILLISEC_TIMER, TC_INT_LVL_LO);
	tc_set_resolution(&MILLISEC_TIMER, MILLISEC_TIMER_RESOLUTION); // set the timer resolution to 1us
	
	// Start USB stack to authorize VBus monitoring
	udc_start();
	cdcRxChar = FALSE;
	
	// Start the ADC
	adc_enable(&MY_ADC);
	
	// Start the DAC
	dac_enable(&MY_DAC);
	
	// Start the UARTS
	gps_uart_init();
	usart_tx_enable(GPS_USART_SERIAL);
	usart_rx_enable(GPS_USART_SERIAL);
	ntx2b_uart_init();
	ext_uart_init();
	usart_tx_enable(EXT_USART_SERIAL);
	usart_rx_enable(EXT_USART_SERIAL);
	se95_init();
	
	gps_init();
	
	// The main loop 
	
	while (true) {
		if(cdcRxChar){
			while(udi_cdc_is_rx_ready()){
				 hci_process_input( udi_cdc_getc());
				 
			}				 
			cdcRxChar = FALSE;
		}
		get_gps_data();
		sleepmgr_enter_sleep();
	}
}

void main_suspend_action(void)
{
	
}

void main_resume_action(void)
{
	
}

void main_sof_action(void)
{
	if (!main_b_cdc_enable)
		return;
	
}

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Open communication
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		// Startup message to CDC Port
		cdcPutstr_P( psWelcome );      // output msg to serial port
		hci_put_resp_term();        // prompt
		hci_clear_command();
	}else{
		// Host terminal has close COM
		
	}
}

void main_cdc_rx_notify(uint8_t port)
{
	cdcRxChar = TRUE;
}


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

void dac_init(void){
	 struct dac_config dac_conf;
	 
	 dac_read_configuration(&MY_DAC, &dac_conf);
	 dac_set_conversion_parameters(&dac_conf, DAC_REF_BANDGAP, DAC_ADJ_RIGHT);
	 dac_set_active_channel(&dac_conf, NTX2B_DAC, 0);
	 dac_set_conversion_trigger(&dac_conf, 0, 0);
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


/*
|   TIMER CALLBACK ---
|   Timer C0.
|   RTI "Tick Handler" / task scheduler.
|   Short time-critical periodic tasks may be called within this ISR;
|   other periodic tasks are scheduled for execution in "background".
|   (See doBackgroundTasks() in main.c)
*/
void millisecTimer(void)
{
	static  char   b500msecTimer = 0;
	static  char   b50mSecTimer = 0;
	static  char   b5mSecTimer = 0;

	ulClockTicks++;

	if ( ++b5mSecTimer >= 5 )
	{
		b5msecTaskReq = 1;
		b5mSecTimer = 0;
		b50mSecTimer++ ;
	}
	if ( b50mSecTimer >= 10 )
	{
		b50mSecTaskReq = 1;
		b50mSecTimer = 0;
		b500msecTimer++ ;
	}
	if ( b500msecTimer >= 10 )
	{
		b500msecTaskReq = 1;
		b500msecTimer = 0;
		LED_Toggle(LED0_GPIO);
		
	}
}

unsigned long main_get_msclock_ticks(void)
{
	return (ulClockTicks);
}