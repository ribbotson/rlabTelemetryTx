
/*____________________________________________________________________________*/
/* File: timer.c
 *
 * Created: 02/08/2014 16:01:06
 * Revised  02/08/2014 16:55
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 *  This source module implements timers used by the tansmitter
 *
 *  Timer include:
 *	* Millisecond interrupt timer
 *	* Transmitter Modulation baud rate timer
 *	* Timeout timer
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

#include <asf.h>
#include "timer.h"
#include "comms.h"
#include "nvconfig.h"

volatile bool    b5msecTaskReq;
volatile bool    b50mSecTaskReq;
volatile bool    b500msecTaskReq;
volatile bool	 bBeaconTaskReq;

volatile  unsigned long  ulClockTicks;     // General-purpose "tick" counter

volatile unsigned char mod_state = 0;
volatile bool	ntx2b_mod_busy_flag = false; // transmission in process


extern unsigned char beacon_repeat_time;
extern char		beacon_string[100];
extern unsigned char	beacon_length;
// External configuration
extern struct NVCONFIG nvconfig;

void timer_init(void)
{
	evsys_init();

	// start the millisecond timer
	tc_enable(&MILLISEC_TIMER);
	tc_set_overflow_interrupt_callback(&MILLISEC_TIMER,	millisecTimer);
	tc_set_wgm(&MILLISEC_TIMER, TC_WG_NORMAL);
	tc_write_period(&MILLISEC_TIMER, MILLISEC_TIMER_PERIOD);
	tc_set_overflow_interrupt_level(&MILLISEC_TIMER, TC_INT_LVL_LO);
	tc_set_resolution(&MILLISEC_TIMER, MILLISEC_TIMER_RESOLUTION); // set the timer resolution to 250ns


	// start the modulation baud rate timer, This create both interrupts and events
	tc_enable(&MOD_BAUD_TIMER);
	tc_set_overflow_interrupt_callback(&MOD_BAUD_TIMER,	mod_baudTimer);
	tc_set_wgm(&MOD_BAUD_TIMER, TC_WG_NORMAL);
	tc_write_period(&MOD_BAUD_TIMER, MOD_BAUD_TIMER_PERIOD);
	tc_set_overflow_interrupt_level(&MOD_BAUD_TIMER, TC_INT_LVL_OFF); // interrupt off
	tc_set_resolution(&MOD_BAUD_TIMER, MOD_BAUD_TIMER_RESOLUTION); // set the timer resolution to 2us
}

// Set up Event system channel 0 for Timer C1 to DAC routing
void evsys_init(void)
{
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_EVSYS);
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC1_OVF_gc;
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
	static	char   bBeaconTimer = 0;

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
		bBeaconTimer++;
	}
	if ( bBeaconTimer >= nvconfig.beacon_repeat_time )
	{
		bBeaconTaskReq = 1;
		bBeaconTimer = 0;
	}
}

unsigned long main_get_msclock_ticks(void)
{
	return (ulClockTicks);
}

/*
|   TIMER CALLBACK ---
|   Timer C1. Modulation Baud rate timer
|

*/
void mod_baudTimer(void)
{
	static volatile unsigned char beacon_pointer;
	static volatile unsigned char bit_pointer;
	static volatile unsigned char beacon_char;
	
	
	
	switch(mod_state) {
		
		case 0: // sync to baud clock
		mod_state = 1;
		break;
		
		case 1: // marking preamble
		dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_MARK); //stop bit
		mod_state = 2;
		break;
		
		case 2: // marking preamble /stop bit
		mod_state = 3;
		break;
		
		case 3: // Grab a char and lets go transmit it.
		if ( beacon_pointer < beacon_length)
		{
			beacon_char = beacon_string[beacon_pointer++];
			mod_state = 4;
			dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_SPACE); //start bit
			bit_pointer = 0;
			
		}
		else
		{
			mod_state=0; // finished
			ntx2b_mod_busy_flag = false;
			tc_set_overflow_interrupt_level(&MOD_BAUD_TIMER, TC_INT_LVL_OFF);
			beacon_pointer = 0;
		}
		break;
		
		case 4: // sending character
		if(bit_pointer < 8)
		{
			bit_pointer++;
			if (beacon_char & 1) dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_MARK);
			else dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_SPACE);
			beacon_char >>= 1;
			break;
		}
		else
		{
			dac_set_channel_value(&MY_DAC, NTX2B_DAC, RTTY300_MARK); //stop bit
			mod_state = 2;
			break;
		}
		
	}

}