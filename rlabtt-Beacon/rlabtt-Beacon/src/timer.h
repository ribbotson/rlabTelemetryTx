/*____________________________________________________________________________*/
/* File: timer.h
 *
 * Created: 02/08/2014 16:01:06
 * Revised  02/08/2014 16:55
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
 *  This include file implements timers used by the tansmitter
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


#ifndef TIMER_H_
#define TIMER_H_

void timer_init(void);

void evsys_init(void);

unsigned long main_get_msclock_ticks(void);

void millisecTimer(void);

void mod_baudTimer(void);

/* MILLISEC_TIMER is millisecond timer
 * Define which timer to use
 */
#define MILLISEC_TIMER        TCC0


/*
*This example runs at 24MHz peripheral clock
use 333ns resolution ( divide by 8)
*/
#define MILLISEC_TIMER_RESOLUTION	3000000

/* Count is 3000 for 1 ms
*/
#define MILLISEC_TIMER_PERIOD 3000


/* MOD_BAUD_TIMER_Timer is baudrate
 * Define which timer to use
 */
#define MOD_BAUD_TIMER        TCC1
/*
*This example runs at 24MHz peripheral clock
use 2.67 us resolution
*/
#define MOD_BAUD_TIMER_RESOLUTION	375000

/* Count is 10000 for 50 baud
*/
#define MOD_BAUD_TIMER_PERIOD 1250




#endif /* TIMER_H_ */