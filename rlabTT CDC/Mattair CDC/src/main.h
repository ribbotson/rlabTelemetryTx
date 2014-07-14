/*____________________________________________________________________________*/
/*
 * File: main.h
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

#ifndef _MAIN_H_
#define _MAIN_H_

#include "usb_protocol_cdc.h"

/*! \brief Opens the communication port
 * This is called by CDC interface when USB Host enable it.
 *
 * \retval true if cdc startup is successfully done
 */
bool main_cdc_enable(uint8_t port);

/*! \brief Closes the communication port
 * This is called by CDC interface when USB Host disable it.
 */
void main_cdc_disable(uint8_t port);

/*! \brief Manages the leds behaviors
 * Called when a start of frame is received on USB line each 1ms.
 */
void main_sof_action(void);

/*! \brief Enters the application in low power mode
 * Callback called when USB host sets USB line in suspend state
 */
void main_suspend_action(void);

/*! \brief Turn on a led to notify active mode
 * Called when the USB line is resumed from the suspend state
 */
void main_resume_action(void);

/*! \brief Save new DTR state to change led behavior.
 * The DTR notify that the terminal have open or close the communication port.
 */
void main_cdc_set_dtr(uint8_t port, bool b_enable);

void main_cdc_rx_notify(uint8_t port);

void adc_init(void);

void dac_init(void);

void gps_uart_init(void);

void ext_uart_init(void);



unsigned long main_get_msclock_ticks(void);

void millisecTimer(void);

/* MILLISEC_Timer is millisecond timer
 * Define which timer to use
 */
#define MILLISEC_TIMER        TCC0
/*
*This example runs at 32MHz internal clock
use 1 us resolution
*/
#define MILLISEC_TIMER_RESOLUTION	1000000

/* Count is 1000 for 1 ms
*/
#define MILLISEC_TIMER_PERIOD 1000

#define GPS_USART_SERIAL                 &USARTC0
#define GPS_USART_SERIAL_BAUDRATE        9600
#define GPS_USART_SERIAL_CHAR_LENGTH     USART_CHSIZE_8BIT_gc
#define GPS_USART_SERIAL_PARITY          USART_PMODE_DISABLED_gc
#define GPS_USART_SERIAL_STOP_BIT        false

#define NTX2B_USART_SERIAL               &USARTD0
#define NTX2B_USART_SERIAL_BAUDRATE      4800
#define NTX2B_USART_SERIAL_CHAR_LENGTH   USART_CHSIZE_8BIT_gc
#define NTX2B_USART_SERIAL_PARITY        USART_PMODE_DISABLED_gc
#define NTX2B_USART_SERIAL_STOP_BIT      false

#define EXT_USART_SERIAL                 &USARTE0
#define EXT_USART_SERIAL_BAUDRATE        9600
#define EXT_USART_SERIAL_CHAR_LENGTH     USART_CHSIZE_8BIT_gc
#define EXT_USART_SERIAL_PARITY          USART_PMODE_DISABLED_gc
#define EXT_USART_SERIAL_STOP_BIT        false



#endif // _MAIN_H_
