/**
 * \file
 *
 * \brief STK600 with the RC044X routing card board header file.
 *
 * This file contains definitions and services related to the features of the
 * STK600 board with the routing card for 44-pin TQFP AVR XMEGA devices.
 *
 * To use this board, define BOARD=STK600_RC044X.
 *
 * Copyright (c) 2010-2011 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef _MT_DB_X4_H_
#define _MT_DB_X4_H_

#include "compiler.h"


//! Number of LEDs.
#define LED_COUNT   1

/*! \name GPIO Connections of LEDs

 
 */
//! @{
#define LED0_GPIO   IOPORT_CREATE_PIN(PORTB,1)

//! @}


/*! \name GPIO Connections of PCB Links
 * 
 
 */
//! @{
#define GPIO_PCB_LINK    IOPORT_CREATE_PIN(PORTD,2) // Boot link

//! @}

/*! \name GPIO Connections of Cut Down Output
 * 
 
 */
//! @{
#define GPIO_CUTDOWN    IOPORT_CREATE_PIN(PORTD,4) // Cutdown FET

//! @}

/*! \name GPIO Connections of NTX2B Transmitter Enable
 * 
 
 */
//! @{
#define GPIO_NTX2B_EN    IOPORT_CREATE_PIN(PORTD,3) // Transmit enable

//! @}

/* ADC Ports 
VBATT on CH0 connected to port A0 voltage divided by 10
VUSB on CH1 connected to port A1 voltage divided by 10 */
#define MY_ADC			ADCA
#define VBATT_ADC_CH	ADC_CH0
#define VUSB_ADC_CH		ADC_CH1

/* DAC Ports
NTX2B_DAC is CH0 connected to port B2 */
#define MY_DAC			DACB
#define NTX2B_DAC		DAC_CH0


/*! \name External oscillator
 */
//@{
#define BOARD_XOSC_HZ          16000000
#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
#define BOARD_XOSC_STARTUP_US  500000
//@}


#endif  // _MT_DB_X4_H_
