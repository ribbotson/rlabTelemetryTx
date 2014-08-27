/*____________________________________________________________________________*/
/* File: cmnd.c
 *
 * Created: 22/05/2014 16:34:57
 * Revised  08/07/2014 16:00
 * Author: Richard Ibbotson
 * Compiler:    GNU-AVR-GCC
 * Project:  rlab Telemetry Transmitter
 */ 
/*____________________________________________________________________________*/
/*
 * 
 *  This source module implements the "host command interface" (HCI) and a
 *  set of "debug" commands.
 *
 *  A command string is composed of a 2-letter command "name" and a number of
 *  user-supplied "arguments" (parameters). Some commands have no arguments.
 *  A single space must be inserted between command buffer arguments, where
 *  there is more than one (including the 2-letter command name).
*/
/*____________________________________________________________________________*/
/*
 * Copyright, License and Attribution:
 *
 * This is a derived work and a modified version of the HCI interface created by 
 * M.J. Bauer ( 2007-2011) [Ref:  www.mjbauer.biz]
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

#include  "cmnd.h"
#include "udi_cdc.h"
#include "gendef.h"
#include "stdlib.h"
#include "stdio.h"
#include "asf.h"
#include "main.h" 
#include "sensor.h"
#include "timer.h"
#include "comms.h"
#include "nvconfig.h"

extern int build_major;
extern int build_minor;
extern int build_release;
extern char * build_date;
extern struct CONFIG config;
extern FILE usbout;
extern FILE ntx2bout;

// Globals...

uchar   currentTxChan = 0xff;
uchar   currentTxTrim = 0xff;
extern char transmit_state;
extern char power_status;
// Command table entry looks like this
struct  CmndTableEntry_t
{
	char     cName1;            // command name, 1st char
	char     cName2;            // command name, 2nd char
	pfnvoid  Function;          // pointer to HCI function
};


static  char    gacCmdMsg[CMD_MSG_SIZE+1];      // Command message buffer
static  char  * pcCmdPtr;               // Pointer into gacCmdMsg[]





/*****
|   Command table -- maximum number of commands is 250.
|   (Application-specific command functions should go at the top)
*/
const  struct  CmndTableEntry_t  asCommand[] =
{
	{ 'L','S',    list_cmd           },
	{ 'V','N',    version_cmd        },
	{ 'T','D',    show_telemetry_cmd },		
	{ 'A','D',    adc_cmd			 },
	{ 'D','A',    dac_cmd            },
	{ 'C','H',    set_tx_channel_cmd },
	{ 'S','T',    show_status		 },	
	{ 'C','O',    show_config		 },	
	{ 'E','R',    read_eeprom		 },			
	{ 'E','W',    write_eeprom		 },			
	{ 'W','C',    write_tx_channel_cmd},
	{ 'F','O',    set_tx_trim_cmd    },	
	{ 'C','W',    set_cw_cmd	     },	
	{ 'B','E',    beacon_cmd		 },			
	{ '$','$',    null_cmd           }      // Last entry in cmd table
} ;


/*
|   Initialise the Host Command Interface
|   Called from main before HCI is used.
*/
void  hci_init(void)
{
	hci_clear_command();
}





/*
|   Function examines a character received via the HCI input stream and decides what
|   to do with it. If printable, it goes into the command buffer, gacCmdMsg[].
|   When a command terminator (CR) is received, the command message is interpreted;
|   if the command name is identified, the respective command function is executed.
|   Command functions may generate response data to be transmitted back to the host PC.
*/
void  hci_process_input( char c )
{
	if ( c == '\r' )      // CR is the command message terminator
	{
		if ( pcCmdPtr != gacCmdMsg )    // if the command msg is not empty...
		{
			hci_exec_command();         // ... interpret the command.
		}
		else  hci_put_resp_term();
	}
	else if ( isprint(c) )      // if printable, append c to command buffer
	{
		if ( pcCmdPtr < (gacCmdMsg + CMD_MSG_SIZE) )  *pcCmdPtr++ = c;
		putchar( c );	    // echo char back to user terminal
	}
	else if ( c == ESC || c == CAN )    // Expected from "interactive" user only
	{
		hci_clear_command();    // Trash cmd message
		hci_put_resp_term();
	}
}


/*
|   Function looks for command name (mnemonic, 2 chars) in command table;
|   if found, executes respective command function.
*/
void  hci_exec_command( void )
{
	char   c1, c2;
	uint8  n;
	bool   yFoundCndName = FALSE;

	c1 = toupper( gacCmdMsg[0] );
	c2 = toupper( gacCmdMsg[1] );

	for ( n = 0;  n < 250;  n++ )
	{
		if ( asCommand[n].cName1 == '$' )       // end of table
		break;
		if ( asCommand[n].cName1 == c1 )
		{
			if ( asCommand[n].cName2 == c2 )    // found match
			{
				yFoundCndName = TRUE;
				break;
			}
		}
	}
	if ( yFoundCndName )
	{
		putchar('\n');
		(*asCommand[n].Function)();     // Do command function
	}
	else  hci_put_cmd_error();          // Unrecognised command

	hci_put_resp_term();        // Output the response terminator codes
	hci_clear_command();        // Prepare for new command
}


/*
|   Function:   hci_clear_command();
|   Clear command buffer buffer and reset pointer.
|
|   Affects:  gacCmdMsg[], pcCmdPtr, cRespCode
*/
void  hci_clear_command( void )
{
	uint8  ubx;

	for ( ubx = 0;  ubx < (CMD_MSG_SIZE+1);  ubx++ )
	{
		gacCmdMsg[ubx] = NUL;
	}
	pcCmdPtr = gacCmdMsg;
	
}

/*
|  Indicate to HCI user (human or machine) that the last command msg
|  received had an error.
*/
void  hci_put_cmd_error()
{
	
	printf("\n! Command Error");
}


/*  Dummy command function -- does nothing!  */
void  null_cmd( void )
{
	return;
}


/********************************  HOST COMMAND FUNCTIONS  ******************************/

const  char  acHelpStrLS[] PROGMEM = "LS        | List Command Set\n";
const  char  acHelpStrVN[] PROGMEM = "VN        | Show Version\n";
const  char  acHelpStrAD[] PROGMEM = "AD        | Display ADC values\n";
const  char  acHelpStrDA[] PROGMEM = "DA        | Set DAC output\n";
const  char  acHelpStrCH[] PROGMEM = "CH [aa]   | Read/Set TX Channel\n";
const  char  acHelpStrWC[] PROGMEM = "WC [aa]   | Store TX Channel\n";
const  char  acHelpStrTR[] PROGMEM = "FO [aa]   | Read/Set TX Trim\n";
const  char  acHelpStrCW[] PROGMEM = "CW [a]    | Read/Set CW Enable\n";
const  char  acHelpStrBE[] PROGMEM = "BE [a]    | Beacon Transmit Enable\n";
const  char  acHelpStrCO[] PROGMEM = "CO        | Show Configuration\n";
const  char  acHelpStrER[] PROGMEM = "ER        | Read from EEPROM\n";
const  char  acHelpStrEW[] PROGMEM = "EW        | Write to EEPROM\n";
const  char  acHelpStrST[] PROGMEM = "ST        | Show Status\n";
const  char  acHelpStrTD[] PROGMEM = "TD        | Show Telemetry Data\n";

/*
|  Command function 'LS' :  Lists a command set Summary.
|  Command message format:  "LS"
*/
void  list_cmd( void )
{
	
	printf_P( acHelpStrLS );
	printf_P( acHelpStrVN );
	printf_P( acHelpStrAD );
	printf_P( acHelpStrDA );
	printf_P( acHelpStrCH );
	printf_P( acHelpStrWC );
	printf_P( acHelpStrTR );
	printf_P( acHelpStrCW );
	printf_P( acHelpStrBE );
	printf_P( acHelpStrTD );
	printf_P( acHelpStrST );
	printf_P( acHelpStrCO );
	printf_P( acHelpStrER );
	printf_P( acHelpStrEW );
}


/*
|  Command function 'VN':  Print firmware version number & build date/time.
|
|  Response format:  "Vm.n.rddd : rlab Aug 02 2011" (example)
*/
void  version_cmd( void )
{
	printf("V%1d.%1d.r%03d rlab %s\n", build_major, build_minor, build_release, build_date );
}

/*
|  Command function 'TD':  Print telemetry data.
|
|  Response format:  
*/
void show_telemetry_cmd(void)
{
	int satellites;
	printf("\nTelemetry Data:\n");
	char string_buffer[12];
	dt_format(string_buffer, get_gps_date());
	printf("Date: %s\n", string_buffer);
	dt_format(string_buffer, get_gps_time());
	printf("Time: %s UTC\n", string_buffer);
	printf("Temperature = %d.%d Deg C\n", get_gps_temperature() / 100 , get_gps_temperature() % 100);
	printf("Pressure = %d.%d kPa\n", get_gps_pressure() / 10 , get_gps_pressure() % 10);
	printf("Number of Satellites = %d\n", satellites = get_gps_satellites());
	if (satellites > 4)
	{
		angle_format (string_buffer, get_gps_latitude());
		printf("Latitude: %s\n", string_buffer);
		angle_format (string_buffer, get_gps_longitude());
		printf("Longitude: %s\n", string_buffer);
		printf("Altitude: %ld\n", get_gps_altitude());
		angle_format (string_buffer, get_gps_direction());
		printf("Direction: %s\n", string_buffer);
		printf("Speed: %ld\n", get_gps_speed());
	}
}

void dt_format( char * dtstring, long dtvalue)
{
	char dtbuffer[9];
	
	sprintf(dtbuffer, "%06ld", dtvalue);
	
	dtstring[0] = dtbuffer[0];
	dtstring[1] = dtbuffer[1];
	dtstring[2] = ':';
	dtstring[3] = dtbuffer[2];
	dtstring[4] = dtbuffer[3];
	dtstring[5] = ':';
	dtstring[6] = dtbuffer[4];
	dtstring[7] = dtbuffer[5];
	dtstring[8] = '\0';
}

void angle_format( char * anglestring, signed long anglevalue)
{
	char anglebuffer[12];
	if(anglevalue < 0)
	{
		anglevalue = - anglevalue;
		anglestring[0] = '-';
	}
	else anglestring[0] = '+';
	
	sprintf(anglebuffer, "%08ld", anglevalue);
	
	anglestring[1] = anglebuffer[0];
	anglestring[2] = anglebuffer[1];
	anglestring[3] = ' ';
	anglestring[4] = anglebuffer[2];
	anglestring[5] = anglebuffer[3];
	anglestring[6] = '.';
	anglestring[7] = anglebuffer[4];
	anglestring[8] = anglebuffer[5];
	anglestring[9] = anglebuffer[6];
	anglestring[10] = anglebuffer[7];
	anglestring[11] = '\0';
}


/*
|  Command function 'AD':  Display values from ADC.
*/
void  adc_cmd( void )
{
	float BV = (float) adc_get_result(&MY_ADC, VBATT_ADC_CH);
	BV = BV - 200;
	if(BV < 0) BV = 0;
	BV = BV/717;
	float UV = (float) adc_get_result(&MY_ADC, VUSB_ADC_CH);
	UV = UV- 200;
	if(UV < 0) UV = 0;
	UV = UV/717;
	printf("VBATT = %1.2fV  VUSB = %1.2fV", BV, UV);	
}




/*
|  Command function 'DA':  Set the DAC output.
arg 1 is decimal value 0 to 4095
*/
void  dac_cmd( void )
{
	uint16  uwArgValue;
	if ( isDecDigit( gacCmdMsg[3] ) ) // DAC value present ?
	{  
		uwArgValue = decatoi( &gacCmdMsg[3], 5 );      // defaults to 0 if invalid arg. 
		if (uwArgValue > 4095) uwArgValue = 4095;
		dac_set_channel_value(&MY_DAC, NTX2B_DAC, uwArgValue);
		printf("DAC value = %d\n", uwArgValue);
		
	}			   
	else printf("No Value given !\n");
	
}

void set_tx_channel_cmd()
{
	unsigned long txfreq;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxChan = hexatoi( &gacCmdMsg[3] );
		if (currentTxChan > 0xEA) currentTxChan = 0xEA;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		delay_ms(100);
		fprintf(&ntx2bout, "%cch%02X\r", 0x80, currentTxChan);
		delay_ms(100);
		ioport_set_pin_mode(GPIO_NTX2B_EN, (transmit_state != TRANSMIT_OFF)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxChan == 0xff) printf("Tx Channel unknown \n");
	else
	{
		txfreq = 50000ul + (unsigned long) currentTxChan * 3125ul;
		printf("TX Channel = %x  Frequency = 434.%06ld\n", currentTxChan, txfreq );				
	}
	
	
	
	
	
}
		 
void write_tx_channel_cmd(void)
{
	unsigned long txfreq;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxChan = hexatoi( &gacCmdMsg[3] );
		if (currentTxChan > 0xEA) currentTxChan = 0;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		delay_ms(100);
		fprintf(&ntx2bout, "%cwr%02X\r", 0x80, currentTxChan);
		delay_ms(100);
		ioport_set_pin_mode(GPIO_NTX2B_EN, (transmit_state != TRANSMIT_OFF)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxChan == 0xff) printf("Tx Channel unknown \n");
	else
	{
		txfreq = 50000ul + (unsigned long) currentTxChan * 3125ul;
		printf("TX Channel = %x  Frequency = 434.%06ld\n", currentTxChan, txfreq );
	}
			
}

void set_tx_trim_cmd(void)
{
	signed int trimfreq;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxTrim = hexatoi( &gacCmdMsg[3] );
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		delay_ms(100);
		fprintf(&ntx2bout, "%ctr%02X\r", 0x80, currentTxTrim);
		delay_ms(100);
		ioport_set_pin_mode(GPIO_NTX2B_EN, (transmit_state != TRANSMIT_OFF)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxTrim == 0xff) printf("Tx Trim unknown \n");
	else
	{
		if(currentTxTrim < 0x7f) trimfreq =  -(0x7f - currentTxTrim) * 125 / 10;
		else trimfreq = ((currentTxTrim - 0x7f) * 125) / 10;	
		printf("TX Trim = %x  Trim Frequency = %d Hz", currentTxTrim, trimfreq);
		
	}	
}

void   show_config(void)
{
		
		printf("Serial at Startup = ");
		if (config.uart_on_startup == START_UART_USB) printf("USB\n");
		else if(config.uart_on_startup == START_UART_EXT) printf("EXT UART\n"); 
		else if(config.uart_on_startup == START_UART_AUTO) printf("Auto Detect\n");	
		else printf("??\n");	
		
		printf("Beacon Repeat Time = %d\n", config.beacon_repeat_time);
		
		printf("Payload ID = %s\n", config.payload_id);
		
		printf("Modulation Mode = ");
		if(config.modulation_mode == RTTY300) printf("RTTY 300 baud\n");
		else printf("??\n");
		
		printf("Transmit Channel = %x\n", config.tx_channel);
		
		printf("Transmit Trim = %X\n", config.tx_trim);
		
		
		unsigned char	beacon_repeat_time; // units of 500ms
		char			payload_id[10];
		char			modulation_mode;
		char			tx_channel;
		char			tx_trim;	
	
}

void   read_eeprom(void)
{
	
	
}

void   write_eeprom(void)
{
	
	
}


void   show_status()
{
	printf("System Status:\n");
	
	printf(" Power Status =");
	if(power_status == POWER_USB) printf("USB\n");
	else if (power_status == POWER_BATTERY) printf("Battery\n");
	else if (power_status == POWER_BOTH) printf("USB and Battery\n");
	else printf("Unknown\n");
	
	printf(" Transmit Status =");
	if(transmit_state == TRANSMIT_CW ) printf("On CW\n");
	else if (transmit_state == TRANSMIT_BEACON) printf("On Beacon\n");
	else printf("Off\n");
}


void set_cw_cmd(void)
{
	if(transmit_state == TRANSMIT_BEACON)
	{
		printf("Error: Beacon is enabled\r");
	}
	else
	{
		
	if (gacCmdMsg[3] == '0')
	{ 
		transmit_state = TRANSMIT_OFF;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}		
	else if (gacCmdMsg[3] == '1')
	{
		 transmit_state = TRANSMIT_CW;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM );
	}		 

	
	printf("Transmitter is CW ");
	if(transmit_state == TRANSMIT_CW) printf("enabled\n");
	else printf("disabled\n");
	}
}

void beacon_cmd(void)
{
	
	if (gacCmdMsg[3] == '0')
	{
		transmit_state = TRANSMIT_OFF;
		ntx2b_mod_disable();
		
	}
	else if (gacCmdMsg[3] == '1')
	{
		transmit_state = TRANSMIT_BEACON;
		ntx2b_mod_enable(RTTY300);
		
	}

	
	printf("Beacon is ");
	if(transmit_state == TRANSMIT_BEACON) printf("enabled\n");
	else printf("disabled\n");
	
}






/*
|  Send response termination sequence to the HCI serial output stream.
|  In "interactive user mode", this is a prompt for new command.
*/
void  hci_put_resp_term( void )
{
	printf("\n>" );		
}



/*****************************************************************************************
|                           CHARACTER CONVERSION FUNCTIONS
|
|  Convert decimal ASCII char to 4-bit BCD value (returned as unsigned byte).
|
|  Entry args: c = decimal digit ASCII encoded
|  Returns:    0xFF if arg is not a decimal digit, else unsigned byte (0..9)
|  Affects:    ---
*/
unsigned char  dectobin( char c )
{
	if ( c >= '0'  &&  c <= '9')
	return ( c - '0' );
	else
	return 0xFF ;
}



unsigned int   decatoi( char * pnac, char bNdigs )
{
	unsigned char   ubDigit, ubCount;
	unsigned int	uwResult = 0;

	for ( ubCount = 0;  ubCount < bNdigs;  ubCount++ )
	{
		if ( (ubDigit = dectobin( *pnac++ )) == 0xFF )
		break;
		uwResult = 10 * uwResult + ubDigit;
	}
	return  uwResult;
}


/*
|  Convert Hexadecimal ASCII char (arg) to 4-bit value (returned as unsigned byte).
|
|  Called by:  various, background only
|  Entry args: c = Hex ASCII character
|  Returns:    0xFF if arg is not hex, else digit value as unsigned byte ( 0..0x0F )
*/
unsigned char  hexctobin( char c )
{
	if ( c >= '0'  &&  c <= '9')
	return ( c - '0' );
	else if ( c >= 'A'  &&  c <= 'F' )
	return ( c - 'A' + 10 );
	else if ( c >= 'a'  &&  c <= 'f' )
	return ( c - 'a' + 10 );
	else
	return 0xFF ;
}


/*
|  Convert Hexadecimal ASCII string, up to 4 digits, to 16-bit unsigned word.
|  The string must be stored in the data RAM space.
|  There cannot be any leading white space.
|  Conversion is terminated when a non-Hex char is found.
|
|  Entry args: (char *) s = pointer to first char of hex string.
|  Returns:    Unsigned 16bit word ( 0 to 0xffff ).
|              If the target string (1st char) is non-Hex, returns 0.
*/
unsigned int   hexatoi( char * s )
{
	unsigned char   ubDigit, ubCount;
	unsigned int  uwResult = 0;

	for ( ubCount = 0;  ubCount < 4;  ubCount++ )
	{
		if ( (ubDigit = hexctobin( *s++ )) == 0xFF )
		break;
		uwResult = 16 * uwResult + ubDigit;
	}
	return  uwResult;
}


/*
|  Function returns TRUE if char is hex ASCII digit ('0'..'F')
*/
char  isHexDigit( char c )
{
	if ( hexctobin( c ) == 0xFF ) return FALSE;
	else  return TRUE;
}

/*
|  Function returns TRUE if char is dec ASCII digit ('0'..'F')
*/
char  isDecDigit( char c )
{
	if ( dectobin( c ) == 0xFF ) return FALSE;
	else  return TRUE;
}


// end
