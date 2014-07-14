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
#include <stdlib.h>
#include "asf.h"
#include "main.h" 
#include "ntx2b.h"
#include "gps.h"
#include "i2c400.h"

// Globals...
uint16  gwDebugFlags;
uint16  gwSystemError;
uchar   currentTxChan = 0xff;
uchar   currentTxTrim = 0xff;
bool	ntx2b_enabled = 0;



// Command table entry looks like this
struct  CmndTableEntry_t
{
	char     cName1;            // command name, 1st char
	char     cName2;            // command name, 2nd char
	pfnvoid  Function;          // pointer to HCI function
};


static  char    gacCmdMsg[CMD_MSG_SIZE+1];      // Command message buffer
static  char  * pcCmdPtr;               // Pointer into gacCmdMsg[]
static  char    cRespCode;              // Response termination code
static  bool    yInteractive= TRUE;



/*****
|   Command table -- maximum number of commands is 250.
|   (Application-specific command functions should go at the top)
*/
const  struct  CmndTableEntry_t  asCommand[] =
{
	{ 'L','S',    list_cmd           },
	{ 'V','N',    version_cmd        },
	{ 'E','N',    show_env_cmd       },		
	{ 'A','D',    adc_cmd			 },
	{ 'D','A',    dac_cmd            },
	{ 'C','H',    set_tx_channel_cmd },
	{ 'G','P',    show_gps_pos_cmd   },	
	{ 'D','T',    show_gps_date_cmd  },	
	{ 'W','C',    write_tx_channel_cmd},
	{ 'T','R',    set_tx_trim_cmd    },	
	{ 'T','X',    set_tx_enable      },			
	{ '$','$',    null_cmd           }      // Last entry in cmd table
} ;


/*
|   Initialise the Host Command Interface
|   Called from main before HCI is used.
*/
void  hci_init(void)
{
	#if INTERACTIVE_ON_STARTUP      // Set default HCI comm's mode
	yInteractive = TRUE;
	#else
	yInteractive = FALSE;
	#endif
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
		if ( yInteractive ) cdcPutch( c );	    // echo char back to user terminal
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
		if ( yInteractive )  NEW_LINE;
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
	if ( yInteractive ) cRespCode = '=';
	else  cRespCode = '-';
}


/*
|  Indicate to HCI user (human or machine) that the last command msg
|  received had an error.
*/
void  hci_put_cmd_error()
{
	cRespCode = '!';
	if ( yInteractive ) cdcPutstr("\n! Command Error");
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
const  char  acHelpStrTR[] PROGMEM = "TR [aa]   | Read/Set TX Trim\n";
const  char  acHelpStrTX[] PROGMEM = "TX [a]    | Read/Set TX Enable\n";
const  char  acHelpStrGP[] PROGMEM = "GP        | Show GPS Position\n";
const  char  acHelpStrDT[] PROGMEM = "DT        | Show GPS Date and Time\n";
const  char  acHelpStrEN[] PROGMEM = "EN        | Show Environment Data\n";

/*
|  Command function 'LS' :  Lists a command set Summary.
|  Command message format:  "LS"
*/
void  list_cmd( void )
{
	
	cdcPutstr_P( acHelpStrLS );
	cdcPutstr_P( acHelpStrVN );
	cdcPutstr_P( acHelpStrAD );
	cdcPutstr_P( acHelpStrDA );
	cdcPutstr_P( acHelpStrCH );
	cdcPutstr_P( acHelpStrWC );
	cdcPutstr_P( acHelpStrTR );
	cdcPutstr_P( acHelpStrTX );
	cdcPutstr_P( acHelpStrEN );
	cdcPutstr_P( acHelpStrDT );
	cdcPutstr_P( acHelpStrGP );
}



/*
|  Command function 'AD':  Display values from ADC.
*/
void  adc_cmd( void )
{
	cdcPutstr("VBATT = ");
	cdcPutDecWord(adc_get_result(&MY_ADC, VBATT_ADC_CH), 5);
	cdcPutstr("  VUSB = ");
	cdcPutDecWord(adc_get_result(&MY_ADC, VUSB_ADC_CH), 5);
	cdcPutch( '\r' );      // return cursor to start of line
	
}

void show_env_cmd(void)
{
	long wDelayStartTime;
	signed long Pressure;
	mpl115_convert();
	wDelayStartTime = main_get_msclock_ticks();     // Start 4ms delay
	while ( main_get_msclock_ticks() - wDelayStartTime < 10 )
	{
		
	}
	cdcPutstr("SE95 Temperature = ");
	cdcPutDecWord((se95_read() >> 8), 3);
	cdcPutstr(" Deg C");
	cdcPutch( '\r' );      // return cursor to start of line
	cdcPutch( '\n' );      // return cursor to start of line
	cdcPutstr("MPL115 Pressure = ");
	Pressure = (((signed long) mpl115_read_pressure() * 1041) >> 14) + 800;
	cdcPutDecWord((unsigned int) Pressure >> 4, 3);
	cdcPutstr(" kPa");
	cdcPutch( '\r' );      // return cursor to start of line
	
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
		cdcPutDecWord(uwArgValue, 5);
		
		cdcPutch( '\r' );      // return cursor to start of line
	}
			   
	else
	{
		cdcPutstr("No Value given !\r");
	}
}

void set_tx_channel_cmd()
{
	unsigned long txfreq;
	unsigned long   wDelayStartTime;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxChan = hexatoi( &gacCmdMsg[3] );
		if (currentTxChan > 0xEA) currentTxChan = 0xEA;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );		
		ntx2bPutch(0x80);
		ntx2bPutstr("ch");
		ntx2bPutHexByte(currentTxChan);
		ntx2bPutch('\r');
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );
		ioport_set_pin_mode(GPIO_NTX2B_EN, (ntx2b_enabled)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxChan == 0xff) cdcPutstr("Tx Channel unknown \r");
	else
	{
		cdcPutstr("Tx Channel = ");
		cdcPutHexByte(currentTxChan);
		cdcPutstr(" Frequency = 434.");
		txfreq = 50000ul + (unsigned long) currentTxChan * 3125ul;
		cdcPutDecLong(txfreq, 6);
		cdcPutch( '\r' );      // return cursor to start of line
		
	}
	
	
	
	
	
}
		 
void write_tx_channel_cmd(void)
{
	unsigned long txfreq;
	unsigned long   wDelayStartTime;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxChan = hexatoi( &gacCmdMsg[3] );
		if (currentTxChan > 0xEA) currentTxChan = 0;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );
		ntx2bPutch(0x80);
		ntx2bPutstr("wr");
		ntx2bPutHexByte(currentTxChan);
		ntx2bPutch('\r');
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );
		ioport_set_pin_mode(GPIO_NTX2B_EN, (ntx2b_enabled)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxChan == 0xff) cdcPutstr("Tx Channel unknown \r");
	else
	cdcPutstr("Tx Channel = ");
	cdcPutHexByte(currentTxChan);
	cdcPutstr(" Frequency = 434.");
	txfreq = 50000ul + (unsigned long) currentTxChan * 3125ul;
	cdcPutDecLong(txfreq, 6);
	cdcPutch( '\r' );      // return cursor to start of line
}

void set_tx_trim_cmd(void)
{
	unsigned long   wDelayStartTime;
	if (isHexDigit( gacCmdMsg[3] ) ) // new value present ?
	{
		currentTxTrim = hexatoi( &gacCmdMsg[3] );
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM);
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );
		ntx2bPutch(0x80);
		ntx2bPutstr("tr");
		ntx2bPutHexByte(currentTxTrim);
		ntx2bPutch('\r');
		wDelayStartTime = main_get_msclock_ticks();     // Start 100ms delay
		while ( main_get_msclock_ticks() - wDelayStartTime < 100 );
		ioport_set_pin_mode(GPIO_NTX2B_EN, (ntx2b_enabled)? IOPORT_MODE_TOTEM : IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}
	if(currentTxTrim == 0xff) cdcPutstr("Tx Trim unknown \r");
	else
	{
		cdcPutstr("Tx Trim = ");
		cdcPutHexByte(currentTxTrim);
		cdcPutstr(" Trim Frequency = ");
		if(currentTxTrim < 0x7f)
		{
			cdcPutch( '-' );
			cdcPutDecWord(( (0x7f - currentTxTrim)* 125) / 10, 4);
			
		}
		else
		{
			
			cdcPutDecWord(((currentTxTrim - 0x7f) * 125) / 10, 4);
			
		}
	}	
	cdcPutch( '\r' );      // return cursor to start of line
}

void   show_gps_date_cmd(void)
{
	long lvalue;
	cdcPutstr("Date = ");
	if((lvalue = get_gps_date()) == GPS_INVALID_DATE) cdcPutstr(" Invalid ");
	else cdcPutDecLong(lvalue, 10);	
	cdcPutstr("  Time = ");
	if((lvalue = get_gps_time()) == GPS_INVALID_TIME) cdcPutstr(" Invalid ");
	else cdcPutDecLong(lvalue, 10);
	cdcPutch( '\r' );      // return cursor to start of line
	
}


void   show_gps_pos_cmd()
{
	long lvalue;
	cdcPutstr("Latitude = ");
	if((lvalue = get_gps_latitude()) == GPS_INVALID_ANGLE) cdcPutstr(" Invalid ");
	else cdcPutDecLong(lvalue, 10);
	cdcPutstr("  Longitude = ");
	if((lvalue = get_gps_longitude()) == GPS_INVALID_ANGLE) cdcPutstr(" Invalid ");
	else cdcPutDecLong(lvalue, 10);
	cdcPutstr("  Altitude = ");
	if((lvalue = get_gps_altitude()) == GPS_INVALID_ALTITUDE) cdcPutstr(" Invalid ");
	else cdcPutDecLong(lvalue, 10);

	cdcPutch( '\r' );      // return cursor to start of line
}


void set_tx_enable(void)
{
	
	if (gacCmdMsg[3] == '0')
	{ 
		ntx2b_enabled = false;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM | IOPORT_MODE_INVERT_PIN );
	}		
	else if (gacCmdMsg[3] == '1')
	{
		 ntx2b_enabled = true;
		ioport_set_pin_mode(GPIO_NTX2B_EN, IOPORT_MODE_TOTEM );
	}		 

	
	cdcPutstr("Transmitter is ");
	if(ntx2b_enabled) cdcPutstr("enabled\r");
	else cdcPutstr("disabled\r");
	
}


/*
|  Command function 'VN':  Print firmware version number & build date/time.
|
|  Response format:  "Vm.n.ddd : MJB Aug 02 2011" (example)
*/
void  version_cmd( void )
{
	cdcPutch( 'V' );
	cdcPutDecWord( BUILD_VER_MAJOR, 1 );
	cdcPutch( '.' );
	cdcPutDecWord( BUILD_VER_MINOR, 1 );
	cdcPutch( '.' );
	cdcPutch( 'r' );
	cdcPutDecWord( BUILD_VER_DEBUG, 3 );
	if ( yInteractive )
	{
		cdcPutstr( " rlab " );
		cdcPutstr( (char *) __DATE__ );
		if ( yInteractive ) NEW_LINE;
	}
}






/*
|  Send response termination sequence to the HCI serial output stream.
|  In "interactive user mode", this is a prompt for new command.
*/
void  hci_put_resp_term( void )
{
	cdcPutch( '\r' );
	cdcPutch( '\n' );
    cdcPutch( '>' );
}



/******************************  HCI "I/O LIBRARY" FUNCTIONS  ***************************/
void cdcPutch(char value)
{
	
	udi_cdc_putc(value);
}	
/*
|  Output a NUL-terminated string to the HCI serial port.
|  The string is expected to be in the data memory (SRAM) space.
|  Newline (0x0A) is expanded to CR + LF (0x0D + 0x0A).
|
|  After outputting the string, the background task dispatcher is called,
|  so as to minimize the delay of any pending or currently executing task.
|  To avoid the possibility of infinite recursion, this function cannot be
|  called from any scheduled background task.
|
|  Called by:   HCI command functions only (NOT scheduled B/G tasks!)
|  Entry args:  pstr = address of NUL-terminated string in SRAM.
*/
void  cdcPutstr( char * pstr )
{
	char   c;

	while ( (c = *pstr++) != 0 )
	{
		if ( c == '\n' )
		{
			cdcPutch( '\r' );
			cdcPutch( '\n' );
		}
		else   cdcPutch( c );
	}
	
}

/*
|  Output a NUL-terminated string to the HCI serial port.
|  The string is expected to be in the program code (flash) memory space.
|  Newline (0x0A) is expanded to CR + LF (0x0D + 0x0A).
|
|  After outputting the string, the background task dispatcher is called,
|  so as to minimize the delay of any pending or currently executing task.
|  To avoid the possibility of infinite recursion, this function cannot be
|  called from any scheduled background task.
|
|  Called by:   HCI command functions only (NOT scheduled B/G tasks!)
|  Entry args:  pksz = address of NUL-terminated string in PROGMEM.
*/
void  cdcPutstr_P( PGM_P pksz )
{
	char   c;

	while ( (c = pgm_read_byte(pksz)) != 0 )
	{
		if ( c == '\n' )
		{
			cdcPutch( '\r' );
			cdcPutch( '\n' );
		}
		else   cdcPutch( c );
		pksz++ ;
	}
	
}

/*
|  Output Boolean value as ASCII '0' or '1'.
|
|  Called by:  command functions, etc.
|  Entry args: b = Boolean variable (zero or non-zero)
|  Returns:    void
|  Affects:
*/
void  cdcPutBoolean( bool  b )
{
	if ( b )  cdcPutch( '1');
	else  cdcPutch ( '0' );
}

/*
|  Output 4 LS bits of a byte as Hex (or BCD) ASCII char.
|
|  Called by:  command functions, etc.
|  Entry args: d = value of Hex digit (0 to 0xf)
|  Returns:    void
|  Affects:    --
*/
void  cdcPutHexDigit( unsigned char d )
{
	d &= 0x0F;
	if ( d < 10 )  cdcPutch ( '0' + d );
	else  cdcPutch ( 'A' + d - 10 );
}

/*
|  Output byte as 2 Hex ASCII chars, MSD first.
|
|  Called by:  command functions, etc.
|  Entry args: b = byte to output
|  Returns:    void
|  Affects:    --
*/
void  cdcPutHexByte( unsigned char b )
{
	cdcPutHexDigit( b >> 4 );
	cdcPutHexDigit( b );
}

/*
|  Output 16-bit word as 4 Hex ASCII chars, MSD first.
|
|  Called by:  command functions, etc.
|  Entry args: uwArg1 = word to output
|  Returns:    void
|  Affects:    --
*/
void  cdcPutHexWord( unsigned int uwArg1 )
{
	cdcPutHexDigit( (unsigned char) (uwArg1 >> 12) );
	cdcPutHexDigit( (unsigned char) (uwArg1 >> 8) );
	cdcPutHexDigit( (unsigned char) (uwArg1 >> 4) );
	cdcPutHexDigit( (unsigned char) (uwArg1 & 0xF) );
}

/*
|  Output a 16-bit unsigned word as an ASCII decimal number, with leading zeros.
|  The number of digit places to be output (0 to 5) is specified as a parameter.
|  If the decimal word value is larger than can fit into the number of places
|  specified, then the output will be truncated to the least significant digit(s).
|  If the decimal word value is smaller than can occupy the number of places
|  specified, then the output will be padded with leading 0's.
|
|
|  Called by:  command functions, etc
|  Entry args: (uint16) uwArg1 = word to output
|              (uint8) ubPlaces = number of digit places to output (1..5)
|  Returns:    void
|  Affects:    --
*/
void  cdcPutDecWord( unsigned int uwArg1, unsigned char ubPlaces )
{
	char   bPos;
	unsigned char  aubDigit[5];    // BCD result, 1 byte for each digit

	if ( ubPlaces > 5 )  ubPlaces = 5;
	for ( bPos = 4;  bPos >= 0;  --bPos )
	{
		aubDigit[bPos] = uwArg1 % 10;
		uwArg1 /= 10;
	}
	for ( bPos = 5 - ubPlaces;  bPos < 5;  ++bPos )
	cdcPutHexDigit( aubDigit[bPos] );
}

void   cdcPutDecLong(unsigned long ularg, unsigned char ubPlaces)
{
	char   bPos;
	unsigned char  aubDigit[10];    // BCD result, 1 byte for each digit

	if ( ubPlaces > 10 )  ubPlaces = 10;
	for ( bPos = 9;  bPos >= 0;  --bPos )
	{
		aubDigit[bPos] = ularg % 10;
		ularg /= 10;
	}
	for ( bPos = 10 - ubPlaces;  bPos < 10;  ++bPos )
	cdcPutHexDigit( aubDigit[bPos] );
}


/*
|  Output 16-bit word as 16 binary digits, MS bit first.
|
|  Called by:  command functions, etc.
|  Entry args: uwArg1 = word to output
*/
void  cdcPut_word_bits( unsigned int wArg )
{
	unsigned int   wBit;

	for ( wBit = 0x8000;  wBit != 0;  wBit >>= 1 )
	{
		cdcPutch( (wArg & wBit) ? '1' : '0' );
		cdcPutch( SPACE );
		if ( wBit == 0x0100 ) { cdcPutch( SPACE ); cdcPutch( SPACE ); }
	}
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


/*
|  Convert decimal ASCII string, up to 5 digits, to 16-bit unsigned word.
|  There may be leading zeros, but there cannot be any leading white space.
|  Conversion is terminated when a non-decimal char is found, or when the
|  specified number of characters has been processed.
|
|  Entry args: (char *) pac = pointer to first char of decimal ASCII string
|              (int8)  bNdigs = number of characters to process (max. 5)
|  Returns:    Unsigned 16bit word ( 0 to 0xffff ).
|              If the target string (1st char) is non-numeric, returns 0.
*/
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
