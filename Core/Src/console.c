/**
  ******************************************************************************
  * @file    console.c
  * @brief   This file contains console implimintation. 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
	

#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "commands.h"
#include "console.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"


	
//extern char	toupper(char);
//extern char	tolower(char);
extern int _EXFUN(toupper, (int __c));
extern int _EXFUN(isspace, (int __c));
/***************** Variable Initialization Block Begin ********/
console_t console;
/***************** Variable Initialization Block End **********/	
/***************** Fucntion Declaration Block End  *************/
const console_command_t g_command_list[] = {
    /*{
        (const char *) "\x1B",
        command_open,
        "OPEN CONSOLE", // USER/ONLINE/COMMAND MODE 
        CT_OFFLINE
    },*/
    {
        (const char *) "CONNECT",
        command_open,
        "OPEN CONSOLE", // USER/ONLINE/COMMAND MODE 
        CT_OFFLINE
    },
    {
        (const char *) "START",
        command_open,
        "OPEN CONSOLE", // USER/ONLINE/COMMAND MODE 
        CT_OFFLINE
    },
	{
		(const char *) "WIND",
		command_wind,
		"WIND", // GET WIND DATA
		CT_ONLINE
	},
	{
		(const char *) "AIR",
		command_air,
		"AIR", // GET AIR DATA
		CT_ONLINE
	},
	{
		(const char *) "TEST",
		command_test,
		"TEST", // GET TEST DATA
		CT_ONLINE
	},
	{
		(const char *) "CALIBRATE",
		command_calibrate,
		"CALIBRATE", // CALIBRATE SENSORS
		CT_ONLINE
	},
	{
		(const char *) "@ANALOG",
		command_reply_data,
		"RETURN DATA", // message conatains all analog data
		CT_ONLINE | CT_OFFLINE
	},
	{
		(const char *) "REBOOT",
		command_bootloader,
		"REBOOT BOOTLOADER", // REBOOT TO BOOTLOADER COMMAND
		CT_ONLINE | CT_UNICAST
	},
	{
		(const char *) "ADDRESS",
		command_address,
		"CHANGE ADDRESS", // change address of the module
		CT_ONLINE | CT_UNICAST
	},
	{
        (const char *) "CLOSE",
        command_close,
        "CLOSE CONSOLE", // AUTOMATIC COMMUNICATION/OFFLINE MODE
        CT_ONLINE
    },
    {
        NULL,
        NULL,
        NULL,
		NULL,
    }
		
};	

/***************** Fucntion Declaration Block Begin ***********/
void console_init( void )
{
   /*
   serial_interchar_timeout( CLI_INTERCHAR_TIMEOUT );
   serial_setup_istream( g_command_input, sizeof( g_command_input ), 0 );
   serial_setup_ostream( g_buffer, sizeof( g_buffer ), 0 );
   */
   
	console.output.buffer = console.output_buffer;
	console.output.size = sizeof( console.output_buffer );
	console.output.count = 0;
	console.output.offset = 0;
	console.output.tail = 0;
	console.output.head = 0;

	console.input.buffer = console.input_buffer;
	console.input.size = sizeof( console.input_buffer );
	console.input.count = 0;
	console.input.offset = 0;
	console.input.tail = 0;
	console.input.head = 0;

	console.state = CLI_CLOSED;
	console.session_timeout = CONSOLE_OFFLINE_SESSION_TIMEOUT;

	console.inactivity_timer = CONSOLE_INACTIVITY_TIMEOUT;
	console.inactivity_timeout = CONSOLE_INACTIVITY_TIMEOUT;


//	console.input.complete = xEventGroupCreate();
//	console.input.event_bits = xEventGroupSetBits( console.input.complete, CIS_IDLE );
	console.input.buffer[ 0 ] = '\0';
	console.input.count = 0;
		
		
}

/**
\brief convert command line into argv
\param[in] line command line
\param[in] argv table
\return argc
*/
static uint8_t line_to_argv( char *line, char **argv, uint8_t argcmax ) // line2argv
{
    char shift;
    char quotation = false;
    char *dest;
    uint8_t argc = 0;
    
    while ( *line && (argc < argcmax - 1) ) { // argv[ argc ] is NULL    
        while ( isspace( *line ) ) {
            *line = '\0';
            ++line;
        }
        
        if ( *line ) {
            *argv = line;
            ++argv;
            ++argc; // if (argc >= argcmax - 1) break;
        }
        else {
            // *argv = (char *) NULL;
            break;
        }
        
        shift = false;
        while ( *line && ( quotation || !isspace( *line ) ) ) {
            if ( '"' == *line ) {
                quotation = !quotation;
                if ( !shift ) {
                    dest = line;
                    shift = true;
                }
                *dest = '\0';
                ++line;
                
                continue;
            }
            
            if ( shift ) {
                *dest = *line;
                ++dest;
            }
            
            ++line;
        }
        
        if ( shift ) {
            *dest = '\0';
        } 
    }
    
    *argv = (char *) NULL;
    
    return argc;
}

int32_t str2int( char *str )
{
    if ( ( !str ) || ( !*str ) ) {
        return 0;
    }

    char chr = *str;

    // Check for a sign
    // Increase index if either positive or negative sign is detected
    uint8_t sign = 0;
    if ( chr == '-' ) {
        // Set the sign to negative
        sign = 1;
        chr = *(++str);
    }
    else if ( chr == '+' ) {
        chr = *(++str);
    }

    uint8_t base = 10;
    int32_t result = 0;

    // Skip leading zeros
    while ( chr == '0' ) {
        chr = *(++str);
    }

    if ( (chr == 'x') || (chr == 'X') ) { // if ( (*(str - 1) == '0') && ((chr == 'x') || (chr == 'X')) ) {
        base = 16;
        chr = *(++str);
    }

    // The number is a decimal number
    if ( base == 10 ) {
        while ( isdigit( chr ) ) {
            // result = result * 10 + (chr - '0');
            result <<= 1;
            result += (result << 2);
            result += (chr - '0');
            chr = *(++str);
        }
    }
    // The number is a hexadecimal number
    else if ( base == 16 ) {
        while ( isxdigit( chr ) ) {
            chr = toupper( chr );
            result <<= 4;
            if ( chr < 'A' ) {
                result += (chr - '0');
            }
            else {
                result += (chr - ('A' - 10)); //'7'
            }
            chr = *(++str);
        }
    }

    if ( (base == 10) && (sign == 1) ) {
        result = -result;
    }

    return result;
}

inline char *strupr( char *s )
{
	char *tmp = s;
	while ( *s != '\0' ) {
		*s = toupper( *s );
		s++;
	}
	return tmp;
}

void console_buffer_clear ( void )
{
	uint16_t i = 0;
	
	for( i = 0 ; i < console.input.count ; ++i )
	console.input_buffer[i] = 0;
	console.input.count = 0;
	console.input.head = 0;
	
}

uint8_t console_process_command( console_t* console_io, uint8_t command_type )
{
#define ARGC_MAX 10

	const console_command_t *command_table;
    static console_command_t entry;
	
		uint8_t result = CR_AGAIN;
    static char *argv[ ARGC_MAX ];
    static uint8_t argc;
		
		console_io->output.count = 0;
    console_io->output.offset = 0;
	
		if ( entry.name == NULL ) {
        argc = line_to_argv( console_io->input.buffer, argv, LENGTH( argv ) );
        //argc = strsplit( command_line, argv, LENGTH( argv ), command_line );
        if ( argc == 0 ) {
            result = CR_DONE; goto EXIT;//return CR_DONE;
        }
        strupr( argv[ 0 ] );
        
        // Search for command
        command_table = g_command_list;
        //read_program_memory( command_table, &entry, sizeof( entry ) );
        entry = *command_table;
        
        // Iterating over g_command_list's
        while ( entry.name != NULL ) {
            // Look for an exact match to the user query
            //if ( strcmppgm2ram( entry.name, argv[ 0 ] ) == 0 ) {
            if ( strcmp( entry.name, argv[ 0 ] ) == 0 ) {
                // An exact match found
                if ( !(entry.type & command_type) ) {
                    result = CR_DONE;
                }
                else if ( CLI_OPENED == console.state ) {
                    printf( "\r\n" );
                    result = CR_AGAIN; goto EXIT;//return CR_AGAIN;
                }    
                break;
            }
            else {
                ++command_table;
                //read_program_memory( command_table, &entry, sizeof( entry ) );
                entry = *command_table;
            }
        }
    }

    if ( ( entry.name != NULL ) && ( result == CR_DONE ) ) {
        // The command was found, but the number of parameters with the command
        // was incorrect.
        //strncpy( buffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n\r\n", length );
        entry.name = NULL;
        *console_io->input.buffer = '\0';
    }
    else if ( entry.name != NULL ) {
        
        // Call the callback function that is registered to this command.
       // result = entry.handler( &(console_io->output), argc, argv );
        result = entry.handler( argc, argv );

        // If result is FALSE, then no further strings will be returned
        // after this one, and cli_command can be reset to NULL ready to search
        // for the next entered command.
        if ( result == CR_DONE ) {
            entry.name = NULL;
            *console_io->input.buffer = '\0';
        }
    }
    else {
        // cli_command was NULL, the command was not found.
        //strncpy( buffer, "Command not recognised.  Enter 'help' to view a list of available commands.\r\n\r\n", length );
        if ( CT_ONLINE == command_type ) {
						RS_485_SEND()
            printf( CLI_MSG_UNKNOWN_COMMAND );
						RS_485_RECEIVE() 
            if ( console_io->input.count >= console_io->input.size ) {
                console_io->input.count = console_io->input.size;
            }
        }
        
        result = CR_DONE;
    }

EXIT:;
    return result;
}



/************************ (C) COPYRIGHT  *****END OF FILE****/

