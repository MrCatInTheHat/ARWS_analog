/**
  ******************************************************************************
  * @file    console.h
  * @brief   console 
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
#include "stdint.h"
//#include "portmacro.h"
//#include "cmsis_os.h"
#include "commands.h"

#ifndef CONSOLE_H
#define CONSOLE_H
#endif


#ifdef __cplusplus
extern "C" {
#endif

/***************** Defines Block Begin ************************/
#define MAX_INPUT_BUFFER 								(256)
#define MAX_OUTPUT_BUFFER 							(128)
#define CONSOLE_OFFLINE_SESSION_TIMEOUT (0)
#define CONSOLE_ONLINE_SESSION_TIMEOUT  (90)
#define CONSOLE_INACTIVITY_TIMEOUT			(5)
#define CLI_MSG_COMMAND_PROMPT  "\r\n> "
//#define CLI_MSG_LINE_CLOSED     "\r\nLINE CLOSED\r\n"
//#define CLI_MSG_LINE_OPENED     "\r\nLINE OPENED\r\n\r\n> "
#define CLI_MSG_UNKNOWN_COMMAND "\r\nUNKNOWN COMMAND. FOR A LIST OF KNOWN COMMANDS ENTER HELP\r\n"
#define CLI_MSG_SESSION_CLOSED  "\r\nSESSION CLOSED\r\n"
#define CLI_MSG_SESSION_OPENED  "\r\nSESSION OPENED\r\n"
#define CLI_MSG_LINK_OPENED     "LINK OPENED\r\n"
#define CLI_MSG_LINK_CLOSED     "\r\nLINK CLOSED"
#define CLI_MSG_LINE_BUSY       "LINE IS BUSY, TRY AGAIN LATER\r\n" // Lines are temporarily busy, please try again later,
/***************** Defines Block End **************************/

/***************** Structures Define Block Begin **************/
typedef volatile struct {
    uint8_t *buffer;    //!< the physical memory address where the buffer is stored
    uint16_t size;      //!< the allocated size of the buffer
    uint16_t count;     //!< the length of the data currently in the buffer
    uint16_t offset;    //!< the index into the buffer where the data starts
	uint16_t head;
	uint16_t tail;
//    EventGroupHandle_t complete;//done, ready, notify complete_event
//	  EventBits_t event_bits;
	uint8_t state;
} stream_t;


typedef struct {
    uint8_t state;
    uint32_t session_timeout;
//    osSemaphoreId  semaphore;
    uint32_t event_wait_time;
//    EventGroupHandle_t event;//done, ready, notify complete_event
    uint32_t event_bits;
	
    // Sets the inactivity timeout for your terminal session. 
    // The range is from 0 to 525600 minutes (8760 hours). 
    // A value of 0 minutes disables the session timeout. The default is 0. 
    uint32_t inactivity_timer;
    uint32_t inactivity_timeout; //! Specifies the number of minutes a session can be inactive before it is terminated. 
    

		uint8_t input_buffer[MAX_INPUT_BUFFER];
		stream_t input;
		
		uint8_t output_buffer[MAX_OUTPUT_BUFFER];
		stream_t output; 
		
} console_t;

extern console_t console;


//typedef uint8_t (* command_handler_t)( stream_t *io, uint8_t argc, char *argv[] );
typedef uint8_t (* command_handler_t)( uint8_t argc, char *argv[] );	

typedef struct {
    const char *name;
    command_handler_t handler;
    const char *help;
    uint8_t type;
} console_command_t;	

typedef enum {
    CLI_CLOSED = 0,
    CLI_OPENED,
} cli_state_t;

enum {
    CIS_IDLE  = (1 << 0),
    CIS_RECV  = (1 << 1),
    CIS_STOP  = (1 << 2),
		CIS_TRANS = (1 << 3),
};

typedef enum {
   CT_ONLINE    = (1 << 0),
   CT_OFFLINE   = (1 << 1),
   CT_UNICAST   = (1 << 2),
   CT_BROADCAST = (1 << 3),
   CT_NETWORK   = (CT_UNICAST | CT_BROADCAST)
} cmd_type_t;


typedef enum {
    CR_DONE  = (1 << 0),
    CR_AGAIN = (1 << 1),
    CR_WAIT  = (1 << 2),
	  CR_ERROR = (1 << 3),
} cmd_result_t;


/***************** Structures Define Block End  ***************/

/***************** Functions Macro Define Block Begin *********/
/***************** Functions Macro Define Block End  **********/
	
	
/***************** Function Initialization Block Begin ********/	
void console_init( void );
int  console_lock_port( void );
void console_unlock_port( void );
void console_handle_input( uint8_t port, char rxdata );
uint8_t console_process_command( console_t* console_io, uint8_t command_type );
void console_buffer_clear ( void );
int32_t str2int( char *str );
/***************** Function Initialization Block End **********/	
	/***************** Command Table List **************************/

		
	
/************************ (C) COPYRIGHT  *****END OF FILE****/
