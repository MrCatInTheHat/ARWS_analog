/**
  ******************************************************************************
  * @file    commands.c
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
	
#include "commands.h"
#include "console.h"
#include "stdlib.h"
#include "stdout_user.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <string.h>
#include "stddef.h"
#include "scheduler.h"
#include "usb_device.h"

/***************** Variable Initialization Block Begin ********/
extern event_t event;
/***************** Variable Initialization Block End **********/
uint8_t buffer[] = "TrapHappens";
extern USBD_HandleTypeDef hUsbDeviceFS;
/***************** Fucntion Declaration Block Begin ***********/

uint8_t command_open( uint8_t argc, char *argv[] )
{

    // operate only in off-line mode
    if ( CLI_CLOSED == console.state ) {
        console.state = CLI_OPENED;
        console.session_timeout = CONSOLE_ONLINE_SESSION_TIMEOUT;
        console.inactivity_timer = 0;
        console.inactivity_timeout = 0;        
	//flash read
//		RS_485_SEND()
//		printf("%s", CLI_MSG_SESSION_OPENED);
//		RS_485_RECEIVE()
        USBD_CDC_SetTxBuffer(&hUsbDeviceFS,buffer,sizeof(buffer));
	    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    }
    
    return CR_DONE;
}

uint8_t command_close( uint8_t argc, char *argv[] )
{
    static uint8_t state;
    
    if ( 0 == state ) {
		RS_485_SEND()
        printf("%s", CLI_MSG_SESSION_CLOSED);	 
		RS_485_RECEIVE()
        ++state;
    }
    else {
     //   console_unlock_port();
        
        console.state = CLI_CLOSED;
        console.session_timeout = CONSOLE_OFFLINE_SESSION_TIMEOUT;
        console.inactivity_timer = CONSOLE_INACTIVITY_TIMEOUT;
        console.inactivity_timeout = CONSOLE_INACTIVITY_TIMEOUT;
        
        state = 0;
    }

    return ( 0 == state ? CR_DONE : CR_AGAIN );
}




/***************** Fucntion Declaration Block End  *************/

/***************** Command Table List **************************/

/************************ (C) COPYRIGHT  *****END OF FILE****/

