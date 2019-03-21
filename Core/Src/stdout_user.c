/*-----------------------------------------------------------------------------
 * Name:    stdout_user.c
 * Purpose: STDOUT User Template
 * Rev.:    1.0.0
 *-----------------------------------------------------------------------------*/
 
/* Copyright (c) 2013 - 2015 ARM LIMITED
 
   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
 
#include "stdout_user.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "scheduler.h"

extern UART_HandleTypeDef huart1;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern scheduler_t scheduler;

#define RS_485_SEND() 	{ \
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); \
					HAL_Delay(1);  \
					}

#define RS_485_RECEIVE()  { \
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); \
					}


int _write(int fd, char *str, int len);
/**
  Put a character to the stdout
 
  \param[in]   ch  Character to output
  \return          The character written, or -1 on write error.
*/
int _write(int fd, char *str, int len){

	if ( scheduler.vdd == vdd_type_usb ) {
		USBD_CDC_SetTxBuffer(&hUsbDeviceFS,str,len);
    	USBD_CDC_TransmitPacket(&hUsbDeviceFS);
	} else {
		RS_485_SEND()
		HAL_UART_Transmit(&huart1, str, len, 50 );
		RS_485_RECEIVE()
	}

  return len;

}

