/**
  ******************************************************************************
  * @file    commands.h
  * @brief   commands 
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

#include "stdbool.h"



#ifndef COMMANDS_H
#define COMMANDS_H
#endif

#ifdef __cplusplus
extern "C" {
#endif

/***************** Defines Block Begin ************************/
#define RS_485_SEND() 	{ \
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); \
					HAL_Delay(1);	\
					} 

#define RS_485_RECEIVE()  { \
					HAL_Delay(1);			\
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); \
					}	
/***************** Defines Block End **************************/
/*
typedef struct {
    uint32_t image_size;
    uint32_t image_crc;
    uint32_t signature;
    uint32_t address;
    uint32_t reserved[ 124 ];
    uint32_t image[];
} firmware_metadata_t;

*/
/***************** Structures Define Block Begin **************/

/***************** Structures Define Block End  ***************/

/***************** Functions Macro Define Block Begin *********/
/***************** Functions Macro Define Block End  **********/
	
	
/***************** Function Initialization Block Begin ********/	
uint8_t command_open( uint8_t argc, char *argv[] );
uint8_t command_close( uint8_t argc, char *argv[] );
uint8_t command_wind( uint8_t argc, char *argv[] );
uint8_t command_test( uint8_t argc, char *argv[] );
uint8_t command_air( uint8_t argc, char *argv[] );
uint8_t command_calibrate( uint8_t argc, char *argv[] );
uint8_t command_reply_data( uint8_t argc, char *argv[] );
uint8_t command_air( uint8_t argc, char *argv[] );
/***************** Function Initialization Block End **********/	

		
	
/************************ (C) COPYRIGHT  *****END OF FILE****/
