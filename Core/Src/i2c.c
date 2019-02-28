/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "stdbool.h"
#include "eeprom.h"
/* USER CODE BEGIN 0 */
#define LOW_BYTE(x)     ((unsigned char)((x)&0xFF))
#define HIGH_BYTE(x)    ((unsigned char)(((x)>>8)&0xFF))
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
_Bool i2c_bus_write( uint8_t address, uint8_t opcode, const uint8_t *buffer, uint8_t length ){
#define 	TIMEOUT 		5
#define 	OPCODE_SIZE		1
#define 	I2C_BUFFER_SIZE	5

	_Bool result = false;
	uint8_t i = 0;
	uint8_t i2c_buffer[I2C_BUFFER_SIZE] = { 0 };

	i2c_buffer[0] = opcode;
	for ( i = 0; i < length; ++i )
		i2c_buffer[i + OPCODE_SIZE] = buffer[i];

	if ( HAL_I2C_Master_Transmit(&hi2c1, address, i2c_buffer, length + OPCODE_SIZE, TIMEOUT) == HAL_OK) result = true;


	return result;

}

_Bool i2c_bus_read( uint8_t address, uint8_t opcode, const uint8_t *buffer, uint8_t length ){

	_Bool result = false;

	if ( HAL_I2C_Master_Transmit(&hi2c1, address, &opcode, OPCODE_SIZE, TIMEOUT) == HAL_OK) result = true;
	if ( HAL_I2C_Master_Receive(&hi2c1, address, buffer, length, TIMEOUT) == HAL_OK) result = true;

	return result;
}


bool i2c_bus_eeprom_write( uint8_t address, uint16_t eeprom_addr, uint8_t *buffer, uint8_t length ){
#define ADDRESS_SIZE  2
	bool result = false;
	uint8_t i = 0;
	uint8_t i2c_buffer[sizeof(eeprom_ex_t)+2] = { 0 };

	i2c_buffer[0] = HIGH_BYTE( eeprom_addr );
	i2c_buffer[1] = LOW_BYTE ( eeprom_addr );


	for ( i = 2; i < sizeof(i2c_buffer); ++i )
		i2c_buffer[i] = buffer[i - 2];

	if ( HAL_I2C_Master_Transmit(&hi2c1, address, &i2c_buffer[0], length + ADDRESS_SIZE, TIMEOUT*3) == HAL_OK) result = true;


	return result;

}




bool i2c_bus_eeprom_read( uint8_t address, uint16_t eeprom_addr, uint8_t *buffer, uint8_t length ){

	bool result = false;
	uint8_t i2c_buffer[2] = { 0 };

	i2c_buffer[0] = HIGH_BYTE( eeprom_addr );
	i2c_buffer[1] = LOW_BYTE ( eeprom_addr );

	if ( HAL_I2C_Master_Transmit(&hi2c1, address, &i2c_buffer[0], 2, TIMEOUT*2) == HAL_OK) result = true;
	if ( HAL_I2C_Master_Receive(&hi2c1, address, buffer, length, TIMEOUT*3) == HAL_OK) result = true;

	result = true;

	return result;

}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
