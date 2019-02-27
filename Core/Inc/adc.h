/*
 * adc.h
 *
 *  Created on: 22 февр. 2019 г.
 *      Author: konstantinov
 */
#include "stdint.h"
#include "eeprom.h"
#include "meteo.h"

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define MAX_CODE       			32767                           // max code ADS1115
#define V_REF           		6.144                           // Reference Voltage; Vout = code*Vref/maxcode
#define V_HUM_LOW       		0.8                             // humidity's sensor low voltage
#define V_SUP           		5.0                             // Supply Voltage
#define COEF_ADD        		V_SUP*MAX_CODE*0.16/V_REF       // Divider coefficient
#define COEF_DIV        		V_SUP*MAX_CODE*0.0062           // Divider coefficient
#define TRUE_COEF1     		 	COEF_DIV*1.0546                 // True_RH divider 1 coefficient
#define TRUE_COEF2     		 	COEF_DIV*0.000216               // True_RH divider 2 coefficient; !! temperature is x10 already
#define HUM_LOW_LVL    		 	800 * MAX_CODE / ( V_REF * 1000)// V_HUM_LOW * MAX_CODE / V_REF

#define MUX_1 					0b100
#define MUX_2 					0b101
#define MUX_3 					0b110
#define MUX_4 					0b111

#define PGA_6					0b000
#define PGA_4					0b001
#define PGA_2					0b010
#define PGA_0					0b111

#define AD1115_1_ADDRESS         0x94
#define AD1115_2_ADDRESS         0x92
#define AD1115_3_ADDRESS		 0x90

#define swap_data(x)	( ( ( (uint16_t)(x) << 8 ) & 0xFF00 ) | ( ( (uint16_t)(x) >> 8 ) & 0x00FF  ) )

typedef struct {
	uint16_t ads_delay;
	uint8_t  lo_reg;
	uint8_t  hi_reg;
	uint8_t  conv_reg;
	uint8_t  conf_reg;
	uint8_t  data_len;
} ads1115_t;


typedef struct {
    union {
        uint16_t data;
        struct{
            uint8_t d15 : 1;
            uint8_t d14 : 1;
            uint8_t d13 : 1;
            uint8_t d12 : 1;
            uint8_t d11 : 1;
            uint8_t d10 : 1;
            uint8_t d9  : 1;
            uint8_t d8  : 1;
            uint8_t d7  : 1;
            uint8_t d6  : 1;
            uint8_t d5  : 1;
            uint8_t d4  : 1;
            uint8_t d3  : 1;
            uint8_t d2  : 1;
            uint8_t d1  : 1;
            uint8_t d0  : 1;
        } bitt;
    } conv;
    union {
//#pragma pack(push, 1)
        struct {

        	unsigned MODE      : 1;
        	unsigned PGA       : 3;
        	unsigned MUX       : 3;
        	unsigned OS        : 1;

        	unsigned COMP_QUE  : 2;
        	unsigned COMP_LAT  : 1;
        	unsigned COMP_POL  : 1;
        	unsigned COMP_MODE : 1;
        	unsigned DR        : 3;


        } bitt __attribute__((packed));
//#pragma pack(pop)
        uint16_t all;
    } config;
} adc_reg_t;

void adc_init( void );
void volt_meas( void );
void temp_meas ( void );
float calc_temperature( uint8_t channel );
float calc_dewpoint( float RH, float TA );
float calculate_temperature( float RT, pt_type_t pt_type );
float calculate_resistance( int16_t Z_cur, int16_t Z_sens, float pga_op, pt_type_t pt_type );
float calculate_pga( uint16_t RT, int16_t Z_sens );



#endif /* INC_ADC_H_ */
