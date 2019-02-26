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
#include "meteo.h"
#include "eeprom.h"
#include "dsp.h"
#include "adc.h"

/***************** Variable Initialization Block Begin ********/
extern event_t event;
extern meteo_t meteo;
extern volatile sample_t adc_channels[ 9 ];
/***************** Variable Initialization Block End **********/


#define FROUND(x) ((x) >= 0.0 ? (x) + 0.005 : (x) - 0.005)

typedef union {
    int32_t l;
    float f;
} lf_t;

size_t ftostr( float f, char *outbuf, size_t precision )
{
    lf_t x;
    char *p = outbuf;

    x.f = f;

    int8_t exp2 = (x.l >> 23);
    uint32_t mantissa = (x.l & 0x7FFFFF); // 0x7FFFFF = ((1 << 23) - 1)

    if ( exp2 == -1 ) { // ((1 << 8) - 1)
        if ( mantissa == 0 ) {
            if ( x.l < 0 ) *p++ = '-';
            *p++ = 'i';
            *p++ = 'n';
            *p++ = 'f';
        }
        else {
            *p++ = 'n';
            *p++ = 'a';
            *p++ = 'n';
        }
	*p = 0;
        return 0;
    }

    exp2 -= 127; // 127 = ((1 << 8) >> 1) - 1
    mantissa |= 0x800000; // 0x800000 = (1 << 23)
    uint32_t frac_part = 0;
    uint32_t int_part = 0;

    if ( exp2 >= 23 ) {
        int_part = mantissa << (exp2 - 23);
    }
    else if ( exp2 >= 0 ) {
        int_part = mantissa >> (23 - exp2);
        frac_part = (mantissa << (exp2 + 1)) & 0xFFFFFF; // 0xFFFFFF = (1 << (23 + 1)) - 1
    }
    else { // if ( exp2 < 0 )
        frac_part = (mantissa  & 0xFFFFFF) >> -(exp2 + 1); // 0xFFFFFF = (1 << (23 + 1)) - 1
    }

    if ( x.l < 0 ) *p++ = '-';

    if ( int_part == 0 ) {
        *p++ = '0';
    }
    else {
        utoa( p, int_part, 10 );
        while ( *p ) p++;
    }

    if ( precision != 0 ) *p++ = '.';


    size_t max = precision;

    for ( size_t m = 0; m < max; ++m ) {
        // frac_part *= 10;
        frac_part = (frac_part << 3) + (frac_part << 1);
        *p++ = (char) (frac_part >> 24) + '0';  // 24 = (23 + 1)
        frac_part &= 0xFFFFFF;  // 0xFFFFFF = (1 << (23 + 1)) - 1
    }



    *p = 0;
    return (size_t) (p - outbuf);
}


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
		printf("%s", CLI_MSG_SESSION_OPENED);
//		RS_485_RECEIVE()

    }
    
    return CR_DONE;
}
static inline uint16_t calc_wspeed( uint16_t freq )
{
    if ( freq ) {
		// - NRG #200P Wind Speed Vane
		return (uint16_t) ( 0.765 * (float) freq + 3.5 );
    }
    return 0;
}

uint8_t command_wind( uint8_t argc, char *argv[] )
{

    // operate only in off-line mode

	//flash read
//		RS_485_SEND()
		printf("WIND\t%d\t%d\t%d\t%d\r\n", calc_wspeed( meteo.wind.wind_speed ),
		calc_wspeed( meteo.wind.wind_speed_15_min ),
		calc_wspeed( meteo.wind.wind_speed_30_min ),
		calc_wspeed( meteo.wind.wind_gust_30_min ));
//		RS_485_RECEIVE()


    return CR_DONE;
}

uint8_t command_test( uint8_t argc, char *argv[] )
{

    // operate only in off-line mode

	//flash read
	for ( uint8_t channel = 0; channel < 3; ++channel ) {
	                float RT;
	                float T;
	                if ( adc_channels[ channel ].raw_sample.q.integer == INT16_MIN ) {
	                    adc_channels[ channel ].avg_sample.value = 0;
	                    RT = -999.99;
	                    T = -999.99;
	                }
	                else {
	#define     CUR_OFFSET          6
	#define     CUR_EE_OFFSET       3
	                    int16_t offset;
	                    uint16_t coefficient;
	                    float pga;
	                    pt_type_t pt_type;

	                    e2prom_ex_read ( pt_sensors[ channel ].offset, (uint8_t *) &offset );
	                    int16_t Z_sens = adc_channels[ channel ].avg_sample.q.integer - offset;

	                    uint8_t channel_cur = channel + CUR_OFFSET;
	                    uint8_t channel_ee = channel + CUR_EE_OFFSET;

	                    e2prom_ex_read ( pt_sensors[ channel_ee ].offset, (uint8_t *) &offset );
	                    int16_t Z_cur = adc_channels[ channel_cur ].avg_sample.q.integer - offset;

	                    e2prom_ex_read ( pt_sensors[ channel ].pga, (uint8_t *) &pga );
	                    e2prom_ex_read ( pt_sensors[ channel ].pt_type, (uint8_t *) &pt_type );

	                    RT = calculate_resistance( Z_cur, Z_sens, pga, pt_type );
	                    T = calculate_temperature( RT, pt_type );
	                }

	                char rt_strbuf[ 10 ];
	                char t_strbuf[ 10 ];
	                //ftostr( FROUND( RT ), rt_strbuf, 2 );
	                //ftostr( FROUND( T ), t_strbuf, 2 );
	                gcvt(FROUND(RT), 7, rt_strbuf);
	                gcvt(FROUND(T), 7, t_strbuf);

	                printf( "CH%hu: % 6d [raw] % 6d [avg] % 7s [ohm] % 7s [C]\r\n",
	                    channel,
	                    adc_channels[ channel ].raw_sample.q.integer,
	                    adc_channels[ channel ].avg_sample.q.integer,
	                    rt_strbuf,
	                    t_strbuf
	                );

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

