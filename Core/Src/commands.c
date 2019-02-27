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
extern wgauge_t wind_gauge;
/***************** Variable Initialization Block End **********/


#define FROUND(x) ((x) >= 0.0 ? (x) + 0.005 : (x) - 0.005)

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


	int8_t index = wind_gauge.index - 1;
	                    if ( index < 0 ) index = LENGTH( wind_gauge.samples ) - 1;
	                    float F = (float) wind_gauge.samples[ index ] * (1 / 60.0);
	                    char f_strbuf[ 10 ];
	                    gcvt( F + 0.005, 7, f_strbuf );

	                    printf(
	                        "CH3:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
	                        "CH4:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
	                        "CH5:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
	                        "CH6:   % 6u [raw] % 5s [Hz]\r\n\r\n",
	                        adc_channels[ 3 ].raw_sample.q.integer,
	                        adc_channels[ 3 ].avg_sample.q.integer,
	                        (uint16_t) (((uint32_t) adc_channels[ 3 ].avg_sample.q.integer * 6144 + (32768 >> 1)) / 32768),
	                        adc_channels[ 5 ].raw_sample.q.integer,
	                        adc_channels[ 5 ].avg_sample.q.integer,
	                        (uint16_t) (((uint32_t) adc_channels[ 5 ].avg_sample.q.integer * 6144 + (32768 >> 1)) / 32768),
	                        adc_channels[ 4 ].raw_sample.q.integer,
	                        adc_channels[ 4 ].avg_sample.q.integer,
	                        (uint16_t) (((uint32_t) adc_channels[ 4 ].avg_sample.q.integer * 6144 + (32768 >> 1)) / 32768),
	                        wind_gauge.samples[ index ],
	                        f_strbuf
	                    );

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

