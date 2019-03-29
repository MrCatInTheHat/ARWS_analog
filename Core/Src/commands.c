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
#include "ctype.h"

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

	if  ( argc == 3  ) {
		strupr( argv[ 1 ] );
		if ( ( str2int( argv[ 2 ] ) == 1 ) && (  0 == strcmp( "ANALOG", argv[ 1 ] ) ) ) {

			if ( CLI_CLOSED == console.state ) {
				console.state = CLI_OPENED;
				console.session_timeout = CONSOLE_ONLINE_SESSION_TIMEOUT;
				console.inactivity_timer = 0;
				console.inactivity_timeout = 0;

				printf("%s", CLI_MSG_SESSION_OPENED);
			}
		}
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


	printf("WIND\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", calc_wspeed( meteo.wind.wind_speed ),
		calc_wspeed( meteo.wind.wind_speed_15_min ),
		calc_wspeed( meteo.wind.wind_speed_30_min ),
		calc_wspeed( meteo.wind.wind_gust_30_min ),
		meteo.wind.wind_dir,
		meteo.wind.wind_dir_15_min,
		meteo.wind.wind_dir_30_min);




    return CR_DONE;
}


uint8_t command_air( uint8_t argc, char *argv[] )
{


	printf("AIR\t%d\t%d\t%d\t%d\t%d\r\n",
			meteo.air.temperature,
			meteo.air.humidity,
			meteo.air.dew_point,
			meteo.surf[0].surface_temperature,
			meteo.surf[0].ground_temperature);


    return CR_DONE;
}


uint8_t command_test( uint8_t argc, char *argv[] )
{

    // operate only in off-line mode
	event_post(&event, timer_5_seconds);
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
		if ( RT < 100 ) gcvt(FROUND(RT), 5, rt_strbuf);
		else gcvt(FROUND(RT), 6, rt_strbuf);
		if ( ( T < 10 ) && ( T > -10 ) ) gcvt(FROUND(T), 3, t_strbuf);
		else gcvt(FROUND(T), 4, t_strbuf);

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
	if ( F < 10 ) gcvt( F + 0.005, 3, f_strbuf );
	else gcvt( F + 0.005, 4, f_strbuf );


	printf(
		"CH3:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
		"CH4:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
		"CH5:   % 6u [raw] % 5u [avg] % 7u [mV]\r\n"
		"CH6:   % 6u [raw] % 5s [Hz]\r\n\r\n\r\n",
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


static inline float calc_humidity( uint8_t channel )
{
   /* #define MAX_CODE        32767                           // max code ADS1115
    #define V_REF           6.144                           // Reference Voltage; Vout = code*Vref/maxcode
    #define V_HUM_LOW       0.8                             // humidity's sensor low voltage
    #define V_SUP           5.0                             // Supply Voltage
    #define COEF_ADD        V_SUP*MAX_CODE*0.16/V_REF       // Divider coefficient
    #define COEF_DIV        V_SUP*MAX_CODE*0.0062           // Divider coefficient
    #define TRUE_COEF1      COEF_DIV*1.0546                 // True_RH divider 1 coefficient
    #define TRUE_COEF2      COEF_DIV*0.000216               // True_RH divider 2 coefficient; !! temperature is x10 already
    #define HUM_LOW_LVL     800 * MAX_CODE / ( V_REF * 1000)// V_HUM_LOW * MAX_CODE / V_REF
*/
    float air_humidity;

    if ( adc_channels[ 3 ].avg_sample.q.integer < HUM_LOW_LVL ) { // U < 0.8V
        air_humidity = 0.0;
    }
    else {
        air_humidity = ( (float) adc_channels[ channel ].avg_sample.q.integer ) - COEF_ADD;
        air_humidity = air_humidity / ( TRUE_COEF1 - TRUE_COEF2 * (float) meteo.air.temperature ); // !!! air_temperature = T x 10
        air_humidity = air_humidity * V_REF + 0.05; // fround
    }

    return air_humidity;
}

uint8_t command_calibrate( uint8_t argc, char *argv[] )
{


#define HUMIDIY_CAL_CHANNEL 3
   int16_t humidity;
   int16_t correction;

   if ( argc > 1 ) {

       char sid = toupper( *argv[ 1 ] );

       switch ( sid ) {
       case 'T':
           sid = argv[ 1 ][ 1 ];
           if ( argc == 2 && sid != '\0' && argv[ 1 ][ 2 ] == '\0' ) {

        	   uint32_t channel_number;
               if ( isdigit( sid ) && (channel_number = sid - '0') < 3 ) {
            	   int16_t offset = adc_channels[ channel_number ].avg_sample.q.integer;

				   if ( !e2prom_ex_write(pt_sensors[channel_number].offset, (uint8_t *) &offset) ) {
					   goto ERROR;
				   }

				   e2prom_ex_read ( pt_sensors[ channel_number ].offset, (uint8_t *) &offset );
				   printf("T%hu: %d\t%d\r\n",
					channel_number,
					adc_channels[ channel_number ].avg_sample.q.integer,
					offset
					);
               }
               else if ( toupper( sid ) == 'A' ) {
                   int16_t offsets[ 3 ];
                   offsets[ 0 ] = adc_channels[ 0 ].avg_sample.q.integer;
                   offsets[ 1 ] = adc_channels[ 1 ].avg_sample.q.integer;
                   offsets[ 2 ] = adc_channels[ 2 ].avg_sample.q.integer;

                   if ( e2prom_ex_write(pt_sensors[0].offset, (uint8_t *) &offsets[0]) ) {
                       goto ERROR;
                   }
                   if ( e2prom_ex_write(pt_sensors[1].offset, (uint8_t *) &offsets[1]) ) {
                       goto ERROR;
                   }
                   if ( e2prom_ex_write(pt_sensors[2].offset, (uint8_t *) &offsets[2]) ) {
                       goto ERROR;
                   }


			   e2prom_ex_read ( pt_sensors[ 0 ].offset, (uint8_t *) &offsets[0] );
			   e2prom_ex_read ( pt_sensors[ 1 ].offset, (uint8_t *) &offsets[1] );
			   e2prom_ex_read ( pt_sensors[ 2 ].offset, (uint8_t *) &offsets[2] );

			   printf( "T0: %d\t%d\r\nT1: %d\t%d\r\nT2: %d\t%d\r\n",
                       adc_channels[ 0 ].avg_sample.q.integer, offsets[ 0 ],
                       adc_channels[ 1 ].avg_sample.q.integer, offsets[ 1 ],
                       adc_channels[ 2 ].avg_sample.q.integer, offsets[ 2 ]
                   );
               }
               else {
                   goto ERROR;
               }
           }
           else {
               goto ERROR;
           }
           break;

       case 'H':

           humidity = (int16_t) (calc_humidity( HUMIDIY_CAL_CHANNEL ) * 10.);
           if ( argc == 3 ) {
               correction = str2int( argv[ 2 ] );
               correction -= humidity;

               if ( !e2prom_ex_write ( adc_sensors[ 0 ].correction, (uint8_t *) &correction ) ) {
                   goto ERROR;
               }
           }

           e2prom_ex_read ( adc_sensors[ 0 ].correction, (uint8_t *) &correction );
           printf( "RH: %d %%\t%d\r\n", humidity, correction );
           break;

       case 'C':


           sid = argv[ 1 ][ 1 ];
           if ( ( argc == 3 ) && ( sid != '\0' ) && ( argv[ 1 ][ 2 ] == '\0' ) ) {
		   static uint8_t channel_number;
		   static int16_t offset;
		   static uint16_t coefficient;
	       static uint16_t resistance = 0;
		   static double coefficient_d = 0;
		   static float RT;
		   static float T;

		   if ( isdigit( sid ) && ( (channel_number = sid - '0') < 3 ) ) {

			   e2prom_ex_read ( pt_sensors[ channel_number ].coefficient, (uint8_t *) &coefficient );
			   e2prom_ex_read ( pt_sensors[ channel_number ].offset, (uint8_t *) &offset );
			   int16_t Z = adc_channels[ channel_number ].avg_sample.q.integer - offset;


			   if ( 0 == strcmp( "DEL", strupr( argv[ 2 ] ) ) ) {
				   coefficient = 0;
				   float pga = 0;
				   if ( !e2prom_ex_write ( pt_sensors[ channel_number ].pga, (uint8_t *) &pga ) ) {
					   goto ERROR;
				   }
			   } else {
				   float pga = 0;

				   resistance = str2int( argv[ 2 ] );
				   pga = calculate_pga( resistance, Z );

				   if ( !e2prom_ex_write ( pt_sensors[ channel_number ].pga, (uint8_t *) &pga ) ) {
					   goto ERROR;
				   }
			   }

		   }

		   float pga = 0;
		   char pga_strbuf[ 10 ];
		   e2prom_ex_read ( pt_sensors[ channel_number ].pga, (uint8_t *) &pga );
		   gcvt( FROUND(pga), 5, pga_strbuf );
		   printf( "C%hu: \t% 10s\r\n",
		   channel_number,
		   pga_strbuf
		   );
	   }

       break;
       default:
           goto ERROR;
       }

   }
   else {
       goto ERROR;
   }

   return CR_DONE;

ERROR:

	printf("%s", CLI_MSG_SESSION_ERROR);


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



uint8_t command_reply_data( uint8_t argc, char *argv[] )
{
	uint32_t address = 0;


	if ( argc == 3 ) {
		strupr( argv[ 2 ] );
		address = str2int( argv[ 1 ] );
		if (  0 == strcmp( "MES", argv[ 2 ] ) ) {
			if ( address == 1 ){
				printf("AIR: %d %d %d %d %d WIND: %d %d %d %d %d %d %d", meteo.air.temperature,
						meteo.air.humidity,
						meteo.air.dew_point,
						meteo.surf[0].surface_temperature,
						meteo.surf[0].ground_temperature,
						calc_wspeed( meteo.wind.wind_speed ),
						calc_wspeed( meteo.wind.wind_speed_15_min ),
						calc_wspeed( meteo.wind.wind_speed_30_min ),
						calc_wspeed( meteo.wind.wind_gust_30_min ),
						meteo.wind.wind_dir,
						meteo.wind.wind_dir_15_min,
						meteo.wind.wind_dir_30_min);
				printf("\r\n");
			} else {
				//printf("Wrong address \r\n");
			}
		}
	}
	else {
		//printf("%s", CLI_MSG_UNKNOWN_COMMAND);
	}


	return CR_DONE;

}

/***************** Fucntion Declaration Block End  *************/

/***************** Command Table List **************************/

/************************ (C) COPYRIGHT  *****END OF FILE****/

