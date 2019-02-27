/*
 * adc.c
 *
 *  Created on: 22 ôåâð. 2019 ã.
 *      Author: konstantinov
 */

#include "adc.h"
#include "dsp.h"
#include "i2c.h"
#include "meteo.h"
#include "eeprom.h"
#include "math.h"

ads1115_t ads1115 = {
		150,	// ads_delay
		0x02,	// lo_reg
		0x03,	// hi_reg
		0x00,	// conv_reg
		0x01,	// conf_reg
		0x02	// data_len
};


volatile sample_t adc_channels[ 9 ];
extern meteo_t meteo;

void adc_init ( void ) {

	uint8_t i2c_buffer_lo[2] = { 0 , 0 };
	uint8_t i2c_buffer_hi[2] = { 0x80 , 0 };

    i2c_bus_write( AD1115_1_ADDRESS, ads1115.lo_reg, i2c_buffer_lo, ads1115.data_len );
    i2c_bus_write( AD1115_1_ADDRESS, ads1115.hi_reg, i2c_buffer_hi, ads1115.data_len );

    i2c_bus_write( AD1115_2_ADDRESS, ads1115.lo_reg, i2c_buffer_lo, ads1115.data_len );
    i2c_bus_write( AD1115_2_ADDRESS, ads1115.hi_reg, i2c_buffer_hi, ads1115.data_len );

    i2c_bus_write( AD1115_3_ADDRESS, ads1115.lo_reg, i2c_buffer_lo, ads1115.data_len );
	i2c_bus_write( AD1115_3_ADDRESS, ads1115.hi_reg, i2c_buffer_hi, ads1115.data_len );


}

//--------------------------------------------------------------------


float calculate_pga( uint16_t RT, int16_t Z_sens )
{
    #define RTD_A1  6003.752345
    #define RTD_A2  60.037523
    #define RTD_A3  1.912046E-4


    float PGA = 0;

    float Z = ( (float)RT * RTD_A2 - RTD_A1 ) / ( (float)RT * RTD_A3 +1 );
    PGA =  Z_sens / Z;

    return PGA;
}

float calculate_resistance( int16_t Z_cur, int16_t Z_sens, float pga_op, pt_type_t pt_type )
{
    #define RTD_A1  6003.752345
    #define RTD_A2  60.037523
    #define RTD_A3  1.912046E-4

    #define RTD_B1  5.23E+3
    #define RTD_B2  31399624.765478
    #define RTD_B3  313996.247655

    #define MAX_CODE 32.768
    //#define PGA 		8
    #define REF_VALUE 2.048
    //#define OP_AMP_PGA 8.9518


    if ( pt_type == PT100_THREE ){
        double Z_new = 0;

        Z_new = Z_sens / pga_op;
        float RT = ( Z_new + RTD_A1 ) / ( RTD_A2 - Z_new * RTD_A3 );

        return RT;
    } else if ( pt_type == PT100_FOUR ) {
        double Z_new = 0;

        Z_new = Z_sens;
        float RT = ( Z_new + RTD_A1 ) / ( RTD_A2 - Z_new * RTD_A3 );

        return RT;

    }
}

// Determine temperature from a measured resistance
float calculate_temperature( float RT, pt_type_t pt_type )
{

    #define RTD_C1   0.13181768625E+8
    #define RTD_C2  -0.17316017316E+5
    #define RTD_C3   0.3383809524E+4

    #define RTD_A   3.9083E-3
    #define RTD_B   -5.775E-7
    #define RTD_D1  255.819
    #define RTD_D2  9.14550
    #define RTD_D3  -2.92363
    #define RTD_D4  1.79090

    float T;
    float p = RT * 0.01 - 1.0; // Rt/R0 - 1 = Rt/100 - 1
    if ( p < 0.0 ) { // Rt/R0 < 1 (t < 0 C)
        // an approximate inverse of Callendar-Van Dusen Eq. within 0.002 C
        T = RTD_D3 + p * RTD_D4;
        T *= p;
        T += RTD_D2;
        T *= p;
        T += RTD_D1;
        T *= p;
    }
    else {
        T = RTD_C3 - sqrt( RTD_C1 + RTD_C2 * RT );
    }

    return T;
}
// calculates dew point
// input:   humidity [%RH], temperature [°C]
// output:  dew point [°C]
float calc_dewpoint( float RH, float TA )
{
    // -40C <= T <= +60C, +-0.35C
    #define LN100   4.605170185988092
    #define DP_B    17.62
    #define DP_C    243.12

    float gamma = ( log( RH ) - LN100 ) + ( DP_B * TA ) / ( DP_C + TA );
    float t_dp = ( (DP_C * 10.0) * gamma ) / ( DP_B - gamma ); // x 10 C
    return t_dp;
    // Round to the nearest tenth
    //return (t_dp < 0.0 ? t_dp - 0.5 : t_dp + 0.5);
}

float calc_temperature( uint8_t channel )
{
#define     CUR_OFFSET          6
#define     CUR_EE_OFFSET       3

    union {
        int16_t offset;
        int16_t correction;
    } u;

    e2prom_ex_read ( pt_sensors[ channel ].offset, (uint8_t *) &u.offset );
    int16_t Z_sens = adc_channels[ channel ].avg_sample.q.integer - u.offset;

    uint8_t channel_cur = channel + CUR_OFFSET;
    uint8_t channel_ee = channel + CUR_EE_OFFSET;

    e2prom_ex_read ( pt_sensors[ channel_ee ].offset, (uint8_t *) &u.offset );
    int16_t Z_cur = adc_channels[ channel_cur ].avg_sample.q.integer - u.offset;

    float pga;
    pt_type_t pt_type;
    e2prom_ex_read ( pt_sensors[ channel ].pga, (uint8_t *) &pga );
    e2prom_ex_read ( pt_sensors[ channel ].pt_type, (uint8_t *) &pt_type );

    float RT = calculate_resistance( Z_cur, Z_sens, pga, pt_type );
    float T = calculate_temperature( RT, pt_type );
    T = T * 10 + (float) u.correction * 0.1; // x 10 C
    return T;
    // Round to the nearest tenth
    //return (T < 0.0 ? T - 0.5 : T + 0.5);
}

void temp_meas ( void ) {

	float air_temperature;
	adc_reg_t adc_reg = { 0 };


	adc_reg.config.bitt.OS = 1;
	adc_reg.config.bitt.MUX = MUX_1;
	adc_reg.config.bitt.PGA = PGA_4;
	adc_reg.config.bitt.MODE = 1;
	adc_reg.config.bitt.DR = 0;
	adc_reg.config.bitt.COMP_MODE = 0;
	adc_reg.config.bitt.COMP_POL = 0;
	adc_reg.config.bitt.COMP_LAT = 0;
	adc_reg.config.bitt.COMP_QUE = 0x3;

	//1 - 1

	if  ( i2c_bus_write( AD1115_1_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 6 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_1_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 6 ].raw_sample.q.integer = 32767;
		else adc_channels[ 6 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	}// else HAL_Delay(ads1115.ads_delay);

	//1 - 2

	adc_reg.config.bitt.MUX = MUX_2;
	adc_reg.config.bitt.PGA = PGA_0;

	if  ( i2c_bus_write( AD1115_1_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 0 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_1_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 0 ].raw_sample.q.integer = 32767;
		else adc_channels[ 0 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	} //else HAL_Delay(ads1115.ads_delay);

	//1 - 3

	adc_reg.config.bitt.MUX = MUX_3;
	adc_reg.config.bitt.PGA = PGA_4;

	if  ( i2c_bus_write( AD1115_1_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 7 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_1_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 7 ].raw_sample.q.integer = 32767;
		else adc_channels[ 7 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	}// else HAL_Delay(ads1115.ads_delay);

	//1-4

	adc_reg.config.bitt.MUX = MUX_4;
	adc_reg.config.bitt.PGA = PGA_0;

	if  ( i2c_bus_write( AD1115_1_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 1 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_1_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 1 ].raw_sample.q.integer = 32767;
		else adc_channels[ 1 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	}// else HAL_Delay(ads1115.ads_delay);

	//2 - 1

	adc_reg.config.bitt.MUX = MUX_3;
	adc_reg.config.bitt.PGA = PGA_4;

	if  ( i2c_bus_write( AD1115_2_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 8 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_2_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 8 ].raw_sample.q.integer = 32767;
		else adc_channels[ 8 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	}// else HAL_Delay(ads1115.ads_delay);

	//2 - 2

	adc_reg.config.bitt.MUX = MUX_4;
	adc_reg.config.bitt.PGA = PGA_0;

	if  ( i2c_bus_write( AD1115_2_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 2 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_2_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 2 ].raw_sample.q.integer = 32767;
		else adc_channels[ 2 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	}// else HAL_Delay(ads1115.ads_delay);



	for ( char i = 0; i < 3; ++i ) {
		if ( adc_channels[ i ].raw_sample.q.integer == 32767 ) continue;
		median_filter( (sample_t *) &adc_channels[ i ] );
		extend_sign( (fixed_t *) &adc_channels[ i ].raw_sample );
		ema_filter( (fixed_t *) &adc_channels[ i ].raw_sample, (fixed_t *) &adc_channels[ i ].avg_sample );
	}

	for ( char i = 6; i < 9; ++i ) {
		if ( adc_channels[ i ].raw_sample.q.integer == 32767 ) continue;
		median_filter( (sample_t *) &adc_channels[ i ] );
		extend_sign( (fixed_t *) &adc_channels[ i ].raw_sample );
		ema_filter( (fixed_t *) &adc_channels[ i ].raw_sample, (fixed_t *) &adc_channels[ i ].avg_sample );
	}


	/*      Translate to physics from measurmets        */



	// T_air
	if ( adc_channels[ 0 ].raw_sample.q.integer != 32767 ) {
		air_temperature = calc_temperature( 0 );
		meteo.air.temperature = (int16_t) round( calc_temperature( 0 ) );
	}
	else {
		meteo.air.temperature = -9999;
	}

	// T_surf
	if ( adc_channels[ 1 ].raw_sample.q.integer != 32767 ) {
		meteo.surf[ 0 ].surface_temperature = (int16_t) round( calc_temperature( 1 ) );
	}
	else {
		meteo.surf[ 0 ].surface_temperature = -9999;
	}

	// T_gnd
	if ( adc_channels[ 2 ].raw_sample.q.integer != 32767 ) {
		meteo.surf[ 0 ].ground_temperature = (int16_t) round( calc_temperature( 2 ) );
	}
	else {
		meteo.surf[ 0 ].ground_temperature = -9999;
	}


	if ( meteo.air.temperature != -9999 ) {
		// Rh (HIH-4010)
		float air_humidity;
		if ( adc_channels[ 3 ].avg_sample.q.integer < HUM_LOW_LVL ) { // U < 0.8V
			air_humidity = 0.0;
		}
		else {
			air_humidity = ( (float) adc_channels[ 3 ].avg_sample.q.integer ) - COEF_ADD;
			air_humidity = air_humidity / ( TRUE_COEF1 - TRUE_COEF2 * air_temperature ); // !!! air_temperature = T x 10
			air_humidity = air_humidity * V_REF + 0.05; // fround
		}

		//air_humidity_raw = (int16_t) (air_humidity * 10.);
	int16_t correction;

		e2prom_ex_read ( adc_sensors[ 0 ].correction, (uint8_t *) &correction );
		air_humidity += (float) correction * 0.1; //(float) adc_channels[ 3 ].correction * 0.1;
		if ( air_humidity > 100.0 ) {
			air_humidity = 100.0;
		} else if ( air_humidity < 0.0 )
			air_humidity = 0.0;

		meteo.air.humidity = (int16_t) (air_humidity * 10.0);
		meteo.air.dew_point = (int16_t) round( calc_dewpoint( air_humidity, air_temperature * 0.1 ) ); // !!! air_temperature = T x 10
	}
	else {
		meteo.air.humidity = -9999;
		meteo.air.dew_point = -9999;
	}

}

void volt_meas( void ) {

	adc_reg_t adc_reg = { 0 };

	if  ( i2c_bus_write( AD1115_2_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len ) ) {
		HAL_Delay(ads1115.ads_delay);
		adc_channels[ 2 ].raw_sample.value = 0;
		if  ( !i2c_bus_read( AD1115_2_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len ) )
			adc_channels[ 2 ].raw_sample.q.integer = 32767;
		else adc_channels[ 2 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	} else HAL_Delay(ads1115.ads_delay);

	adc_reg.config.bitt.OS = 1;
	adc_reg.config.bitt.MUX = MUX_1;
	adc_reg.config.bitt.PGA = PGA_6;
	adc_reg.config.bitt.MODE = 1;
	adc_reg.config.bitt.DR = 0;
	adc_reg.config.bitt.COMP_MODE = 0;
	adc_reg.config.bitt.COMP_POL = 0;
	adc_reg.config.bitt.COMP_LAT = 0;
	adc_reg.config.bitt.COMP_QUE = 0x3;


	i2c_bus_write( AD1115_3_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len );       // 1 adc1115 channel

	HAL_Delay( ads1115.ads_delay );
	adc_channels[ 3 ].raw_sample.value = 0;

	i2c_bus_read( AD1115_3_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len );
	adc_channels[ 3 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	if ( ( ( adc_channels[ 3 ].raw_sample.q.integer ) >> 15  ) & 0x1 )
		adc_channels[ 3 ].raw_sample.q.integer = 0;

	median_filter( (sample_t *) &adc_channels[ 3 ] );
	ema_filter( (fixed_t *) &adc_channels[ 3 ].raw_sample, (fixed_t *) &adc_channels[ 3 ].avg_sample );



	adc_reg.config.bitt.MUX = MUX_2;
	i2c_bus_write( AD1115_3_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len );      // 2 adc1115 channel

	HAL_Delay( ads1115.ads_delay );
	adc_channels[ 5 ].raw_sample.value = 0;

	i2c_bus_read( AD1115_3_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len );
	adc_channels[ 5 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	if ( ( ( adc_channels[ 5 ].raw_sample.q.integer ) >> 15  ) & 0x1 )
	  adc_channels[ 5 ].raw_sample.q.integer = 0;

	median_filter( (sample_t *) &adc_channels[ 5 ] );
	ema_filter( (fixed_t *) &adc_channels[ 5 ].raw_sample, (fixed_t *) &adc_channels[ 5 ].avg_sample );



	adc_reg.config.bitt.MUX = MUX_3;
	i2c_bus_write( AD1115_3_ADDRESS, ads1115.conf_reg, (uint8_t *) &adc_reg.config.all, ads1115.data_len );      // 3 adc1115 channel

	HAL_Delay( ads1115.ads_delay );
	adc_channels[ 4 ].raw_sample.value = 0;

	i2c_bus_read( AD1115_3_ADDRESS, ads1115.conv_reg, (uint8_t *) &adc_reg.conv.data, ads1115.data_len );
	adc_channels[ 4 ].raw_sample.q.integer = swap_data(adc_reg.conv.data);
	if ( ( ( adc_channels[ 4 ].raw_sample.q.integer ) >> 15  ) & 0x1 )
	  adc_channels[ 4 ].raw_sample.q.integer = 0;

	median_filter( (sample_t *) &adc_channels[ 4 ] );
	ema_filter( (fixed_t *) &adc_channels[ 4 ].raw_sample, (fixed_t *) &adc_channels[ 4 ].avg_sample );



}
