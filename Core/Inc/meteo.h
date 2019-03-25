/*
 * meteo.h
 *
 *  Created on: 22 февр. 2019 г.
 *      Author: konstantinov
 */

#ifndef INC_METEO_H_
#define INC_METEO_H_

#include "stdint.h"


#define MAX_PT_SENSORS      6
#define MAX_ADC_SENSORS     3

enum { WS_INTEGRATION_TIME = 30 };
enum { WD_INTEGRATION_TIME = 30 };

typedef struct {
    uint8_t  index;
    uint8_t  index_15;
    uint8_t  sample_number;
    uint8_t  sample_number_15;
    uint32_t total_sum;
    uint32_t total_sum_15;
    uint16_t samples[ WS_INTEGRATION_TIME ];
} wgauge_t;


typedef enum {
    AMA_INIT = 0,
    AMA_RUN
} TAMA_ST;

typedef struct {
    TAMA_ST state;
    int16_t angle;
    int16_t total_sum; // max. 90 samples
    uint8_t sample_number;
} TAMA, *PAMA; // Arithmetic Mean Angle

typedef struct {
    uint8_t index;
    uint8_t index_15;
    //sint16_t total_sum;
    int16_t total_sum_15;
    uint8_t sample_number_15;
    TAMA minute_mean;
    TAMA half_hourly_mean;
    int16_t samples[ WD_INTEGRATION_TIME ];
} wvane_t;

typedef struct {
   int16_t temperature;
   int16_t temperature_1ch;
   int16_t temperature_2ch;
   int16_t temperature_3ch;
   int16_t humidity;
   int16_t dew_point;  // ? without pressure is it real
} air_t;

typedef struct {
	int16_t surface_temperature;
	int16_t ground_temperature;
} surf_t;

typedef struct {
	uint16_t counter;
    int16_t wind_speed;
    int16_t wind_speed_15_min;
    int16_t wind_speed_30_min;
    int16_t wind_gust_30_min;
    int16_t wind_dir;
    int16_t wind_dir_15_min;
    int16_t wind_dir_30_min;
} wind_t;


typedef volatile struct {
   air_t        air;
   wind_t       wind;
   surf_t		surf[2];
} meteo_t;

typedef struct {
    //uint8_t index;
    uint8_t testpoints[ 3 ];
} pgm_flow_t;




#endif /* INC_METEO_H_ */
