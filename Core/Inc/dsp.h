/*
 * dsp.h
 *
 *  Created on: 22 февр. 2019 г.
 *      Author: konstantinov
 */

#ifndef INC_DSP_H_
#define INC_DSP_H_



#include <stdint.h>


#define bit_test( var, bit )    ( ( (var) & (1L << (bit)) ) != 0 )

//! UQ16.8
typedef union {
    uint8_t bytes[ 3 ];
    struct {
        uint8_t  fraction; // fractional bits
        uint16_t integer;  // integer bits (16 integer bits)
    } q;
} uint24x_t;

//! Q16.8
typedef union {
    int8_t bytes[ 3 ];
    struct {
        int8_t  fraction; // fractional bits
        int16_t integer;  // integer bits (15 integer bits + 1 sign bit)
    } q;
} int24x_t;

//! Q16.8
// Signed fixed point number (Qm.n format number):
//   m - integer part (if m is not specified it is taken as zero or one)
//   n - fractional part (if n = 0, the Q number is integer)
// Using an m+n bit signed integer container with n fractional bits:
//   its range is [-2^{m-1}, 2^{m-1} - 2^{-n}]
//   its resolution is 2^{-n}
typedef union {
    int32_t value;
    int8_t  bytes[ 4 ];
    struct {
        int8_t  fraction; // fractional bits
        int16_t integer;  // integer bits (15 integer bits + 1 sign bit)
    } q;
} fixed_t;

typedef struct {
    uint8_t head;
    int16_t buffer[ 3 ];
} median_t;

typedef struct {
    //int16_t raw_min;
    //int16_t raw_max;
    fixed_t raw_sample;
    fixed_t avg_sample;
    //uint8_t correction;
    median_t medfilt;          // buffer for median filter
} sample_t;


extern volatile sample_t adc_channels[ 9 ];
extern void median_filter( sample_t *sample );
extern void ema_filter( fixed_t *raw, fixed_t *avg );

static inline void extend_sign( fixed_t *val )
{
    if ( bit_test( val->q.integer, 15 ) ) {
        val->value |= 0xFF000000;
    }
}


#endif /* INC_DSP_H_ */
