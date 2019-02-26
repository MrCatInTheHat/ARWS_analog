/*
 * dsp.c
 *
 *  Created on: 22 февр. 2019 г.
 *      Author: konstantinov
 */

#include "dsp.h"

void median_filter( sample_t *sample )
{
    sample->medfilt.buffer[ sample->medfilt.head ] = sample->raw_sample.q.integer;
    if ( ++sample->medfilt.head >= 3 ) sample->medfilt.head = 0;

    #define a sample->medfilt.buffer[ 0 ]
    #define b sample->medfilt.buffer[ 1 ]
    #define c sample->medfilt.buffer[ 2 ]

    int16_t middle;
    if ( a < b ) {
        if ( a < c ) {
            if ( b < c ) {
                // a < b < c
                middle = b;
            }
            else {
                // a < c < b
                middle = c;
            }
        }
        else {
            // c < a < b
            middle = a;
        }
    }
    else {
        if ( b < c ) {
            if ( a < c ) {
                // b < a < c
                middle = a;
            }
            else {
                // b < c < a
                middle = c;
            }
        }
        else {
            // c < b < a
            middle = b;
        }
    }

    sample->raw_sample.q.integer = middle;
}


#define BITS_PER_SAMPLE     16
#define FRACTION_SHIFT      4
#define DIFF_SIGN_BIT       (31 - FRACTION_SHIFT)
#define SIGN_EXTENSION_MASK (((1UL << FRACTION_SHIFT) - 1) << (32 - FRACTION_SHIFT))

/*!
    Simple single pole low pass filter can be realized with the algorithm:
    FILT <-- FILT + FF * (NEW - FILT)
    When FF is the "filter fraction" is 1 / 2^N
*/
void ema_filter( fixed_t *raw, fixed_t *avg )
{
//    if ( bit_test( raw->q.integer, 15 ) ) {
//        raw->value |= 0xFF000000;
//    }

    fixed_t rawval, avgval;
    rawval.value = raw->value;
    avgval.value = avg->value;

    // Implementation of AVG += (RAW - AVG) / 2^FRACTION_SHIFT
    rawval.value -= avgval.value;
    rawval.value >>= FRACTION_SHIFT;

    // Right-shifting a signed value will involve sign extension
    if ( bit_test( rawval.value, DIFF_SIGN_BIT ) ) {
        rawval.value |= SIGN_EXTENSION_MASK;
    }

    avgval.value += rawval.value;
    avg->value = avgval.value;
}
