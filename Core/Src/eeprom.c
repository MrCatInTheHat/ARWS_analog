/*
 * eeprom.c
 *
 *  Created on: 25 февр. 2019 г.
 *      Author: konstantinov
 */
#include "meteo.h"
#include "adc.h"
#include "eeprom.h"
#include "i2c.h"


#define ADDRESS_M24512          0xA0
#define MAX_SIZE                sizeof(eeprom_ex_t)
#define START_ADDR_LO           0x00
#define START_ADDR_HI           0x00
#define ADDR_SIZE               0x02
#define EEPROM_EX_MIN_ADDR      0x00
#define EEPROM_EX_MAX_ADDR      0x7F


const eeprom_ex_t eeprom_ex = {
    /* .status */
    { EEPROM_TEST },
    /* .snum */
    { 0 , 0, 0, 0, 0, 0, 0, 0, 0 },
    /* .pt_sensors */
    {
        //pt_sensor[0]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },
        //pt_sensor[1]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },
        //pt_sensor[2]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },
        //pt_sensor[3]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },
        //pt_sensor[4]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },
        //pt_sensor[5]
        {
            /* .pt_type */          PT100_THREE,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0,
            /* .pga */              8.951807
        },

    },
    /* .adc_sensors */
    {
        //adc_sensor[0]
        {
            /* .adc_type */         STANDART,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0
        },
        //adc_sensor[1]
        {
            /* .adc_type */         STANDART,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0
        },
        //adc_sensor[2]
        {
            /* .adc_type */         STANDART,
            /* .offset */           0,
            /* .correction */       0,
            /* .coefficient */      0
        },
    },

};

bool eeprom_ex_init ( void )
{
    bool result = false;

    if ( _eeprom_ex_write( 0, (uint8_t*)&eeprom_ex, sizeof( eeprom_ex_t ) ) )
        result = true;

    return result;
}


bool check_eeprom_ex ( void )
{
    bool result = false;
    eeprom_ex_t eeprom_test = { 0xFF };

    for ( uint8_t i = 0; i < sizeof(eeprom_ex_t); ++i )
        *(((uint8_t*)&eeprom_test) + i) = 0xFF;

    _eeprom_ex_read( 0, (uint8_t*)&eeprom_test, sizeof( eeprom_ex_t ) );


    if ( ( eeprom_test.status != EEPROM_EMPTY ) || ( eeprom_test.status != EEPROM_TEST ) )
        result = eeprom_ex_init();

    return result;
}

bool _eeprom_ex_read( eeaddr_t offset, uint8_t *buffer, size_t count )
{
    bool result = false;
    //uint8_t buffer_addr[ADDR_SIZE] = { 0x00 };

    if ( ( offset > EEPROM_EX_MAX_ADDR )
        || ( ( offset + count ) > EEPROM_EX_MAX_ADDR )
        || ( buffer == NULL )
        || ( count == 0 ) ) {
        return false;
    }


    if ( i2c_bus_eeprom_read( ADDRESS_M24512, offset, buffer, count) )
        result = true;

    return result;
}


bool _eeprom_ex_write( eeaddr_t offset, uint8_t *buffer, size_t count )
{
    bool result = false;


    if ( ( offset > EEPROM_EX_MAX_ADDR )
        || ( ( offset + count ) > EEPROM_EX_MAX_ADDR )
        || ( buffer == NULL )
        || ( count == 0 ) ) {
        return false;
    }

    if ( i2c_bus_eeprom_write( ADDRESS_M24512, offset, buffer, count ) )
        result = true;

    return result;
}


