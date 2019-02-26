/*
 * eeprom.h
 *
 *  Created on: 25 февр. 2019 г.
 *      Author: konstantinov
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "meteo.h"
#include "stdbool.h"


#define OFFSETOF(s, m)      ( (size_t) &(((s *) 0)->m) )
#define membersizeof(s, m)  sizeof( ((s *) 0)->m )


#define e2prom_ex_read(m,d)    _eeprom_ex_read( OFFSETOF( eeprom_ex_t, m ), d, membersizeof( eeprom_ex_t, m ) )
#define e2prom_ex_write(m,d)   _eeprom_ex_write( OFFSETOF( eeprom_ex_t, m ), d, membersizeof( eeprom_ex_t, m ) )

typedef uint16_t eeaddr_t;
typedef	unsigned	size_t;

typedef enum {
    EEPROM_EMPTY = 0,
    EEPROM_TEST,
    EEPROM_ACTUAL,
    EEPROM_FULL,
} eeprom_status_t;

typedef enum {
    NO_PT = 0,
    PT100_THREE,
    PT100_FOUR,
    PT1000_THREE,
    PT1000_FOUR,
} pt_type_t;

typedef enum {
    NO_ADC = 0,
    STANDART,
} adc_type_t;

typedef struct {
    pt_type_t pt_type;
    int16_t offset;
    int16_t correction;
    uint16_t coefficient;
    float pga;
} pt_sensors_t;

typedef struct {
    adc_type_t adc_type;
    int16_t offset;
    int16_t correction;
    uint16_t coefficient;
} adc_sensors_t;

typedef struct {
    eeprom_status_t status;
    uint8_t snum[10];
    pt_sensors_t pt_sensors[MAX_PT_SENSORS];
    adc_sensors_t adc_sensors[MAX_ADC_SENSORS];
} eeprom_ex_t;


/* m24512-r */
bool eeprom_ex_init ( void );
bool check_eeprom_ex ( void );
//bool _eeprom_i2c_write ( uint16_t address , const uint8_t *buffer, size_t count );
//bool _eeprom_i2c_read ( uint16_t address , const uint8_t *buffer, size_t count );
bool _eeprom_ex_read( eeaddr_t offset, uint8_t *buffer, size_t count );
bool _eeprom_ex_write( eeaddr_t offset, uint8_t *buffer, size_t count );

#endif /* INC_EEPROM_H_ */
