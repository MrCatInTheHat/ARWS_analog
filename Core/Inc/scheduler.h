/*
 * scheduler.h
 *
 *  Created on: 19 февр. 2019 г.
 *      Author: Konstantinov
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_


#include "stdint.h"

#define TASK_NUMBER 3

typedef enum {
    adc_temp_first           = 0x94,
	adc_temp_second          = 0x92,
	adc_miscellaneous        = 0x90,
} adc_address_e;

typedef enum {
	time_to_poll_adc_ch1     = 0x01,
	calibrate_adc            = 0x02,
	time_to_start_counter    = 0x04,
    console_read_event       = 0x08,
	console_out_event        = 0x10,
	counter_ready 		     = 0x20,
	time_to_poll_adc_ch4     = 0x40,
	console_test_command     = 0x80,
	timer_5_seconds			 = 0x100,
} state_events_e;

typedef enum {
    vdd_type_no           	 = 0x00,
	vdd_type_ext             = 0x01,
	vdd_type_usb             = 0x02,
} vdd_type_e;

typedef enum {
    console_task             = 1,
    adc_task                 = 2,
    counter_task             = 3,
} task_type_e;

typedef enum {
    adc_task_group             = time_to_poll_adc_ch1 + time_to_poll_adc_ch4 + calibrate_adc,
	counter_task_group         = time_to_start_counter + counter_ready + timer_5_seconds,
	console_task_group         = console_read_event + console_out_event + console_test_command,
} task_events_e;


typedef enum {
    low_prior                = 7,    // rare priority
    medium_prior             = 4,    // meidum priority
    high_prior               = 0,    // well done priority
} task_priority_e;

typedef enum {
    deep_sleep_state         = 0,    // sleep
    idle_state               = 1,    // only
    active_state             = 2,    // active state
	test_state 				 = 3,
} state_type_e;

typedef void (*task_callback) ( struct task_t * );

typedef struct task_t {
    uint32_t event;
    task_callback callback;
    task_priority_e priority;
    state_type_e state;
    task_type_e task;
    task_events_e event_group;
};


typedef struct {
    struct task_t task[TASK_NUMBER];
    uint32_t event;
    state_type_e state;
    vdd_type_e	vdd;
} scheduler_t;

typedef uint32_t event_t;

void scheduler_init( scheduler_t *scheduler_p );
void scheduler_run( scheduler_t *scheduler_p );
void event_post( event_t *global_event, uint32_t event_set );
uint32_t event_pend( event_t *global_event );

#endif /* INC_SCHEDULER_H_ */
