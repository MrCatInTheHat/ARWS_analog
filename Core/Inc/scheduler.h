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
    time_to_poll_adc         = 0x01,
	calibrate_adc            = 0x02,
	time_to_start_counter    = 0x04,
    console_read_event       = 0x08,
	console_out_event        = 0x10,
} state_events_e;


typedef enum {
    console_task             = 1,
    adc_task                 = 2,
    counter_task             = 3,
} task_type_e;

typedef enum {
    adc_task_group             = time_to_poll_adc + calibrate_adc,
	counter_task_group         = time_to_start_counter,
	console_task_group         = console_read_event + console_out_event,
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
} scheduler_t;

typedef uint32_t event_t;

void scheduler_init( scheduler_t *scheduler_p );
void scheduler_run( scheduler_t *scheduler_p );
void event_post( event_t *global_event, uint32_t event_set );
uint32_t event_pend( event_t *global_event );

#endif /* INC_SCHEDULER_H_ */
