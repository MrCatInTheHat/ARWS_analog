/*
 * scheduler.c
 *
 *  Created on: 19 февр. 2019 г.
 *      Author: Konstantinov
 */

#include "scheduler.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
/*********************************************************************
 * @fn      Scheduler init function
 *
 * @brief   Perform a scheduler init
 *
 *
 * @param   None.
 *
 * @return  None.
 */
extern USBD_HandleTypeDef hUsbDeviceFS;


void task_adc ( struct task_t * task )
{

    uint8_t event_flag = 1;

    task->state = idle_state;

    while ( task->event ) {

          switch( task->event & event_flag ){

          case time_to_poll_adc:

              break;

          case calibrate_adc:

              break;


          default:

              break;
          }
        task->event &= ~event_flag;
        event_flag <<= 1;
        if ( !event_flag ) event_flag = 1;
    }


}

void task_counter ( struct task_t * task )
{

    uint8_t event_flag = 1;


    task->state = idle_state;

    while ( task->event ) {

          switch( task->event & event_flag ){

          case time_to_start_counter:

              break;

          default:

              break;
          }
        task->event &= ~event_flag;
        event_flag <<= 1;
        if ( !event_flag ) event_flag = 1;
    }


}

void task_console ( struct task_t * task )
{

    uint8_t event_flag = 1;
    uint8_t buffer[] = "Hello, kitty!";

    task->state = idle_state;

    while ( task->event ) {

          switch( task->event & event_flag ){

          case console_read_event:

              break;

          case console_out_event:
        	  USBD_CDC_SetTxBuffer(&hUsbDeviceFS,buffer,sizeof(buffer));
			  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
              break;


          default:

              break;
          }
        task->event &= ~event_flag;
        event_flag <<= 1;
        if ( !event_flag ) event_flag = 1;
    }


}

void scheduler_init( scheduler_t *scheduler_p ){

    scheduler_p->task[0].task = adc_task;
    scheduler_p->task[0].priority = medium_prior;
    scheduler_p->task[0].callback = &task_adc;
    scheduler_p->task[0].event_group = adc_task_group;

    scheduler_p->task[1].task = counter_task;
    scheduler_p->task[1].priority = low_prior;
    scheduler_p->task[1].callback = &task_counter;
	scheduler_p->task[1].event_group = counter_task_group;

    scheduler_p->task[2].task = console_task;
    scheduler_p->task[2].priority = high_prior;
    scheduler_p->task[2].callback = &task_console;
	scheduler_p->task[2].event_group = console_task_group;


}


void scheduler_run( scheduler_t *scheduler_p ){
    uint8_t i = 0;


    while ( scheduler_p->event ) {

        if ( scheduler_p->event & scheduler_p->task[i].event_group ) {

            scheduler_p->task[i].event = scheduler_p->event & scheduler_p->task[i].event_group;
            scheduler_p->task[i].callback(&scheduler_p->task[i]);
        }

        scheduler_p->event = scheduler_p->event & ( ~((uint32_t)scheduler_p->task[i].event_group) );

        if ( i >= TASK_NUMBER ) i = 0;
        else ++i;

    }

}

void event_post( event_t *global_event, uint32_t event_set ){
	uint32_t prim;

	prim = __get_PRIMASK();
	__disable_irq();

	*global_event = *global_event + event_set;

	if (!prim) {
		__enable_irq();
	}

}


uint32_t event_pend( event_t *global_event ){
	event_t return_event;
	uint32_t prim;

	prim = __get_PRIMASK();
	__disable_irq();

	return_event = *global_event;
	*global_event = 0;

	if (!prim) {
		__enable_irq();
	}

	return return_event;
}


