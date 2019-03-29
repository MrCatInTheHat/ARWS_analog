/*
 * scheduler.c
 *
 *  Created on: 19 февр. 2019 г.
 *      Author: Konstantinov
 */

#include "scheduler.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "console.h"
#include "meteo.h"
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
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

extern USBD_HandleTypeDef hUsbDeviceFS;
extern console_t console;
extern event_t event;

volatile meteo_t meteo = { 0 };
volatile wgauge_t wind_gauge;
volatile wvane_t wind_vane;


inline static uint32_t round_div_u32( uint32_t x, uint32_t y )
{
    return ( ( x + (y >> 1) ) / y );
}



void task_adc ( struct task_t * task )
{

    uint32_t event_flag = 1;


    while ( task->event ) {

          switch( task->event & event_flag ){

          case time_to_poll_adc_ch1:

        	  temp_meas();
        	  event_post(&event, time_to_poll_adc_ch4);

              break;

          case time_to_poll_adc_ch4:

			  volt_meas();
			  event_post(&event, time_to_poll_adc_ch1);

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

	uint32_t event_flag = 1;


    while ( task->event ) {

          switch( task->event & event_flag ){

          case time_to_start_counter:

        	  HAL_TIM_Base_Start_IT(&htim2);
        	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//        	  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
        	  HAL_TIM_Base_Start_IT(&htim3);
//        	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//        	  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

              break;


          case timer_5_seconds:

        	//  if ( task->state == test_state )
        	 // HAL_TIM_Base_Start_IT(&htim3);

              break;


          case counter_ready:


        	  wind_gauge.total_sum_15 -= wind_gauge.samples[ wind_gauge.index_15 ];
			  wind_gauge.total_sum_15 += meteo.wind.counter;
			  wind_gauge.total_sum -= wind_gauge.samples[ wind_gauge.index ];
			  wind_gauge.total_sum += meteo.wind.counter;
			  wind_gauge.samples[ wind_gauge.index ] = meteo.wind.counter;

			  ++wind_gauge.index;
			  if ( wind_gauge.index >= LENGTH( wind_gauge.samples ) ) {
				  wind_gauge.index = 0;
			  }

			  wind_gauge.index_15 = wind_gauge.index - 15;
			  if ( (int8_t) wind_gauge.index_15 < 0 ) {
				  wind_gauge.index_15 += LENGTH( wind_gauge.samples );
			  }

			  // 2.
			  if ( wind_gauge.sample_number_15 < 15 ) {
				  ++wind_gauge.sample_number_15;
			  }

			  if ( wind_gauge.sample_number < LENGTH( wind_gauge.samples ) ) {
				  ++wind_gauge.sample_number;
			  }

			  meteo.wind.wind_speed = round_div_u16( meteo.wind.counter, 6 );
			  meteo.wind.wind_speed_15_min = round_div_u32( wind_gauge.total_sum_15, wind_gauge.sample_number_15 * 6 );
			  meteo.wind.wind_speed_30_min = round_div_u32( wind_gauge.total_sum, wind_gauge.sample_number * 6 );

			  uint16_t wind_gust_30_min = 0;
			  for ( uint8_t i = 0; i < LENGTH( wind_gauge.samples ); ++i ) {
				  if ( wind_gauge.samples[ i ] > wind_gust_30_min ) {
					  wind_gust_30_min = wind_gauge.samples[ i ];
				  }
			  }
			  meteo.wind.wind_gust_30_min = round_div_u16( wind_gust_30_min, 6 );

        	  event_post(&event, time_to_start_counter);
        	  meteo.wind.counter = 0;


        	  int16_t wind_direction = round_div_i16( wind_vane.minute_mean.total_sum, wind_vane.minute_mean.sample_number );
			  unwrap_angle( &wind_direction );
			  wind_vane.minute_mean.state = AMA_INIT;
			  wvane_accumulate_data( wind_direction );
			  wvane_average_data();


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

	uint32_t event_flag = 1;
	static cmd_type_t command_type;
	static uint8_t result = CR_ERROR;


    while ( task->event ) {

          switch( task->event & event_flag ){

          case console_read_event:

        	  console.input.state = CIS_IDLE;
        	  if ( CLI_OPENED == console.state ) {
        	  		  command_type = CT_ONLINE;
			  }  else command_type = CT_OFFLINE;

			  do {
			  // Get the next output string from the command interpreter.
				  result = console_process_command( &console, command_type );
			  } while ( CR_DONE != result );
			  console_buffer_clear();

              break;

          case console_out_event:

              break;

          case console_test_command:

        	  if ( task->state == test_state )
        	  command_test( console.state, console.input_buffer );

			  break;


          default:

              break;
          }
        task->event &= ~event_flag;
        event_flag <<= 1;
        if ( !event_flag ) event_flag = 1;
    }

    huart1.gState = HAL_UART_STATE_READY;


}

void scheduler_init( scheduler_t *scheduler_p ){

	scheduler_p->task[0].task = console_task;
	scheduler_p->task[0].priority = high_prior;
	scheduler_p->task[0].callback = &task_console;
	scheduler_p->task[0].event_group = console_task_group;
	scheduler_p->task[0].state = scheduler_p->state;

	scheduler_p->task[1].task = counter_task;
	scheduler_p->task[1].priority = low_prior;
	scheduler_p->task[1].callback = &task_counter;
	scheduler_p->task[1].event_group = counter_task_group;
	scheduler_p->task[1].state = scheduler_p->state;

	scheduler_p->task[2].task = adc_task;
	scheduler_p->task[2].priority = medium_prior;
	scheduler_p->task[2].callback = &task_adc;
	scheduler_p->task[2].event_group = adc_task_group;
	scheduler_p->task[2].state = scheduler_p->state;

	console_init();

	adc_init();

}


void scheduler_run( scheduler_t *scheduler_p ){
	uint32_t i = 0;

	if ( scheduler_p->event & timer_5_seconds ) scheduler_p->state = test_state;
	else if ( scheduler_p->event & console_read_event ) scheduler_p->state = active_state;


    while ( scheduler_p->event ) {

        if ( scheduler_p->event & scheduler_p->task[i].event_group ) {

        	scheduler_p->task[i].state = scheduler_p->state;
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


