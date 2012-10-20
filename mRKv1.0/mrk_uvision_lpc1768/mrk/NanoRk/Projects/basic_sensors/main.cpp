#include "mbed.h"                    
#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>

void nrk_register_drivers();

uint8_t fd,val,chan;
uint8_t buf[2];
uint16_t adc_int;

NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

void nrk_create_taskset();

// You do not need to modify this function
struct __FILE { int handle; };

int main(void)

  {
			
    	nrk_setup_ports();
			nrk_register_drivers();
			nrk_init();
			nrk_create_taskset();
			nrk_start();
			
			return 0;

	}

	
	void Task1()

	{
		nrk_led_clr(ORANGE_LED);
    nrk_led_clr(BLUE_LED);
    nrk_led_clr(GREEN_LED);
    nrk_led_clr(RED_LED);

		while(1)
		{
				nrk_led_toggle(ORANGE_LED);
        fd=nrk_open(FIREFLY_SENSOR_BASIC,READ);
        wait_ms(1);
        val = nrk_set_status(fd, SENSOR_SELECT, TEMP);
        val = nrk_read(fd, &buf[0], 2);
        adc_int = buf[0] + (buf[1]<<8);
        printf(" temp: %d\r\n", adc_int);
       
        nrk_close(fd);
        
			nrk_wait_until_next_period();
	}
		
	}

		

void nrk_create_taskset()

{
	
    nrk_task_set_entry_function( &TaskOne, Task1);
    nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
    TaskOne.prio = 5;
    TaskOne.FirstActivation = TRUE;
    TaskOne.Type = BASIC_TASK;
    TaskOne.SchType = PREEMPTIVE;
    TaskOne.period.secs = 0;
    TaskOne.period.nano_secs = 500*NANOS_PER_MS;
    TaskOne.cpu_reserve.secs = 0;
    TaskOne.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
    TaskOne.offset.secs = 0;
    TaskOne.offset.nano_secs= 0;
    nrk_activate_task (&TaskOne);

}


void nrk_register_drivers()
{
    int8_t val;

    // Register the ADC device driver
     
    val=nrk_register_driver( &dev_manager_ff_sensors,FIREFLY_SENSOR_BASIC);
    if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n"));

}