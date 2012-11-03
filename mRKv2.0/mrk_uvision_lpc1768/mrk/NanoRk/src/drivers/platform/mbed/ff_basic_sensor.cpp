/******************************************************************************
 *  Nano-RK, a real-time operating system for sensor networks.
 *  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This is the Open Source Version of Nano-RK included as part of a Dual
 *  Licensing Model. If you are unsure which license to use please refer to:
 *  http://www.nanork.org/nano-RK/wiki/Licensing
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2.0 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Contributing Authors (specific to this file):
 *  Anthony Rowe
 *  Zane Starr
 *******************************************************************************/


#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
#include <stdint.h>
//#include <basic_rf.h>
#include <nrk_timer.h>
#include <adc_driver.h>

#define ADC_STARTUP_DELAY  1000
#define ADC_SETUP_DELAY  200

uint8_t channel2;

uint8_t is_open;

uint16_t read_voltage_status();
uint8_t dev_manager_ff_sensors(uint8_t action,uint8_t opt,uint8_t *buffer,uint8_t size)
{
    uint8_t count=0;
    // key and value get passed as opt and size
    uint8_t key=opt;
    uint8_t value=size;

    switch(action)
    {
        case INIT: 
            // Set the pwr ctrl pin as output
            LPC_SC->PCONP |= (1<<15); // power up GPIO
            LPC_GPIO0->FIODIR |= (1<<6); // puts P0.6 into output mode (p8)
            LPC_GPIO0->FIOSET |= (1<<6);
            init_adc();  
            is_open=0;
            return 1;

        case OPEN:  
            if(is_open==1) return NRK_ERROR;
            is_open=1; 
            if(opt&READ_FLAG)
            {
                // Turn on Sensor Node Power
                LPC_GPIO0->FIOCLR |= (1<<6);
                channel2=0;
                set_adc_chan(0);
                //nrk_spin_wait_us(ADC_STARTUP_DELAY);
                nrk_spin_wait_us(250);
                return NRK_OK;
            }
            if(opt&WRITE_FLAG)
            {
                return NRK_ERROR;
            }
            if(opt&APPEND_FLAG)
            {
                return NRK_ERROR;
            }
            if(((opt)&(READ_FLAG|WRITE_FLAG|APPEND_FLAG))==0)
                return NRK_ERROR;
            else return NRK_OK;

        case READ:
            count=0;
            if(size!=1 && size!=2) return 0;
            if(channel2!=BAT && channel2<7) {
                /* Conversion to 8-bit value*/
                uint16_t val=get_adc_val();

                if(size==2)
                {
                    buffer[count]=val  & 0xFF;
                    count++;
                    buffer[count]=(val>>8)  & 0xFF;
                }

                if(size==1)
                {
                    buffer[count]=(val>>2)  & 0xFF;
                }


            } else if(channel2==BAT) {
                /* TO IMPLEMENT
                uint16_t tmp;
                tmp=read_voltage_status();
                if(size==2)
                {
                    buffer[count]=tmp & 0xFF;
                    count++;
                    buffer[count]=(tmp>>8) & 0xFF;
                }
                if(size==1)
                {
                    buffer[count]=(tmp>>2) & 0xFF;
                }
                */
            } else if(channel2==AUDIO_P2P) {
                /* Conversion to 8-bit value*/
                //uint16_t val=get_adc_val();
                uint16_t val,min, max;
                uint8_t i;
                max=0;
                min=4095;
                for(i=0; i<128; i++ )
                {
                    val=get_adc_val();
                    if(val<min)min=val;
                    if(val>max)max=val;
                    // 8 Khz
                    //nrk_spin_wait_us(125);	
                    nrk_spin_wait_us(125);
                }
                val=max-min;	
                if(size==2)
                {
                    buffer[count]=val  & 0xFF;
                    count++;
                    buffer[count]=(val>>8)  & 0xFF;
                }

                if(size==1)
                {
                    buffer[count]=(val>>2)  & 0xFF;
                }

            }

            count++;
            return count;

        case CLOSE:
            // Turn off sensor power
            LPC_GPIO0->FIOSET |= (1<<6);
            is_open=0; 
            return NRK_OK;

        case GET_STATUS:
            // use "key" here 
            if(key==SENSOR_SELECT) return channel2;
            return NRK_ERROR;

        case SET_STATUS:
            // use "key" and "value" here
            if(key==SENSOR_SELECT) 
            {
                // Set to audio channel if it is an average value
                if(value==AUDIO_P2P) 
                {
                    channel2=value;
                    //ADC_VREF_2_56();	
                    //ADC_VREF_VCC();	
                    //ADC_SET_CHANNEL (AUDIO);
                    set_adc_chan(AUDIO-1);
                    //nrk_spin_wait_us(ADC_SETUP_DELAY);
                    nrk_spin_wait_us(250);
                    return NRK_OK;

                } else
                {
                    if(value>7) 
                    {
                        _nrk_errno_set(1);
                        return NRK_ERROR;
                    }
                    channel2=value;
                    /*
                    if(channel2==LIGHT)
                        ADC_VREF_VCC();	
                    else
                        ADC_VREF_2_56();	
                    */
                    //ADC_SET_CHANNEL (AUDIO);
                    set_adc_chan(channel2 - 1);
                    //nrk_spin_wait_us(ADC_SETUP_DELAY);
                    nrk_spin_wait_us(250);
                    return NRK_OK;
                }
            }
            return NRK_ERROR;
        default:
            nrk_kernel_error_add(NRK_DEVICE_DRIVER,0);
            return 0;
    }
}


// read_voltage_status()
//
// This function sets different voltage threshold levels on
// the cc2420 chip to search for the voltage.
// If the voltage is above 3.3 volts, then the ADC reads
// the external voltage value going through a voltage divider.
// This function will return VOLTS*100

/*
uint16_t read_voltage_status()
{
    volatile uint16_t val;
    uint8_t check,level;
    nrk_sem_t *radio_sem;

    radio_sem= rf_get_sem();

    // if semaphore not created, then assume you own the radio 
    if(radio_sem!=NULL)
        nrk_sem_pend (radio_sem);

    // activate cc2420 vreg
    SET_VREG_ACTIVE();
    // FIXME: Check at end if VREG needs to be disabled again...

    level=0;
    while(level<0x1F)
    {
        val=0x0020 | level;
        FASTSPI_SETREG(CC2420_BATTMON, val);
        nrk_spin_wait_us(2);
        FASTSPI_GETREG(CC2420_BATTMON, val);
        if(val&0x0040) break; 
        level++;
    }
    if(radio_sem!=NULL)
        nrk_sem_post(radio_sem);
    if(level==0)
    {
        val=get_adc_val();
        // FIXME:  This probably isn't correct...
        if(val>174) val-=174;
        if(val<330) val=330;
    }
    else val=(9000-(level*125)) / 27;

    return val;
}
*/