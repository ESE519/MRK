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
 *  Zane Starr
 *  Anthony Rowe
 *******************************************************************************/


#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <adc_driver.h>
#include <include.h>
#include <stdio.h>
#include <ulib.h>
#include <nrk_error.h>
#include <nrk.h>
#include <stdint.h>
#include <nrk_timer.h>

#define ADC_SETUP_DELAY  500

// Dalton ADC

uint8_t channel;
volatile uint32_t *adc_reg;

void set_adc_chan(uint8_t channel);

uint8_t dev_manager_adc(uint8_t action,uint8_t opt,uint8_t *buffer,uint8_t size)
{
    uint8_t count=0;
    // key and value get passed as opt and size
    uint8_t key=opt;
    uint8_t value=size;
    uint16_t val;

    switch(action)
    {
        case INIT: 
            init_adc();  
            return 1;

        case OPEN:   
            if(opt&READ_FLAG)
            {
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
            if(opt&(READ_FLAG|WRITE_FLAG|APPEND_FLAG)==0)
                return NRK_ERROR;
            else return NRK_OK;



        case READ:
            // Conversion to 8-bit value
            val=get_adc_val();
            buffer[count]=val & 0xFF;
            count++;
            buffer[count]=(val>>8) & 0xFF;
            count++;
            return count;

        case CLOSE:
            return NRK_OK;

        case GET_STATUS:
            // use "key" here 
            if(key==ADC_CHAN) return channel;
            return NRK_ERROR;

        case SET_STATUS:
            // use "key" and "value" here
            if(key==ADC_CHAN) {
                channel=value;
                set_adc_chan(0);
                return NRK_OK;
            }
            return NRK_ERROR;
        default:
            nrk_kernel_error_add(NRK_DEVICE_DRIVER,0);
            return 0;
    }
}

void init_adc()
{
    // set up pins
    // p15 (P0.23)
    LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 14);
    LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 14;
    LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 14);
    LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 14;
    // p16 (P0.24)
    LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 16);
    LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 16;
    LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 16);
    LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 16;
    // p17 (P0.25)
    LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 18);
    LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 18;
    LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 18);
    LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 18;
    // p18 (P0.26)
    LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 20);
    LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 20;
    LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 20);
    LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 20;
    // p19 (P1.30)
    LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 28);
    LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 28;
    LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 28);
    LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 28;
    // p20 (P1.31)
    LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 30);
    LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 30;
    LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 30);
    LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 30;
    // power up ADC       
    LPC_SC->PCONP |= (1UL<<12);
    // enable ADC (PDN)
    LPC_ADC->ADCR |= (1UL<<21);
    // set clock at cclk / 8 (12 MHz out of max 13 MHz)
    LPC_SC->PCLKSEL0 |= (3UL<<24);
    // don't start
    LPC_ADC->ADCR &= ~(7UL<<24);
    nrk_spin_wait_us(10);
}

void set_adc_chan(uint8_t channel) {
    LPC_ADC->ADCR &= ~(0xFF); // clear channel select
    LPC_ADC->ADCR |= (1UL<<channel); // set channel select
    switch(channel) {
        case 0:
            adc_reg=&(LPC_ADC->ADDR0);
            break;
        case 1:
            adc_reg=&(LPC_ADC->ADDR1);
            break;
        case 2:
            adc_reg=&(LPC_ADC->ADDR2);
            break;
        case 3:
            adc_reg=&(LPC_ADC->ADDR3);
            break;
        case 4:
            adc_reg=&(LPC_ADC->ADDR4);
            break;
        case 5:
            adc_reg=&(LPC_ADC->ADDR5);
            break;
    }
}

uint16_t get_adc_val()
{                         
    // enable ADC (PDN)
    LPC_ADC->ADCR |= (1UL<<21);
    LPC_ADC->ADCR |= (1UL<<24);
    // wait for conversion
    while(!(*adc_reg & (1UL<<31)));
    // don't start
    LPC_ADC->ADCR &= ~(7UL<<24);
    return (0xFFF & (*adc_reg >> 4));
}
void delay()
{
    //nrk_spin_wait_us(ADC_SETUP_DELAY);
}

