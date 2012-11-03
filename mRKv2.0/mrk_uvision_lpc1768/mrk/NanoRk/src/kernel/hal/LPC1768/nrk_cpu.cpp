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
*  Anand Eswaren
*******************************************************************************/

#include <include.h>
#include <nrk.h>
/* Dalton Banks */
// #include <avr/sleep.h>
#include <nrk_stack_check.h>
#include <nrk_task.h>
#include <nrk_cfg.h>
#include <nrk_timer.h>
#include <nrk_error.h>

#define BUILD_DATE "Date: " __DATE__ "\n"
/* Constants required to set up the initial stack. */
#define INITIAL_XPSR             0x01000000 
//Serial pc7(USBTX, USBRX); // tx, rx

/*
*********************************************************************************************************
*                                        INITIALIZE A TASK'S STACK
*
* Description: This function is highly processor specific.
*
* Arguments  : task          is a pointer to the task code
*
*              pdata         is a pointer to a user supplied data area that will be passed to the task
*                            when the task first executes.
*
*              ptos          is a pointer to the top of stack.  It is assumed that 'ptos' points to
*                            a 'free' entry on the task stack.  
*                            'ptos' contains the HIGHEST valid address of the stack.  
*
*              opt           specifies options that can be used to alter the behavior of OSTaskStkInit().
*                            We don't use have any option implemented for this project. You can just
*                            set opt to 0
*
* Returns    : Always returns the location of the new top-of-stack' once the processor registers have
*              been placed on the stack in the proper order.
*
* Note(s)    : 
*********************************************************************************************************
*/


void nrk_battery_save()
{
    /*
#ifdef NRK_BATTERY_SAVE
     _nrk_stop_os_timer();
        _nrk_set_next_wakeup(250);
        nrk_led_clr(0);
        nrk_led_set(1);
        nrk_led_clr(2);
        nrk_led_clr(3);
        SET_VREG_INACTIVE();
        nrk_sleep();
#endif
     */
}

void nrk_sleep()
{
    /*
    set_sleep_mode (SLEEP_MODE_PWR_SAVE);
    sleep_mode ();
     */
		__WFI();
}

void nrk_idle()
{
    /*
    set_sleep_mode( SLEEP_MODE_IDLE);
    sleep_mode ();
     */
		__WFI();
}

void nrk_task_set_entry_function( nrk_task_type *task, void (*func) (void) )
{
    task->task=func; //function pointer...
    
}

void nrk_task_set_stk( nrk_task_type *task, NRK_STK stk_base[], uint16_t stk_size )
{
    if(stk_size<32) nrk_error_add(NRK_STACK_TOO_SMALL);
    /* Dalton Banks - changed cast from void* to NRK_STK* */
    task->Ptos = (NRK_STK *) &stk_base[stk_size-1];
    /* Dalton Banks - changed cast from void* to NRK_STK* */
    task->Pbos = (NRK_STK *) &stk_base[0];
    

}

/* Dalton Banks - changed all stack pointers to NRK_STK types */
void *nrk_task_stk_init (void (*task)(), NRK_STK *ptos, NRK_STK *pbos)
{    
    NRK_STK *stk ;  // 4 bytes
    NRK_STK *stkc;  // 2 byte

    /* Dalton Banks - changed cast from unsigned char* to NRK_STK* */
    stk    = pbos;       /* Load stack pointer */ 
    /* Dalton Banks - changed cast from unsigned char* to NRK_STK* */
    stkc = stk; 
    *stkc = STK_CANARY_VAL;                 // Flag for Stack Overflow  
    stk    = ptos;          /* Load stack pointer */
    /* Simulate the stack */
	  //stk -= 3; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
	  
	 
	  *(stk) = 0x01000000; /* xPSR */
		*(--stk) = ( unsigned int ) task ; /* Entry Point */
		*(--stk) = 0x14141414; /* R14 (LR) */
		*(--stk) = 0x12121212; /* R12 */
		*(--stk) = 0x03030303; /* R3 */
		*(--stk) = 0x02020202; /* R2 */
		*(--stk) = 0x01010101; /* R1 */
		*(--stk) = 0x00000000; /* R0 : argument */
		/* Remaining registers saved on process stack */
		*(--stk) = 0x11111111; /* R11 */
		*(--stk) = 0x10101010; /* R10 */
		*(--stk) = 0x09090909; /* R9 */
		*(--stk) = 0x08080808; /* R8 */
		*(--stk) = 0x07070707; /* R7 */
		*(--stk) = 0x06060606; /* R6 */
		*(--stk) = 0x05050505; /* R5 */
		*(--stk) = 0x04040404; /* R4 */
		
    return stk;
}

/* Dalton Banks - changed all stack pointers to NRK_STK types */
void nrk_stack_pointer_init()
{
    
    NRK_STK *stkc;
    
    #ifdef KERNEL_STK_ARRAY
            stkc = &nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
            
            nrk_kernel_stk[0]=STK_CANARY_VAL;
            nrk_kernel_stk_ptr = &nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
          
        #else
            stkc = (NRK_STK *)(NRK_KERNEL_STK_TOP-NRK_KERNEL_STACKSIZE);
            *stkc = STK_CANARY_VAL;
            stkc = (NRK_STK *)NRK_KERNEL_STK_TOP;
            nrk_kernel_stk_ptr = (NRK_STK *)NRK_KERNEL_STK_TOP;
        #endif
        //figure out why this is done... Abhijeet
        //*stkc++ = (uint16_t)((uint16_t)_nrk_timer_tick>>8);
        //*stkc = (uint16_t)((uint16_t)_nrk_timer_tick&0xFF);
        //*stkc = (NRK_STK)_nrk_timer_tick;
				//pc7.printf("kernel_task:%x \r\n",_nrk_timer_tick);
				//stkc -= 3; 
				//*stkc = INITIAL_XPSR;	 
				//stkc--;
				//*stkc = ( unsigned int ) _nrk_timer_tick -1;	
				//stkc--;
				*(stkc) = 0x01000000; /* xPSR */
				*(--stkc) = ( unsigned int ) _nrk_timer_tick ; /* Entry Point */
				*(--stkc) = 0x14141414; /* R14 (LR) */
				*(--stkc) = 0x12121212; /* R12 */
				*(--stkc) = 0x03030303; /* R3 */
				*(--stkc) = 0x02020202; /* R2 */
				*(--stkc) = 0x01010101; /* R1 */
				*(--stkc) = 0x00000000; /* R0 : argument */
				/* Remaining registers saved on process stack */
				*(--stkc) = 0x11111111; /* R11 */
				*(--stkc) = 0x10101010; /* R10 */
				*(--stkc) = 0x09090909; /* R9 */
				*(--stkc) = 0x08080808; /* R8 */
				*(--stkc) = 0x07070707; /* R7 */
				*(--stkc) = 0x06060606; /* R6 */
				*(--stkc) = 0x05050505; /* R5 */
				*(--stkc) = 0x04040404; /* R4 */
		
				nrk_kernel_stk_ptr = stkc;
				//pc7.printf("kernel_stk_top:%x \r\n",nrk_kernel_stk_ptr);
}


void nrk_stack_pointer_restore()
{
    NRK_STK *stkc;

    #ifdef KERNEL_STK_ARRAY
            /* Dalton Banks - changed (uint16_t*) cast to (unsigned char*) */
            stkc = &nrk_kernel_stk[NRK_KERNEL_STACKSIZE-1];
    #else
            stkc = (NRK_STK *)NRK_KERNEL_STK_TOP;
    #endif
            //figure out why this is done... Abhijeet
            //*stkc++ = (uint16_t)((uint16_t)_nrk_timer_tick>>8);
            //*stkc = (uint16_t)((uint16_t)_nrk_timer_tick&0xFF);
            //*stkc = (uint32_t)_nrk_timer_tick;
				*(stkc) = 0x01000000; /* xPSR */
				*(--stkc) = ( unsigned int ) _nrk_timer_tick ; /* Entry Point */
				*(--stkc) = 0x14141414; /* R14 (LR) */
				*(--stkc) = 0x12121212; /* R12 */
				*(--stkc) = 0x03030303; /* R3 */
				*(--stkc) = 0x02020202; /* R2 */
				*(--stkc) = 0x01010101; /* R1 */
				*(--stkc) = 0x00000000; /* R0 : argument */
				/* Remaining registers saved on process stack */
				*(--stkc) = 0x11111111; /* R11 */
				*(--stkc) = 0x10101010; /* R10 */
				*(--stkc) = 0x09090909; /* R9 */
				*(--stkc) = 0x08080808; /* R8 */
				*(--stkc) = 0x07070707; /* R7 */
				*(--stkc) = 0x06060606; /* R6 */
				*(--stkc) = 0x05050505; /* R5 */
				*(--stkc) = 0x04040404; /* R4 */
				nrk_kernel_stk_ptr = stkc;
				//pc7.printf("kernel_stk_top:%x \r\n",nrk_kernel_stk_ptr);
}

/* start the target running */
void nrk_target_start(void)
{

      _nrk_setup_timer();
      //nrk_int_enable();  
      __enable_irq(); //enable interrupts
}

