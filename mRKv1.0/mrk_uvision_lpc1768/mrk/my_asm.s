

   
	EXPORT 	PendSV_Handler
    EXPORT  _swap
	EXPORT nrk_start_high_ready_task
	EXPORT Switch
    EXPORT _nrk_spin_wait_1us
	
    
	EXTERN  nrk_high_ready_TCB
	EXTERN  _nrk_timer_tick
	EXTERN  task_addr
	EXTERN	nrk_kernel_stk_ptr
	
ID_OFFSET           equ     0               ; 0 word (0byte) offset
STACK_OFFSET        equ     4               ; 1 word (4byte) offset
PRIO_OFFSET         equ     8               ; 2 word (8byte) offset
LIMIT_OFFSET        equ     20              ; 5 word (20byte) offset
NVIC_SYSPRI14		equ		0xE000ED22
NVIC_PENDSV_PRI		equ		0xFF
NVIC_INT_CTRL		equ		0xE000ED04
NVIC_PENDSVSET		equ		0x10000000

    AREA    |.text|, CODE, READONLY, ALIGN=2
	THUMB
    PRESERVE8

_nrk_spin_wait_1us
    NOP
    NOP
    MOV R0,#29
loop   
    SUBS R0,R0,#1
    BNE loop
    BX LR

nrk_start_high_ready_task
	
	LDR R0, =NVIC_SYSPRI14 ; (1) Set the PendSV exception priority
	LDR R1, =NVIC_PENDSV_PRI
	STRB R1, [R0]
	MOV R0, #0 ; (2) Set PSP to 0 for initial context switch call
	MSR PSP, R0
	LDR R0, =NVIC_INT_CTRL ; (4) Trigger the PendSV exception
	LDR R1, =NVIC_PENDSVSET
	STR R1, [R0]
	CPSIE I ; (5) Enable interrupts at processor level
    
	
; Function to initiate a SetPend Exception to swap tasks ---------------------------------
_swap
    LDR R0, =NVIC_INT_CTRL ; trigger the PendSV exception
	LDR R1, =NVIC_PENDSVSET
	STR R1, [R0]
	BX LR

; Main task switching and scheduling functions -------------------------------------------
PendSV_Handler                              ; entered here as handler for context switch
    CPSID I ; Prevent interruption during context switch
	MRS R0, PSP ; (1) PSP is process stack pointer
	CBZ R0, OSPendSV_nosave ; Skip register save the first time
	SUB R0, R0, #0x20 ; (2) Save remaining regs r4-11 on process stack
	STM R0, {R4-R11}
	LDR R1, =nrk_high_ready_TCB ; (3) OSTCBCur->OSTCBStkPtr = SP;
	LDR R1, [R1]
	STR R0, [R1] ; R0 is SP of process being switched out
	
OSPendSV_nosave

	bl _nrk_timer_tick

Switch
	mov 		lr,#0xFFFFFFF9
	;LDR R0, __OS_PrioCur ; (5) OSPrioCur = OSPrioHighRdy
	;LDR R1, __OS_PrioHighRdy
	;LDRB R2, [R1]
	;STRB R2, [R0]
	;LDR R0, __OS_TCBCur ; (6) OSTCBCur = OSTCBHighRdy;
	LDR R1, =nrk_high_ready_TCB
	LDR R2, [R1]
	;STR R2, [R0]
	LDR R0, [R2] ; (7) R0 is new task SP
; SP = OSTCBHighRdy->OSTCBStkPtr;
	LDM R0, {R4-R11} ; (8) Restore R4-R11 from new task stack
	ADD R0, R0, #0x20
	MSR PSP, R0 ; Load PSP with new task SP
	ORR LR, LR, #0x04 ; Ensure exception return uses process stack
	CPSIE I
	BX LR ; (9) Exception return
	
	
    END