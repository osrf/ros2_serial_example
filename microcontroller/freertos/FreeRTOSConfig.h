/*
    FreeRTOS V7.4.0 - Copyright (C) 2013 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.

    >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
    details. You should have received a copy of the GNU General Public License
    and the FreeRTOS license exception along with FreeRTOS; if not itcan be
    viewed here: http://www.freertos.org/a00114.html and also obtained by
    writing to Real Time Engineers Ltd., contact details for whom are available
    on the FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, and our new
    fully thread aware and reentrant UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems, who sell the code with commercial support,
    indemnification and middleware, under the OpenRTOS brand.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.
*/


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* Ensure stdint is only used by the compiler, and not the assembler. */
#ifdef __ICCARM__
	#include <stdint.h>
	extern uint32_t SystemCoreClock;
#endif

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			0
#define configCPU_CLOCK_HZ			( 64000000 )
#define configTICK_RATE_HZ			(( portTickType ) 1000)
#define configMAX_PRIORITIES			(16)
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 100 )
/* From empirical evidence, the smallest HEAP we can possibly have is around 1K for just the blinky task.
 * We obviously want more than this, so for now we kick it up to 10K, which is 25% of our memory.
 */
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 10 * 1024 ) )
#define configMAX_TASK_NAME_LEN			( 16 )
#define configUSE_TRACE_FACILITY		1
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD	                1
#define configUSE_MUTEXES			1
#define configQUEUE_REGISTRY_SIZE		0
#define configCHECK_FOR_STACK_OVERFLOW	        2
#define configUSE_RECURSIVE_MUTEXES		0
#define configUSE_MALLOC_FAILED_HOOK	        0
#define configUSE_APPLICATION_TASK_TAG	        0
#define configUSE_COUNTING_SEMAPHORES	        1
#define configGENERATE_RUN_TIME_STATS	        0
/* Disable the formatting functions.  We'll do it ourselves, and this will save
 * a ton of code space.
 */
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		        0
#define configMAX_CO_ROUTINE_PRIORITIES       ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS			0
#define configTIMER_TASK_PRIORITY		( 2 )
#define configTIMER_QUEUE_LENGTH		16
#define configTIMER_TASK_STACK_DEPTH	        (configMINIMAL_STACK_SIZE * 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		0
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete			0
#define INCLUDE_vTaskCleanUpResources	        0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1
#define INCLUDE_xTaskGetSchedulerState		1

/* Interrupt priorities used by the kernel port layer itself.  These
are generic to all Cortex-M ports, and do not rely on any particular
library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		255

/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero
!!!!  See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191

#define vPortSVCHandler sv_call_handler
#define xPortPendSVHandler pend_sv_handler
#define xPortSysTickHandler sys_tick_handler

/* CRL - everything below here was added by me */
/* portTICK_RATE_MS only works with tick rates <= 1000.  Since we are running
 * faster than that, undefine it so it doesn't get used and add macros that
 * properly convert from milliseconds to ticks
 */
#ifdef portTICK_RATE_MS
#undef portTICK_RATE_MS
#endif

/* we need to use this macro instead of portTICK_RATE_MS, since the latter only
 * works with tick rates < 1000
 */
#define MS_TO_TICKS(_ms)    ((portTickType)((_ms) * configTICK_RATE_HZ / 1000))
#define US_TO_TICKS(_us)    ((portTickType)((_us) * configTICK_RATE_HZ / 1000000))

#endif /* FREERTOS_CONFIG_H */
