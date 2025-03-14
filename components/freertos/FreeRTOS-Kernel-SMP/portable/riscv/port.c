/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"
#include <string.h>
#include "soc/soc_caps.h"
#include "soc/periph_defs.h"
#include "soc/system_reg.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"
#include "riscv/rvruntime-frames.h"
#include "riscv/rv_utils.h"
#include "riscv/interrupt.h"
#include "esp_private/crosscore_int.h"
#include "esp_private/esp_int_wdt.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/systimer.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_heap_caps_init.h"
#include "esp_task_wdt.h"
#include "esp_task.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "FreeRTOS.h"       /* This pulls in portmacro.h */
#include "task.h"
#include "portmacro.h"
#include "esp_memory_utils.h"
#ifdef CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER
#include "soc/periph_defs.h"
#include "soc/system_reg.h"
#include "hal/systimer_hal.h"
#include "hal/systimer_ll.h"
#endif
#ifdef CONFIG_PM_TRACE
#include "esp_private/pm_trace.h"
#endif //CONFIG_PM_TRACE

_Static_assert(portBYTE_ALIGNMENT == 16, "portBYTE_ALIGNMENT must be set to 16");

/* ---------------------------------------------------- Variables ------------------------------------------------------
 *
 * ------------------------------------------------------------------------------------------------------------------ */

BaseType_t uxSchedulerRunning = 0;  // Duplicate of xSchedulerRunning, accessible to port files
volatile UBaseType_t uxInterruptNesting = 0;
portMUX_TYPE port_xTaskLock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE port_xISRLock = portMUX_INITIALIZER_UNLOCKED;
volatile BaseType_t xPortSwitchFlag = 0;
__attribute__((aligned(16))) static StackType_t xIsrStack[configISR_STACK_SIZE];
StackType_t *xIsrStackTop = &xIsrStack[0] + (configISR_STACK_SIZE & (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK)));

// Variables used for IDF style critical sections. These are orthogonal to FreeRTOS critical sections
static UBaseType_t port_uxCriticalNestingIDF = 0;
static UBaseType_t port_uxCriticalOldInterruptStateIDF = 0;

/* ------------------------------------------------ IDF Compatibility --------------------------------------------------
 * - These need to be defined for IDF to compile
 * ------------------------------------------------------------------------------------------------------------------ */

// ------------------ Critical Sections --------------------

void vPortEnterCritical(void)
{
    // Save current interrupt threshold and disable interrupts
    UBaseType_t old_thresh = ulPortSetInterruptMask();
    // Update the IDF critical nesting count
    port_uxCriticalNestingIDF++;
    if (port_uxCriticalNestingIDF == 1) {
        // Save a copy of the old interrupt threshold
        port_uxCriticalOldInterruptStateIDF = (UBaseType_t) old_thresh;
    }
}

void vPortExitCritical(void)
{
    if (port_uxCriticalNestingIDF > 0) {
        port_uxCriticalNestingIDF--;
        if (port_uxCriticalNestingIDF == 0) {
            // Restore the saved interrupt threshold
            vPortClearInterruptMask((int)port_uxCriticalOldInterruptStateIDF);
        }
    }
}

// ----------------------- System --------------------------

#define STACK_WATCH_AREA_SIZE 32
#define STACK_WATCH_POINT_NUMBER (SOC_CPU_WATCHPOINTS_NUM - 1)

void vPortSetStackWatchpoint(void *pxStackStart)
{
    uint32_t addr = (uint32_t)pxStackStart;
    addr = (addr + (STACK_WATCH_AREA_SIZE - 1)) & (~(STACK_WATCH_AREA_SIZE - 1));
    esp_cpu_set_watchpoint(STACK_WATCH_POINT_NUMBER, (char *)addr, STACK_WATCH_AREA_SIZE, ESP_CPU_WATCHPOINT_STORE);
}

// ---------------------- Tick Timer -----------------------

BaseType_t xPortSysTickHandler(void);

#ifdef CONFIG_FREERTOS_SYSTICK_USES_CCOUNT

#ifdef CONFIG_FREERTOS_CORETIMER_0
    #define SYSTICK_INTR_ID (ETS_INTERNAL_TIMER0_INTR_SOURCE+ETS_INTERNAL_INTR_SOURCE_OFF)
#endif
#ifdef CONFIG_FREERTOS_CORETIMER_1
    #define SYSTICK_INTR_ID (ETS_INTERNAL_TIMER1_INTR_SOURCE+ETS_INTERNAL_INTR_SOURCE_OFF)
#endif

#elif CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER

_Static_assert(SOC_CPU_CORES_NUM <= SOC_SYSTIMER_ALARM_NUM - 1, "the number of cores must match the number of core alarms in SYSTIMER");

void SysTickIsrHandler(void *arg);

static uint32_t s_handled_systicks[portNUM_PROCESSORS] = { 0 };

#define SYSTICK_INTR_ID (ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE)

/**
 * @brief Set up the systimer peripheral to generate the tick interrupt
 *
 * Both timer alarms are configured in periodic mode.
 * It is done at the same time so SysTicks for both CPUs occur at the same time or very close.
 * Shifts a time of triggering interrupts for core 0 and core 1.
 */
void vPortSetupTimer(void)
{
    unsigned cpuid = xPortGetCoreID();
#ifdef CONFIG_FREERTOS_CORETIMER_SYSTIMER_LVL3
    const unsigned level = ESP_INTR_FLAG_LEVEL3;
#else
    const unsigned level = ESP_INTR_FLAG_LEVEL1;
#endif
    /* Systimer HAL layer object */
    static systimer_hal_context_t systimer_hal;
    /* set system timer interrupt vector */
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE + cpuid, ESP_INTR_FLAG_IRAM | level, SysTickIsrHandler, &systimer_hal, NULL));

    if (cpuid == 0) {
        periph_module_enable(PERIPH_SYSTIMER_MODULE);
        systimer_hal_init(&systimer_hal);
        systimer_hal_tick_rate_ops_t ops = {
            .ticks_to_us = systimer_ticks_to_us,
            .us_to_ticks = systimer_us_to_ticks,
        };
        systimer_hal_set_tick_rate_ops(&systimer_hal, &ops);
        systimer_ll_set_counter_value(systimer_hal.dev, SYSTIMER_LL_COUNTER_OS_TICK, 0);
        systimer_ll_apply_counter_value(systimer_hal.dev, SYSTIMER_LL_COUNTER_OS_TICK);

        for (cpuid = 0; cpuid < SOC_CPU_CORES_NUM; cpuid++) {
            // Set stall option and alarm mode to default state. Below they will be set to a required state.
            systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_LL_COUNTER_OS_TICK, cpuid, false);
            uint32_t alarm_id = SYSTIMER_LL_ALARM_OS_TICK_CORE0 + cpuid;
            systimer_hal_select_alarm_mode(&systimer_hal, alarm_id, SYSTIMER_ALARM_MODE_ONESHOT);
        }

        for (cpuid = 0; cpuid < portNUM_PROCESSORS; ++cpuid) {
            uint32_t alarm_id = SYSTIMER_LL_ALARM_OS_TICK_CORE0 + cpuid;

            /* configure the timer */
            systimer_hal_connect_alarm_counter(&systimer_hal, alarm_id, SYSTIMER_LL_COUNTER_OS_TICK);
            systimer_hal_set_alarm_period(&systimer_hal, alarm_id, 1000000UL / CONFIG_FREERTOS_HZ);
            systimer_hal_select_alarm_mode(&systimer_hal, alarm_id, SYSTIMER_ALARM_MODE_PERIOD);
            systimer_hal_counter_can_stall_by_cpu(&systimer_hal, SYSTIMER_LL_COUNTER_OS_TICK, cpuid, true);
            if (cpuid == 0) {
                systimer_hal_enable_alarm_int(&systimer_hal, alarm_id);
                systimer_hal_enable_counter(&systimer_hal, SYSTIMER_LL_COUNTER_OS_TICK);
#ifndef CONFIG_FREERTOS_UNICORE
                // SysTick of core 0 and core 1 are shifted by half of period
                systimer_hal_counter_value_advance(&systimer_hal, SYSTIMER_LL_COUNTER_OS_TICK, 1000000UL / CONFIG_FREERTOS_HZ / 2);
#endif
            }
        }
    } else {
        uint32_t alarm_id = SYSTIMER_LL_ALARM_OS_TICK_CORE0 + cpuid;
        systimer_hal_enable_alarm_int(&systimer_hal, alarm_id);
    }
}

/**
 * @brief Systimer interrupt handler.
 *
 * The Systimer interrupt for SysTick works in periodic mode no need to calc the next alarm.
 * If a timer interrupt is ever serviced more than one tick late, it is necessary to process multiple ticks.
 */
IRAM_ATTR void SysTickIsrHandler(void *arg)
{
    uint32_t cpuid = xPortGetCoreID();
    systimer_hal_context_t *systimer_hal = (systimer_hal_context_t *)arg;
#ifdef CONFIG_PM_TRACE
    ESP_PM_TRACE_ENTER(TICK, cpuid);
#endif

    uint32_t alarm_id = SYSTIMER_LL_ALARM_OS_TICK_CORE0 + cpuid;
    do {
        systimer_ll_clear_alarm_int(systimer_hal->dev, alarm_id);

        uint32_t diff = systimer_hal_get_counter_value(systimer_hal, SYSTIMER_LL_COUNTER_OS_TICK) / systimer_ll_get_alarm_period(systimer_hal->dev, alarm_id) - s_handled_systicks[cpuid];
        if (diff > 0) {
            if (s_handled_systicks[cpuid] == 0) {
                s_handled_systicks[cpuid] = diff;
                diff = 1;
            } else {
                s_handled_systicks[cpuid] += diff;
            }

            do {
                xPortSysTickHandler();
            } while (--diff);
        }
    } while (systimer_ll_is_alarm_int_fired(systimer_hal->dev, alarm_id));

#ifdef CONFIG_PM_TRACE
    ESP_PM_TRACE_EXIT(TICK, cpuid);
#endif
}

#endif // CONFIG_FREERTOS_SYSTICK_USES_SYSTIMER



/* ---------------------------------------------- Port Implementations -------------------------------------------------
 * Implementations of Porting Interface functions
 * ------------------------------------------------------------------------------------------------------------------ */

// --------------------- Interrupts ------------------------

UBaseType_t ulPortSetInterruptMask(void)
{
    int ret;
    unsigned old_mstatus = RV_CLEAR_CSR(mstatus, MSTATUS_MIE);
    ret = REG_READ(INTERRUPT_CORE0_CPU_INT_THRESH_REG);
    REG_WRITE(INTERRUPT_CORE0_CPU_INT_THRESH_REG, RVHAL_EXCM_LEVEL);
    RV_SET_CSR(mstatus, old_mstatus & MSTATUS_MIE);
    /**
     * In theory, this function should not return immediately as there is a
     * delay between the moment we mask the interrupt threshold register and
     * the moment a potential lower-priority interrupt is triggered (as said
     * above), it should have a delay of 2 machine cycles/instructions.
     *
     * However, in practice, this function has an epilogue of one instruction,
     * thus the instruction masking the interrupt threshold register is
     * followed by two instructions: `ret` and `csrrs` (RV_SET_CSR).
     * That's why we don't need any additional nop instructions here.
     */
    return ret;
}

void vPortClearInterruptMask(UBaseType_t mask)
{
    REG_WRITE(INTERRUPT_CORE0_CPU_INT_THRESH_REG, mask);
    /**
     * The delay between the moment we unmask the interrupt threshold register
     * and the moment the potential requested interrupt is triggered is not
     * null: up to three machine cycles/instructions can be executed.
     *
     * When compilation size optimization is enabled, this function and its
     * callers returning void will have NO epilogue, thus the instruction
     * following these calls will be executed.
     *
     * If the requested interrupt is a context switch to a higher priority
     * task then the one currently running, we MUST NOT execute any instruction
     * before the interrupt effectively happens.
     * In order to prevent this, force this routine to have a 3-instruction
     * delay before exiting.
     */
    asm volatile ( "nop" );
    asm volatile ( "nop" );
    asm volatile ( "nop" );
}

BaseType_t xPortCheckIfInISR(void)
{
    return uxInterruptNesting;
}

void vPortAssertIfInISR(void)
{
    /* Assert if the interrupt nesting count is > 0 */
    configASSERT(xPortCheckIfInISR() == 0);
}

// ------------------ Critical Sections --------------------

void IRAM_ATTR vPortTakeLock( portMUX_TYPE *lock )
{
    spinlock_acquire( lock, portMUX_NO_TIMEOUT);
}

void IRAM_ATTR vPortReleaseLock( portMUX_TYPE *lock )
{
    spinlock_release( lock );
}

// ---------------------- Yielding -------------------------

void vPortYield(void)
{
    if (uxInterruptNesting) {
        vPortYieldFromISR();
    } else {

        esp_crosscore_int_send_yield(0);
        /* There are 3-4 instructions of latency between triggering the software
           interrupt and the CPU interrupt happening. Make sure it happened before
           we return, otherwise vTaskDelay() may return and execute 1-2
           instructions before the delay actually happens.

           (We could use the WFI instruction here, but there is a chance that
           the interrupt will happen while evaluating the other two conditions
           for an instant yield, and if that happens then the WFI would be
           waiting for the next interrupt to occur...)
        */
        while (uxSchedulerRunning && REG_READ(SYSTEM_CPU_INTR_FROM_CPU_0_REG) != 0) {}
    }
}

void vPortYieldFromISR( void )
{
    //traceISR_EXIT_TO_SCHEDULER();
    uxSchedulerRunning = 1;
    xPortSwitchFlag = 1;
}

/* ------------------------------------------------ FreeRTOS Portable --------------------------------------------------
 * - Provides implementation for functions required by FreeRTOS
 * - Declared in portable.h
 * ------------------------------------------------------------------------------------------------------------------ */

// ----------------- Scheduler Start/End -------------------

BaseType_t xPortStartScheduler(void)
{
    uxInterruptNesting = 0;
    port_uxCriticalNestingIDF = 0;
    uxSchedulerRunning = 0;

    /* Setup the hardware to generate the tick. */
    vPortSetupTimer();

    esprv_intc_int_set_threshold(1); /* set global INTC masking level */
    rv_utils_intr_global_enable();

    vPortYield();

    /*Should not get here*/
    return pdFALSE;
}

void vPortEndScheduler(void)
{
    /* very unlikely this function will be called, so just trap here */
    abort();
}

// ----------------------- Memory --------------------------

#define FREERTOS_SMP_MALLOC_CAPS    (MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)

void *pvPortMalloc( size_t xSize )
{
    return heap_caps_malloc(xSize, FREERTOS_SMP_MALLOC_CAPS);
}

void vPortFree( void *pv )
{
    heap_caps_free(pv);
}

void vPortInitialiseBlocks( void )
{
    ;   //Does nothing, heap is initialized separately in ESP-IDF
}

size_t xPortGetFreeHeapSize( void )
{
    return esp_get_free_heap_size();
}

#if( configSTACK_ALLOCATION_FROM_SEPARATE_HEAP == 1 )
void *pvPortMallocStack( size_t xSize )
{
    return NULL;
}

void vPortFreeStack( void *pv )
{

}
#endif

// ------------------------ Stack --------------------------
#if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
/**
 * Wrapper to allow task functions to return. Force the optimization option -O1 on that function to make sure there
 * is no tail-call. Indeed, we need the compiler to keep the return address to this function when calling `panic_abort`.
 *
 * Thanks to `naked` attribute, the compiler won't generate a prologue and epilogue for the function, which saves time
 * and stack space.
 */
static void __attribute__((optimize("O1"), naked)) vPortTaskWrapper(TaskFunction_t pxCode, void *pvParameters)
{
    asm volatile(".cfi_undefined ra\n");
    extern void __attribute__((noreturn)) panic_abort(const char *details);
    static char DRAM_ATTR msg[80] = "FreeRTOS: FreeRTOS Task \"\0";
    pxCode(pvParameters);
    //FreeRTOS tasks should not return. Log the task name and abort.
    char *pcTaskName = pcTaskGetName(NULL);
    /* We cannot use s(n)printf because it is in flash */
    strcat(msg, pcTaskName);
    strcat(msg, "\" should not return, Aborting now!");
    panic_abort(msg);
}
#endif // CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER


StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters)
{
    extern uint32_t __global_pointer$;
    uint8_t *task_thread_local_start;
    uint8_t *threadptr;
    extern char _thread_local_start, _thread_local_end, _flash_rodata_start;

    /* Byte pointer, so that subsequent calculations don't depend on sizeof(StackType_t). */
    uint8_t *sp = (uint8_t *) pxTopOfStack;

    /* Set up TLS area.
     * The following diagram illustrates the layout of link-time and run-time
     * TLS sections.
     *
     *          +-------------+
     *          |Section:     |      Linker symbols:
     *          |.flash.rodata|      ---------------
     *       0x0+-------------+ <-- _flash_rodata_start
     *        ^ |             |
     *        | | Other data  |
     *        | |     ...     |
     *        | +-------------+ <-- _thread_local_start
     *        | |.tbss        | ^
     *        v |             | |
     *    0xNNNN|int example; | | (thread_local_size)
     *          |.tdata       | v
     *          +-------------+ <-- _thread_local_end
     *          | Other data  |
     *          |     ...     |
     *          |             |
     *          +-------------+
     *
     *                                Local variables of
     *                              pxPortInitialiseStack
     *                             -----------------------
     *          +-------------+ <-- pxTopOfStack
     *          |.tdata (*)   |  ^
     *        ^ |int example; |  |(thread_local_size
     *        | |             |  |
     *        | |.tbss (*)    |  v
     *        | +-------------+ <-- task_thread_local_start
     * 0xNNNN | |             |  ^
     *        | |             |  |
     *        | |             |  |_thread_local_start - _rodata_start
     *        | |             |  |
     *        | |             |  v
     *        v +-------------+ <-- threadptr
     *
     *   (*) The stack grows downward!
     */

    uint32_t thread_local_sz = (uint32_t) (&_thread_local_end - &_thread_local_start);
    thread_local_sz = ALIGNUP(0x10, thread_local_sz);
    sp -= thread_local_sz;
    task_thread_local_start = sp;
    memcpy(task_thread_local_start, &_thread_local_start, thread_local_sz);
    threadptr = task_thread_local_start - (&_thread_local_start - &_flash_rodata_start);

    /* Simulate the stack frame as it would be created by a context switch interrupt. */
    sp -= RV_STK_FRMSZ;
    RvExcFrame *frame = (RvExcFrame *)sp;
    memset(frame, 0, sizeof(*frame));

    /* Initialize the stack frame. */
    #if CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
        frame->mepc = (UBaseType_t)vPortTaskWrapper;
        frame->a0 = (UBaseType_t)pxCode;
        frame->a1 = (UBaseType_t)pvParameters;
    #else
        frame->mepc = (UBaseType_t)pxCode;
        frame->a0 = (UBaseType_t)pvParameters;
    #endif // CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER
    frame->gp = (UBaseType_t)&__global_pointer$;
    frame->tp = (UBaseType_t)threadptr;

    //TODO: IDF-2393
    configASSERT(((uint32_t) frame & portBYTE_ALIGNMENT_MASK) == 0);
    return (StackType_t *)frame;
}

// ------- Thread Local Storage Pointers Deletion Callbacks -------

#if ( CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS )
void vPortTLSPointersDelCb( void *pxTCB )
{
    /* Typecast pxTCB to StaticTask_t type to access TCB struct members.
     * pvDummy15 corresponds to pvThreadLocalStoragePointers member of the TCB.
     */
    StaticTask_t *tcb = ( StaticTask_t * )pxTCB;

    /* The TLSP deletion callbacks are stored at an offset of (configNUM_THREAD_LOCAL_STORAGE_POINTERS/2) */
    TlsDeleteCallbackFunction_t *pvThreadLocalStoragePointersDelCallback = ( TlsDeleteCallbackFunction_t * )( &( tcb->pvDummy15[ ( configNUM_THREAD_LOCAL_STORAGE_POINTERS / 2 ) ] ) );

    /* We need to iterate over half the depth of the pvThreadLocalStoragePointers area
     * to access all TLS pointers and their respective TLS deletion callbacks.
     */
    for ( int x = 0; x < ( configNUM_THREAD_LOCAL_STORAGE_POINTERS / 2 ); x++ ) {
        if ( pvThreadLocalStoragePointersDelCallback[ x ] != NULL ) {  //If del cb is set
            /* In case the TLSP deletion callback has been overwritten by a TLS pointer, gracefully abort. */
            if ( !esp_ptr_executable( pvThreadLocalStoragePointersDelCallback[ x ] ) ) {
                ESP_LOGE("FreeRTOS", "Fatal error: TLSP deletion callback at index %d overwritten with non-excutable pointer %p", x, pvThreadLocalStoragePointersDelCallback[ x ]);
                abort();
            }

            pvThreadLocalStoragePointersDelCallback[ x ]( x, tcb->pvDummy15[ x ] );   //Call del cb
        }
    }
}
#endif // CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS

// -------------------- Tick Handler -----------------------

extern void esp_vApplicationIdleHook(void);
extern void esp_vApplicationTickHook(void);

BaseType_t xPortSysTickHandler(void)
{
#if configBENCHMARK
    portbenchmarkIntLatency();
#endif //configBENCHMARK
    traceISR_ENTER(SYSTICK_INTR_ID);
    BaseType_t ret = xTaskIncrementTick();
    //Manually call the IDF tick hooks
    esp_vApplicationTickHook();
    if (ret != pdFALSE) {
        portYIELD_FROM_ISR();
    } else {
        traceISR_EXIT();
    }
    return ret;
}

// ------------------- Hook Functions ----------------------

void __attribute__((weak)) vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
#define ERR_STR1 "***ERROR*** A stack overflow in task "
#define ERR_STR2 " has been detected."
    const char *str[] = {ERR_STR1, pcTaskName, ERR_STR2};

    char buf[sizeof(ERR_STR1) + CONFIG_FREERTOS_MAX_TASK_NAME_LEN + sizeof(ERR_STR2) + 1 /* null char */] = {0};

    char *dest = buf;
    for (int i = 0; i < sizeof(str) / sizeof(str[0]); i++) {
        dest = strcat(dest, str[i]);
    }
    esp_system_abort(buf);
}

#if  (  configUSE_TICK_HOOK > 0 )
void vApplicationTickHook( void )
{
    esp_vApplicationTickHook();
}
#endif

#if CONFIG_FREERTOS_USE_MINIMAL_IDLE_HOOK
/*
By default, the port uses vApplicationMinimalIdleHook() to run IDF style idle
hooks. However, users may also want to provide their own vApplicationMinimalIdleHook().
In this case, we use to -Wl,--wrap option to wrap the user provided vApplicationMinimalIdleHook()
*/
extern void __real_vApplicationMinimalIdleHook( void );
void __wrap_vApplicationMinimalIdleHook( void )
{
    esp_vApplicationIdleHook(); //Run IDF style hooks
    __real_vApplicationMinimalIdleHook(); //Call the user provided vApplicationMinimalIdleHook()
}
#else // CONFIG_FREERTOS_USE_MINIMAL_IDLE_HOOK
void vApplicationMinimalIdleHook( void )
{
    esp_vApplicationIdleHook(); //Run IDF style hooks
}
#endif // CONFIG_FREERTOS_USE_MINIMAL_IDLE_HOOK

/*
 * Hook function called during prvDeleteTCB() to cleanup any
 * user defined static memory areas in the TCB.
 */
void vPortCleanUpTCB ( void *pxTCB )
{
#if ( CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS )
    /* Call TLS pointers deletion callbacks */
    vPortTLSPointersDelCb( pxTCB );
#endif /* CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS */
}
