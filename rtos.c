/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#include "fsl_debug_console.h"
#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				3
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000
#define RTOS_INVALID_TASK			-1
#define RTOS_MAX_NUMBER_OF_TASK		10
#define STACK_PC_OFFSET				2

#define SP_OFFSET_NORMAL 9
#define SP_OFFSET_ISR 9

#define TRUE 1
#define FALSE 0
/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum {
    S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum {
    kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct {
    uint8_t priority;
    task_state_e state;
    uint32_t *sp;
    void (*task_body)();
    rtos_tick_t local_tick;
    uint32_t reserved[10];
    uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct {
    uint8_t nTasks;
    rtos_task_handle_t current_task;
    rtos_task_handle_t next_task;
    rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
    rtos_tick_t global_tick;
} task_list = { 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

static uint8_t CS_firstTime = TRUE;

void rtos_start_scheduler(void) {
#ifdef RTOS_ENABLE_IS_ALIVE
    init_is_alive();
    #endif
#if 1 //duda
    task_list.global_tick = 0; /**global timer set to 0*/
    rtos_create_task(idle_task, 0, kAutoStart); /**creates the idle task with the lowest priority and set to auto start*/
#endif
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk;
    reload_systick();

    for (;;)
        ;
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
                                    rtos_autostart_e autostart) {
    rtos_task_handle_t retval;
    if (RTOS_MAX_NUMBER_OF_TASK > task_list.nTasks)
    { /**if there is still room for more tasks*/
        if (kAutoStart == autostart)
        { /**if the autostart parameter received equals kautostart value,*/
            task_list.tasks[task_list.current_task].state = S_READY; //sets the current task as ready
        } else
        {
            task_list.tasks[task_list.current_task].state = S_SUSPENDED; //sets the current task as suspended
        }
        task_list.tasks[task_list.nTasks].sp =
                &task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - 1
                        - STACK_FRAME_SIZE];
        task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE
                - STACK_PC_OFFSET] = (uint32_t) task_body; /**initial stack frame loaded with the task body*/
        task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE
                - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT; /**initial stack frame loaded with the default PSR value*/
        task_list.tasks[task_list.nTasks].local_tick = 0; /**local clock set to 0*/
        task_list.tasks[task_list.nTasks].priority = priority;
        task_list.tasks[task_list.nTasks].task_body = task_body;
//task_list.tasks[task_list.nTasks].state = kStartSuspended == autostart ? S_SUSPENDED : S_READY;
        retval = task_list.nTasks; /**the return value is assigned with the current task number*/
        task_list.nTasks++; /**the task index is increased*/
    } else
    { /**if there is no more room for tasks,*/
        retval = RTOS_INVALID_TASK; /**returns -1 (invalid task)*/
    }
    return retval; /**the return value is the task number*/
}

rtos_tick_t rtos_get_clock(void) {
    return task_list.global_tick; /**returns the system's clock value*/
}

void rtos_delay(rtos_tick_t ticks) {
    task_list.tasks[task_list.current_task].state = S_WAITING; /**sets the current task to a waiting state*/
    task_list.tasks[task_list.current_task].local_tick = ticks; /**loads the current task timer with the received ticks*/
    dispatcher(kFromNormalExec); /**calls the dispatcher, from within the task*/
}

void rtos_suspend_task(void) {
    task_list.tasks[task_list.current_task].state = S_SUSPENDED; /**sets the current task to a suspended state*/
    dispatcher(kFromNormalExec); /**calls the dispatcher, from within the task*/
}

void rtos_activate_task(rtos_task_handle_t task) {
    task_list.tasks[task].state = S_READY; /**sets the current task to a ready state*/
    dispatcher(kFromNormalExec); /**calls the dispatcher, from within the task*/
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void) {
    SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
                                  CLOCK_GetCoreSysClkFreq());
    SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type) { /**this function handles the scheduling policies*/
    rtos_task_handle_t next_task = RTOS_INVALID_TASK; /**next task is idle task (which is equivalent to an invalid task)*/
    rtos_task_handle_t index;
    int8_t highest = -1; /**the highest priority is set to -1, the lowest one*/
    for (index = 0; index < task_list.nTasks; index++)
    {
        if (highest < task_list.tasks[index].priority
                && (S_READY == task_list.tasks[index].state
                        || S_RUNNING == task_list.tasks[index].state)) /**if the current task priority is greater than the current highest priority,*/
        {
            highest = task_list.tasks[index].priority; /**the highest priority is that of the task*/
            next_task = index; /**the next task will be the current for loop index*/
        }
    }
    if (task_list.current_task != next_task) /**if the next_task variable is different form the task list's current task,*/
    {
        task_list.next_task = next_task; /**the the task list's current task will bear the next_task variable value*/
        context_switch(type); /**calls the context switch function, from within the task*/
    }
}

FORCE_INLINE static void context_switch(task_switch_type_e type) {
    register uint32_t *sp asm("sp"); //queremos una variable llamada sp equivalente al sp
    if (FALSE == CS_firstTime)
    {
        if(kFromNormalExec == type){
            task_list.tasks[task_list.current_task].sp = sp - SP_OFFSET_NORMAL; /**saves the current sp in the current task's sp*/
        }else{
            task_list.tasks[task_list.current_task].sp = sp + SP_OFFSET_ISR; /**saves the current sp in the current task's sp*/
        }
    }
    CS_firstTime =  FALSE ;


    task_list.current_task = task_list.next_task; /**changes the current task for the next task*/
    task_list.tasks[task_list.current_task].state = S_RUNNING; /**changes current task to a running state*/
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk; /**calls the pendSV to make a push of the current state to the sp by setting the pendSVSET_mask*/
}

static void activate_waiting_tasks() {
    uint8_t index;
    for (index = 0; index < task_list.nTasks; index++)
    {
        if (S_WAITING == task_list.tasks[index].state)  /**if the current index task is waiting*/
        {
            task_list.tasks[index].local_tick--;        /**decreases current index task local timer*/
            if (0 == task_list.tasks[index].local_tick) /**if current index task local timer equals zero,*/
            {
                task_list.tasks[index].state = S_READY;    /**current index task state is changed to ready*/
            }
        }
    }
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void) {
    PRINTF("LOL");
    for (;;)
    {

    }
}

/**********************************************************************************/
// ISR implementation
/**********************************************************************************/

void SysTick_Handler(void) {
#ifdef RTOS_ENABLE_IS_ALIVE
    refresh_is_alive();
#endif
    task_list.global_tick++; /**global timer increased by one unit*/
    activate_waiting_tasks();
    dispatcher(kFromISR); /**calls the dispatcher function from within the ISR*/
    reload_systick();
}

void PendSV_Handler(void) {
    register uint32_t *r0 asm("r0");
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    r0 = task_list.tasks[task_list.current_task].sp;
    //asm("add r7,r0,#0");
    asm("mov r7,r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void) {
    gpio_pin_config_t gpio_config = { kGPIO_DigitalOutput, 1, };

    port_pin_config_t port_config = { kPORT_PullDisable, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };
    CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
    PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
                      &port_config);
    GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
                 &gpio_config);
}

static void refresh_is_alive(void) {
    static uint8_t state = 0;
    static uint32_t count = 0;
    if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
    {
        GPIO_WritePinOutput(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
                            state);
        state = state == 0 ? 1 : 0;
        count = 0;
    } else
    {
        count++;
    }
}
#endif
