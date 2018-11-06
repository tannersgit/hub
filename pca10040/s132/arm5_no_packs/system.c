
#include "nrf52.h"
#include "system.h"

volatile static sys_t sys;

void sys_init( void )
{
	SysTick_Config( SystemCoreClock / SYSTEM_TICKS_PER_SECOND );
	NVIC_EnableIRQ( SysTick_IRQn );	//SysTick interrupts call SysTick_Handler function
}


/* Register a task to be executed every [period]ms */
void sys_task( task_ptr task, uint32_t period )
{
	int i;
	
	for( i = 0; i < SYSTEM_TASKS_MAX && sys.tasks[i] != 0; i++ );
	
	sys.tasks[i] = task;
	sys.taskPeriods[i] = period;
}


void SysTick_Handler(void)
{
	sys.counter++;
	
	for( int i = 0; i < SYSTEM_TASKS_MAX; i++ )
	{
		if( (sys.tasks[i] != 0) && (sys.counter % sys.taskPeriods[i] == 0) )	//if task exists and the system counter is divisible by the period, execute that task
			sys.tasks[i]();
	}
}
