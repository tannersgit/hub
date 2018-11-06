/* basic system functions - SYSTICK, mainly */

/* Header guard */
#ifndef SYSTEM_H__
#define SYSTEM_H__

#include "nrf52.h"


#define SYSTEM_TASKS_MAX					10
#define SYSTEM_TICKS_PER_SECOND		1000

typedef void (*task_ptr)(void);

typedef struct{
	task_ptr tasks[SYSTEM_TASKS_MAX];
	uint32_t taskPeriods[SYSTEM_TASKS_MAX];
	uint32_t counter;
} sys_t;

void sys_init( void );
void sys_task( task_ptr task, uint32_t period );

#endif // SYSTEM_H__
