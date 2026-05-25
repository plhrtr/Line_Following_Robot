/**
 * Cooperative scheduler implementation
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

/**
 * Struct defining a single task.
 */
typedef struct {
  // The name of the task. At max 10 characters long.
  char name[10];
  // The function to run the task
  void (*task_run)();
  // The period the task should run. In ms.
  // Use [UINT32_MAX] for a task that should only run once.
  uint32_t period;
  // Variable to store the next run of the task
  // Set it to zero the schedule it immediately
  uint32_t next_run;
} task_t;

/**
 * Schedule the given task
 * @param task - The task you want to schedule
 * @return - An index of the task for later unscheduling. Or -1 if the maximal
 * number of tasks is reached.
 */
int scheduler_schedule(task_t task);

/**
 * Unschedule the task defined by the index
 * @param task_index - The index of the task
 */
void scheduler_unschedule(uint16_t task_index);

/**
 * Infinite loop to run the scheduler
 */
void scheduler_run();

#endif // !SCHEDULER_H
