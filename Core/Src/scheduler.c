#include "scheduler.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

#define MAX_TASKS (sizeof(uint32_t) * 8)

static uint32_t task_bitmap = 0;
static task_t tasks[MAX_TASKS];

int scheduler_schedule(task_t task) {
  if (task_bitmap == ~(uint32_t)0) {
    return -1;
  }

  uint16_t index = __builtin_ctz(~task_bitmap);

  // Mark the task as occupied
  uint32_t mask = (uint32_t)1 << index;
  task_bitmap |= mask;

  // Insert the task into the array
  tasks[index] = task;

  return index;
}

void scheduler_unschedule(uint16_t task_index) {
  if (task_index >= MAX_TASKS)
    return;

  uint32_t mask = (size_t)1 << task_index;

  task_bitmap &= ~mask;
};

void scheduler_run() {
  while (1) {

    uint32_t current_tick = HAL_GetTick();

    for (size_t i = 0; i < MAX_TASKS; i++) {
      uint32_t mask = (uint32_t)1 << i;

      if (!(task_bitmap & mask)) {
        continue;
      }

      task_t *task = &tasks[i];

      uint32_t elapsed = current_tick - task->next_run;

      if ((elapsed & 0x80000000UL) == 0) {
        task->task_run();

        if (task->period != UINT32_MAX) {
          task->next_run += task->period;
        } else {
          scheduler_unschedule(i);
        }
      }
    }
  }
}
