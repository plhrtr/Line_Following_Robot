#include "mission_control/handlers/waypoint_navigation.h"
#include "mission_control/mission_control.h"
#include "services/motor_service.h"
#include "services/wheel_encoder_service.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Conversion factors: encoder segments per mm / per degree */
#define SEGMENTS_PER_MM 0.1909859318f
#define SEGMENTS_PER_DEGREE 0.1333333334f
#define PAUSE_TIME 250U

/* Task list for the waypoint navigation task
   Use integer literals for the 'value' field (was float literals previously).
 */
static waypoint_navigation_task_t yellow_line_tasks[] = {
    {DRIVE_STRAIGHT, 200, 0}, {TURN_RIGHT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_RIGHT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_RIGHT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_RIGHT, 90, 0},
};

static int32_t current_task_index = -1;
static bool is_pause = false;
static uint32_t pause_start = 0;

static void finish_yellow_line_task() {
  mission_control_set_state(LINE_FOLLOWING);
}

static waypoint_navigation_task_t *current_tasks = yellow_line_tasks;
static uint8_t number_of_tasks =
    sizeof(yellow_line_tasks) / sizeof(waypoint_navigation_task_t);
static void (*on_finish)() = &finish_yellow_line_task;

/* Initialize a task using the provided current distance to avoid stale reads.
   Rounds the computed float to nearest integer and stores in a uint32_t goal.
 */
static void initialize_task(waypoint_navigation_task_t *task,
                            const distance_t *curr_distance) {
  uint32_t added_segments = 0U;

  switch (task->type) {
  case DRIVE_STRAIGHT:
    motors_drive_straight(85);
    /* compute segments for mm distance and round to nearest */
    added_segments = (uint32_t)(SEGMENTS_PER_MM * (float)task->value + 0.5f);
    task->goal_distance = curr_distance->distance_left + added_segments;
    break;
  case TURN_LEFT:
    motors_drive_curve(65, 100, LEFT);
    /* turning left is driven by right wheel in this platform -> measure right
     * encoder */
    added_segments =
        (uint32_t)(SEGMENTS_PER_DEGREE * (float)task->value + 0.5f);
    task->goal_distance = curr_distance->distance_right + added_segments;
    break;
  case TURN_RIGHT:
    motors_drive_curve(65, 100, RIGHT);
    /* turning right is driven by left wheel -> measure left encoder */
    added_segments =
        (uint32_t)(SEGMENTS_PER_DEGREE * (float)task->value + 0.5f);
    task->goal_distance = curr_distance->distance_left + added_segments;
    break;
  case DRIVE_BACKWARDS:
    motors_drive_straight(-85);
    added_segments = (uint32_t)(SEGMENTS_PER_MM * (float)task->value + 0.5f);
    task->goal_distance = curr_distance->distance_left + added_segments;
    break;
  default:
    /* Should not happen - stop motors as a safe fallback */
    motors_stop();
    task->goal_distance = 0U;
    break;
  }
}

static inline void task_completed(void) {
  /* Advance to next task */
  current_task_index++;

  /* If we've executed all tasks, stop and switch state */
  if ((size_t)current_task_index >= number_of_tasks) {
    motors_stop();
    on_finish();
    current_task_index = -1;
    is_pause = false;
    return;
  }

  /* Pause between tasks */
  is_pause = true;
  pause_start = HAL_GetTick();
  motors_stop();
}

void waypoint_navigation_run(void) {
  if (!is_pause) {
    distance_t curr_distance = wheel_encoder_get_current_distance();

    /* Initialize the first task */
    if (current_task_index < 0) {
      current_task_index = 0;
      initialize_task(&current_tasks[0], &curr_distance);
      return;
    }

    /* Guard: ensure current_task is valid before indexing */
    if ((size_t)current_task_index >= number_of_tasks) {
      /* Invalid index: stop as safe fallback */
      motors_stop();
      on_finish();
      current_task_index = -1;
      return;
    }

    /* Check whether the current task's goal has been reached */
    switch (current_tasks[current_task_index].type) {
    case TURN_LEFT:
      if (curr_distance.distance_right >=
          current_tasks[current_task_index].goal_distance) {
        task_completed();
      }
      break;
    case TURN_RIGHT:
      if (curr_distance.distance_left >=
          current_tasks[current_task_index].goal_distance) {
        task_completed();
      }
      break;
    case DRIVE_STRAIGHT:
    case DRIVE_BACKWARDS:
    default:
      if (curr_distance.distance_left >=
          current_tasks[current_task_index].goal_distance) {
        task_completed();
      }
      break;
    }
  } else {
    /* Wait until pause time elapsed, then start next task */
    if ((uint32_t)(HAL_GetTick() - pause_start) >= PAUSE_TIME) {
      is_pause = false;
      distance_t curr_distance = wheel_encoder_get_current_distance();
      /* current_task already points to the next task (set in task_completed) */
      if (current_task_index >= 0) {
        initialize_task(&current_tasks[current_task_index], &curr_distance);
      }
    }
  }
}

void waypoint_navigation_reset(void) {
  current_task_index = -1;
  is_pause = false;
}

void waypoint_navigation_set_tasks(waypoint_navigation_task_t *new_tasks,
                                   uint8_t number_of_tasks_param,
                                   void (*on_finish_param)()) {
  number_of_tasks = number_of_tasks_param;
  current_tasks = new_tasks;
  on_finish = on_finish_param;
  waypoint_navigation_reset();
}
