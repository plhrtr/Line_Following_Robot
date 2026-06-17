#include "mission_control/handlers/waypoint_navigation.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/mission_control.h"
#include "services/motor_service.h"
#include "services/wheel_encoder_service.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static const float SEGMENTS_PER_MM = 0.1909859318f;
static const float SEGMENTS_PER_DEGREE = 0.1333333334f;
static const uint16_t PAUSE_TIME = 150;

// INFO: Subtract 40mm from the last distance for better sensor alignment at the
// end

// Task list for the waypoint navigation task.
static waypoint_navigation_task_t yellow_line_tasks[] = {{DRIVE_STRAIGHT, 425},
                                                         {TURN_RIGHT, 150},
                                                         {DRIVE_STRAIGHT, 250},
                                                         {TURN_LEFT, 45},
                                                         {DRIVE_STRAIGHT, 160}};

static int32_t current_task_index = -1;
static bool is_pause = false;
static uint32_t pause_start = 0;

/**
 * Callback that gets executed when the tasks are finished
 */
static void on_finish_yellow_line_task() {
  line_searching_reset(SEARCH_LEFT);
  mission_control_set_state(LINE_SEARCHING);
}

static waypoint_navigation_task_t *current_tasks = yellow_line_tasks;
static uint8_t number_of_tasks =
    sizeof(yellow_line_tasks) / sizeof(waypoint_navigation_task_t);
static void (*on_finish)() = &on_finish_yellow_line_task;

static void initialize_task(waypoint_navigation_task_t *task,
                            const distance_t *curr_distance) {
  uint32_t added_segments = 0U;

  switch (task->type) {
  case DRIVE_STRAIGHT:
    motors_drive_straight(60);
    added_segments = (uint32_t)(SEGMENTS_PER_MM * (float)task->value + 0.5f);
    task->goal_distance_left = curr_distance->distance_left + added_segments;
    task->goal_distance_right = curr_distance->distance_right + added_segments;
    break;
  case DRIVE_BACKWARDS:
    motors_drive_straight(-60);
    added_segments = (uint32_t)(SEGMENTS_PER_MM * (float)task->value + 0.5f);
    task->goal_distance_left = curr_distance->distance_left + added_segments;
    task->goal_distance_right = curr_distance->distance_right + added_segments;
    break;
  case TURN_LEFT:
    motors_drive_curve(40, 100, LEFT);
    added_segments =
        (uint32_t)(SEGMENTS_PER_DEGREE * (float)task->value + 0.5f);
    task->goal_distance_left = curr_distance->distance_left + added_segments;
    task->goal_distance_right = curr_distance->distance_right + added_segments;
    break;
  case TURN_RIGHT:
    motors_drive_curve(40, 100, RIGHT);
    added_segments =
        (uint32_t)(SEGMENTS_PER_DEGREE * (float)task->value + 0.5f);
    task->goal_distance_left = curr_distance->distance_left + added_segments;
    task->goal_distance_right = curr_distance->distance_right + added_segments;
    break;
  default:
    // Fallback to stop the motor on an invalid task type
    motors_stop();
    task->goal_distance_left = 0U;
    task->goal_distance_right = 0U;
    break;
  }
}

static inline void task_completed(void) {
  // Advance to the next task
  current_task_index++;

  // Check whether all tasks are executed
  if ((size_t)current_task_index >= number_of_tasks) {
    motors_stop();
    on_finish();
    current_task_index = -1;
    is_pause = false;
    return;
  }

  // Start the pause time
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
      motors_stop();
      on_finish();
      current_task_index = -1;
      return;
    }

    /* Check whether the current task's goal has been reached */
    switch (current_tasks[current_task_index].type) {
    case DRIVE_BACKWARDS:
    case DRIVE_STRAIGHT:
    case TURN_LEFT:
    case TURN_RIGHT:
      if (curr_distance.distance_left >=
              current_tasks[current_task_index].goal_distance_left ||
          curr_distance.distance_right >=
              current_tasks[current_task_index].goal_distance_right) {
        task_completed();
      }
      break;
    }
  } else {
    if ((uint32_t)(HAL_GetTick() - pause_start) >= PAUSE_TIME) {
      is_pause = false;
      distance_t curr_distance = wheel_encoder_get_current_distance();
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

void waypoint_navigation_set_default(void) {
  current_tasks = yellow_line_tasks;
  number_of_tasks =
      sizeof(yellow_line_tasks) / sizeof(waypoint_navigation_task_t);
  on_finish = &on_finish_yellow_line_task;
  waypoint_navigation_reset();
}

void waypoint_navigation_set_tasks(waypoint_navigation_task_t *new_tasks,
                                   uint8_t number_of_tasks_param,
                                   void (*on_finish_param)()) {
  number_of_tasks = number_of_tasks_param;
  current_tasks = new_tasks;
  on_finish = on_finish_param;
  waypoint_navigation_reset();
}
