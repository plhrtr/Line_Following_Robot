#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/mission_control.h"
#include "services/line_sensor_service.h"
#include "services/motor_service.h"
#include "services/waypoint_navigation.h"
#include "services/wheel_encoder_service.h"
#include <stdbool.h>
#include <stdint.h>

static const float SEGMENTS_PER_MM = 0.1909859318f;

typedef enum { SEARCH_STATE_FORWARD, SEARCH_STATE_SEARCH } search_state_t;

static waypoint_navigation_task_t search_left_tasks[] = {
    {TURN_LEFT, 100}, {TURN_RIGHT, 200}, {TURN_LEFT, 100}};

static waypoint_navigation_task_t search_right_tasks[] = {
    {TURN_RIGHT, 100}, {TURN_LEFT, 200}, {TURN_RIGHT, 100}};

static void on_search_finish() {
  line_searching_reset(SEARCH_LEFT);
  mission_control_set_state(OVERCOME_GAP);
}

static search_state_t search_state = SEARCH_STATE_FORWARD;
static uint32_t goal_distance = 0;
static search_direction_t start_direction = SEARCH_LEFT;

void line_searching_run() {
  // Check whether the line is found (only after traveling a short distance
  // forward to avoid false positives)
  if (search_state != SEARCH_STATE_FORWARD) {
    line_sensor_get_error();
    if (line_sensor_is_on_line()) {
      line_following_init();
      mission_control_set_state(LINE_FOLLOWING);
      return;
    }
  }

  switch (search_state) {
  case SEARCH_STATE_FORWARD:
    if (goal_distance == 0) {
      goal_distance += 20 * SEGMENTS_PER_MM;
    } else if (wheel_encoder_get_current_distance().distance_left >=
               goal_distance) {
      motors_stop();
      search_state = SEARCH_STATE_SEARCH;
      switch (start_direction) {
      case SEARCH_LEFT:
        waypoint_navigation_set_tasks(search_left_tasks, 3, &on_search_finish);
        break;
      case SEARCH_RIGHT:
        waypoint_navigation_set_tasks(search_right_tasks, 3, &on_search_finish);
        break;
      }
    }
    break;
  case SEARCH_STATE_SEARCH:
    waypoint_navigation_run();
    break;
  }
}

void line_searching_reset(search_direction_t direction) {
  goal_distance = 0;
  search_state = SEARCH_STATE_FORWARD;
  start_direction = direction;
}
