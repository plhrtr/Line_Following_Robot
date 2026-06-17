#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/mission_control.h"
#include "services/line_sensor_service.h"
#include "services/motor_service.h"
#include "services/wheel_encoder_service.h"
#include <stdbool.h>
#include <stdint.h>

static const float SEGMENTS_PER_MM = 0.1909859318f;
static const float SEGMENTS_PER_DEGREE = 0.1333333334f;

typedef enum {
  SEARCH_STATE_FORWARD,
  SEARCH_STATE_TURN_LEFT,
  SEARCH_STATE_RETURN_CENTER_LEFT,
  SEARCH_STATE_TURN_RIGHT,
  SEARCH_STATE_RETURN_CENTER_RIGHT,
  SEARCH_STATE_GAP_FORWARD,
} search_state_t;

static search_state_t search_state = SEARCH_STATE_FORWARD;
static uint32_t goal_distance = 0;
static search_direction_t start_direction = SEARCH_LEFT;

void line_searching_run() {
  if (search_state != SEARCH_STATE_FORWARD) {
    line_sensor_get_error();
    if (line_sensor_is_on_line()) {
      line_following_init();
      mission_control_set_state(LINE_FOLLOWING);
    }
  }

  uint32_t current_dist = wheel_encoder_get_current_distance().distance_left;

  switch (search_state) {
  case SEARCH_STATE_FORWARD:
    if (goal_distance == 0) {
      motors_drive_straight(65);
      goal_distance = current_dist + (SEGMENTS_PER_MM * 20);
    }
    if (current_dist >= goal_distance) {
      goal_distance = current_dist + (SEGMENTS_PER_DEGREE * 100);
      switch (start_direction) {
      case SEARCH_LEFT:
        motors_drive_curve(40, 100, LEFT);
        search_state = SEARCH_STATE_TURN_LEFT;
        break;
      case SEARCH_RIGHT:
        motors_drive_curve(40, 100, RIGHT);
        search_state = SEARCH_STATE_TURN_RIGHT;

        break;
      }
    }
    break;

  case SEARCH_STATE_TURN_LEFT:
    if (current_dist >= goal_distance) {
      motors_drive_curve(40, 100, RIGHT);
      goal_distance = current_dist + (SEGMENTS_PER_DEGREE * 100);
      search_state = SEARCH_STATE_RETURN_CENTER_LEFT;
    }
    break;

  case SEARCH_STATE_RETURN_CENTER_LEFT:
    // Based on the start direction turn right or switch to the gap state
    if (current_dist >= goal_distance) {
      if (start_direction == SEARCH_RIGHT) {
        motors_drive_straight(65);
        goal_distance = current_dist + (SEGMENTS_PER_MM * 130);
        search_state = SEARCH_STATE_GAP_FORWARD;
      } else {
        motors_drive_curve(40, 100, RIGHT);
        goal_distance = current_dist + (SEGMENTS_PER_DEGREE * 100);
        search_state = SEARCH_STATE_TURN_RIGHT;
      }
    }
    break;

  case SEARCH_STATE_TURN_RIGHT:
    if (current_dist >= goal_distance) {
      motors_drive_curve(40, 100, LEFT);
      goal_distance = current_dist + (SEGMENTS_PER_DEGREE * 100);
      search_state = SEARCH_STATE_RETURN_CENTER_RIGHT;
    }
    break;

  case SEARCH_STATE_RETURN_CENTER_RIGHT:
    // Based on the start direction turn left or switch to the gap state
    if (current_dist >= goal_distance) {
      if (start_direction == SEARCH_LEFT) {
        motors_drive_straight(65);
        goal_distance = current_dist + (SEGMENTS_PER_MM * 130);
        search_state = SEARCH_STATE_GAP_FORWARD;
      } else {
        motors_drive_curve(40, 100, RIGHT);
        goal_distance = current_dist + (SEGMENTS_PER_DEGREE * 100);
        search_state = SEARCH_STATE_TURN_LEFT;
      }
    }
    break;

  case SEARCH_STATE_GAP_FORWARD:
    if (current_dist >= goal_distance) {
      // Start line search from the beginning after gap
      line_searching_reset(SEARCH_LEFT);
    }
    break;
  }
}

void line_searching_reset(search_direction_t direction) {
  goal_distance = 0;
  search_state = SEARCH_STATE_FORWARD;
  start_direction = direction;
}
