#include "mission_control/mission_control.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/obstacle_avoidance.h"
#include "mission_control/handlers/waypoint_navigation.h"
#include <stdbool.h>
#include <stdint.h>

const uint16_t MISSION_CONTROL_PERIOD = 2;

// The current state the mission control is in
static mission_control_state_t current_state = WAYPOINT_NAVIGATION;

void mission_control_init() {}

void mission_control_run() {
  switch (current_state) {
  case WAYPOINT_NAVIGATION:
    waypoint_navigation_run();
    break;
  case LINE_SEARCHING:
    line_searching_run();
    break;
  case LINE_FOLLOWING:
    line_following_run();
    break;
  case OBSTACLE_AVOIDANCE:
    obstacle_avoidance_run();
    break;
  }
}

void mission_control_set_state(mission_control_state_t state) {
  current_state = state;
}

mission_control_state_t mission_control_get_state() { return current_state; }
