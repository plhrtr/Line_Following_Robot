#include "mission_control/mission_control.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/obstacle_avoidance.h"
#include "mission_control/handlers/overcome_gap.h"
#include "mission_control/handlers/yellow_line_following.h"
#include <stdbool.h>
#include <stdint.h>

const uint16_t MISSION_CONTROL_PERIOD = 2;

// The current state the mission control is in
static mission_control_state_t current_state = YELLOW_LINE_FOLLOWING;

void mission_control_init() { yellow_line_following_init(); }

void mission_control_run() {
  switch (current_state) {
  case YELLOW_LINE_FOLLOWING:
    yellow_line_following_run();
    // Always transitions to line searching
    break;
  case LINE_SEARCHING:
    // If a line is found in the search range -> Switch to line following
    // If not overcome gap
    line_searching_run();
    break;
  case LINE_FOLLOWING:
    // If the line is lost transitions to line searching
    // If a touch sensor press is detected transitions to obstacle_avoidance
    line_following_run();
    break;
  case OBSTACLE_AVOIDANCE:
    // Always transitions to line searching
    obstacle_avoidance_run();
    break;
  case OVERCOME_GAP:
    // transitions to line following if line is found
    // If not transitions after specified travel distance to Line searching
    overcome_gap_run();
    break;
  }
}

void mission_control_set_state(mission_control_state_t state) {
  current_state = state;
}

mission_control_state_t mission_control_get_state() { return current_state; }
