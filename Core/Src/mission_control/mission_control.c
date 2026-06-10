#include "mission_control/mission_control.h"
#include "mission_control/handlers/waypoint_navigation.h"
#include "services/touch_sensor_service.h"
#include <stdbool.h>
#include <stdint.h>

const uint16_t MISSION_CONTROL_PERIOD = 10;

static mission_control_state_t current_state = WAYPOINT_NAVIGATION;
static volatile bool reset_flag = false;
static void reset() { reset_flag = true; }

void mission_control_init() {
  touch_sensor_subscription_t sub = {&reset, TOUCH_SENSOR_MIDDLE};
  touch_sensor_subscribe(sub);
}

void mission_control_run() {
  if (reset_flag) {
    reset_flag = false;
    current_state = WAYPOINT_NAVIGATION;

    // Reset all handler
    waypoint_navigation_reset();
  } else {
    switch (current_state) {
    case WAYPOINT_NAVIGATION:
      waypoint_navigation_run();
      break;
    case LINE_FOLLOWING:
      break;
    case OBSTACLE_AVOIDANCE:
      // TODO: Use the waypoint navigator
      break;
    case LINE_LOST:
      break;
    }
  }
}

void mission_control_set_state(mission_control_state_t state) {
  current_state = state;
}

mission_control_state_t mission_control_get_state() { return current_state; }
