#include "mission_control/mission_control.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/waypoint_navigation.h"
#include "services/motor_service.h"
#include "services/touch_sensor_service.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

const uint16_t MISSION_CONTROL_PERIOD = 5;
static const uint32_t RESET_PAUSE_TIME = 2000;

static mission_control_state_t current_state = WAYPOINT_NAVIGATION;

static volatile bool reset_flag = false;
static volatile uint32_t reset_start = 0;
static void reset() {
  reset_flag = true;
  motors_stop();
  reset_start = HAL_GetTick();
}

void mission_control_init() {
  touch_sensor_subscription_t sub = {&reset, TOUCH_SENSOR_LEFT};
  touch_sensor_subscribe(sub);
}

void mission_control_run() {
  if (!reset_flag) {
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
      // TODO: Use the waypoint navigator
      break;
    }
  } else {
    if (HAL_GetTick() - reset_start >= RESET_PAUSE_TIME) {
      reset_flag = false;
      current_state = WAYPOINT_NAVIGATION;

      // Reset all handlers
      waypoint_navigation_reset();
      line_searching_reset();
    }
  }
}

void mission_control_set_state(mission_control_state_t state) {
  current_state = state;
}

mission_control_state_t mission_control_get_state() { return current_state; }
