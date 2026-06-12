#include "mission_control/handlers/line_searching.h"
#include "mission_control/mission_control.h"
#include "services/adc_service.h"
#include "services/line_sensor_service.h"
#include "services/motor_service.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

static uint32_t start_time = 0;
static uint32_t current_swirl_duration = 0;
static bool is_swirling_left = false;

/* Configuration */
static const uint32_t INITIAL_SWIRL_DURATION_MS = 200;
static const uint32_t MAX_SWIRL_DURATION_MS = 2000;

void line_searching_run() {
  uint32_t current_time = HAL_GetTick();
  line_sensor_get_error();

  // Check whether the line is found
  if (line_sensor_is_on_line()) {
    motors_stop();
    mission_control_set_state(LINE_FOLLOWING);
    return;
  }

  // Initialize for the first run
  if (current_swirl_duration == 0 && start_time == 0) {
    current_swirl_duration = INITIAL_SWIRL_DURATION_MS;
    start_time = current_time;
    is_swirling_left = true;
    motors_drive_curve(40, 100, LEFT);
    return;
  }

  // Swirl from left to right
  if ((uint32_t)(current_time - start_time) >= current_swirl_duration) {
    is_swirling_left = !is_swirling_left;
    uint32_t next = current_swirl_duration * 2u;
    if (next == 0u || next > MAX_SWIRL_DURATION_MS) {
      next = MAX_SWIRL_DURATION_MS;
    }
    current_swirl_duration = next;
    start_time = current_time;
    motors_drive_curve(40, 100, is_swirling_left ? LEFT : RIGHT);
  }
}

void line_searching_reset() {
  start_time = 0;
  current_swirl_duration = 0;
  motors_stop();
}
