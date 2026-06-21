#include "mission_control/handlers/overcome_gap.h"
#include "mission_control/handlers/line_following.h"
#include "mission_control/mission_control.h"
#include "services/line_sensor_service.h"
#include "services/wheel_encoder_service.h"
#include <stdint.h>

static const float SEGMENTS_PER_MM = 0.1909859318f;

static uint32_t goal_distance = 0;

void overcome_gap_run() {
  // If on the line continue in line following task
  line_sensor_get_error();
  if (line_sensor_is_on_line()) {
    goal_distance = 0;
    line_following_init();
    mission_control_set_state(LINE_FOLLOWING);
  }

  if (goal_distance == 0) {
    goal_distance = wheel_encoder_get_current_distance().distance_left +
                    130 * SEGMENTS_PER_MM;
  } else if (wheel_encoder_get_current_distance().distance_left >=
             goal_distance) {
    goal_distance = 0;
    mission_control_set_state(LINE_SEARCHING);
  }
}
