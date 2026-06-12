#include "mission_control/handlers/line_following.h"
#include "services/line_sensor_service.h"
#include "services/motor_service.h"
#include <stdint.h>

#define P_VALUE 125

static uint8_t minimum_speed = 40;

static int clamp_int(int value) {
  if (value < -100) {
    return -100;
  } else if (value > 100) {
    return 100;
  } else {
    return value;
  }
}

void line_following_run(void) {
  // error in [-1,1]: negative = too far right, positive = too far left
  float error = line_sensor_get_error();
  int control_size = 0;
  uint8_t speed = 0;

  // if (line_sensor_is_on_line()) {
  float scaled = P_VALUE * error;

  // Deadband to avoid actuator chatter for very small errors
  if (scaled > -0.5f && scaled < 0.5f) {
    control_size = 0;
    speed = 100;
  } else {
    control_size = (int)(scaled > 0.0f ? scaled + 0.5f : scaled - 0.5f);
    control_size = clamp_int(control_size);
    speed = 100 - (error > 0 ? error : -error) * 100;
  }
  // } else {
  //   control_size = 0;
  //   speed = minimum_speed;
  // }

  speed = speed > minimum_speed ? speed : minimum_speed;

  if (control_size > 0) {
    motors_drive_curve(speed, control_size, RIGHT);
  } else {
    motors_drive_curve(speed, -control_size, LEFT);
  }
}
