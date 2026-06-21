#include "mission_control/handlers/line_following.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/mission_control.h"
#include "services/led_service.h"
#include "services/line_sensor_service.h"
#include "services/motor_service.h"
#include "services/touch_sensor_service.h"
#include <stdint.h>

// Controller tuning constants
static const uint16_t P_VALUE = 70;

// General tuning constants
static const float JITTER_THRESHOLD = 2.0f;

static int touch_sensor_unsubscribe_id = -1;

static void unsubscribe_touch_sensor() {
  if (touch_sensor_unsubscribe_id != -1) {
    touch_sensor_unsubscribe(touch_sensor_unsubscribe_id);
    touch_sensor_unsubscribe_id = -1;
  }
}

static int clamp_int(int value) {
  if (value < -100) {
    return -100;
  } else if (value > 100) {
    return 100;
  } else {
    return value;
  }
}

/**
 * Callback that gets executed when an obstacle is detected
 */
static void on_obstacle_detected() {
  unsubscribe_touch_sensor();
  mission_control_set_state(OBSTACLE_AVOIDANCE);
}

void line_following_init() {
  unsubscribe_touch_sensor();
  touch_sensor_subscription_t sub = {
      .sensors = TOUCH_SENSOR_LEFT | TOUCH_SENSOR_MIDDLE | TOUCH_SENSOR_RIGHT,
      .callback = &on_obstacle_detected};
  touch_sensor_unsubscribe_id = touch_sensor_subscribe(sub);
}

void line_following_run(void) {
  // error in [-1,1]: negative = too far right, positive = too far left
  float error = line_sensor_get_error();
  int control_size = 0;
  uint8_t speed = 100;

  float scaled = P_VALUE * error;

  if (scaled > -JITTER_THRESHOLD && scaled < JITTER_THRESHOLD) {
    // Avoid jitter for very small errors
    control_size = 0;
  } else {
    control_size = (int)(scaled > 0.0f ? scaled + 0.5f : scaled - 0.5f);
    control_size = clamp_int(control_size);
  }

  if (line_sensor_is_on_line()) {
    led_off(LED_LEFT);
    led_off(LED_RIGHT);

    if (control_size > 0) {
      motors_drive_curve(speed, control_size, RIGHT);
    } else {
      motors_drive_curve(speed, -control_size, LEFT);
    }
  } else {
    if (line_sensor_sharp_left()) {
      led_on(LED_LEFT);
      led_off(LED_RIGHT);
      line_searching_reset(SEARCH_LEFT);
    } else if (line_sensor_sharp_right()) {
      led_off(LED_LEFT);
      led_on(LED_RIGHT);
      line_searching_reset(SEARCH_RIGHT);
    } else {
      led_on(LED_LEFT);
      led_on(LED_RIGHT);
      line_searching_reset(SEARCH_LEFT);
    }
    unsubscribe_touch_sensor();
    mission_control_set_state(LINE_SEARCHING);
  }
}
