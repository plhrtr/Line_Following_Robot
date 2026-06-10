#include "services/wheel_encoder_service.h"
#include "services/adc_service.h"
#include <stdbool.h>
#include <stdint.h>

/* Schmitt trigger boundaries */
static uint16_t left_schmitt_trigger_upper;
static uint16_t left_schmitt_trigger_lower;
static uint16_t right_schmitt_trigger_upper;
static uint16_t right_schmitt_trigger_lower;

/* Current distances for both wheels */
static distance_t current_distance = {0, 0};

/* Last schmitt states (use bool for clarity) */
static bool last_state_right_schmitt_trigger = false;
static bool last_state_left_schmitt_trigger = false;

const uint16_t WHEEL_ENCODER_SAMPLING_PERIOD = 2;

/* Inline helpers avoid duplicated logic and keep type safety by updating struct
 * fields directly. */
static inline void process_left_encoder(uint16_t value) {
  if (value > left_schmitt_trigger_upper) {
    if (!last_state_left_schmitt_trigger) {
      last_state_left_schmitt_trigger = true;
      current_distance.distance_left++;
    }
  } else if (value < left_schmitt_trigger_lower) {
    if (last_state_left_schmitt_trigger) {
      last_state_left_schmitt_trigger = false;
      current_distance.distance_left++;
    }
  }
}

static inline void process_right_encoder(uint16_t value) {
  if (value > right_schmitt_trigger_upper) {
    if (!last_state_right_schmitt_trigger) {
      last_state_right_schmitt_trigger = true;
      current_distance.distance_right++;
    }
  } else if (value < right_schmitt_trigger_lower) {
    if (last_state_right_schmitt_trigger) {
      last_state_right_schmitt_trigger = false;
      current_distance.distance_right++;
    }
  }
}

void wheel_encoder_set_boundaries(uint16_t left_schmitt_trigger_upper_param,
                                  uint16_t left_schmitt_trigger_lower_param,
                                  uint16_t right_schmitt_trigger_upper_param,
                                  uint16_t right_schmitt_trigger_lower_param) {
  left_schmitt_trigger_upper = left_schmitt_trigger_upper_param;
  left_schmitt_trigger_lower = left_schmitt_trigger_lower_param;
  right_schmitt_trigger_upper = right_schmitt_trigger_upper_param;
  right_schmitt_trigger_lower = right_schmitt_trigger_lower_param;
}

void wheel_encoder_update() {
  uint16_t left_encoder_value = adc[ENCODER_LEFT];
  uint16_t right_encoder_value = adc[ENCODER_RIGHT];

  process_left_encoder(left_encoder_value);
  process_right_encoder(right_encoder_value);
}

distance_t wheel_encoder_get_current_distance() {
  /* return a copy (value semantics) */
  return current_distance;
}

void wheel_encoder_reset() {
  current_distance.distance_left = 0;
  current_distance.distance_right = 0;
}
