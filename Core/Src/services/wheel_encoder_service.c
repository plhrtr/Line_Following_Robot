#include "services/wheel_encoder_service.h"
#include "logger.h"
#include "services/adc_serive.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

// -------------------------------
// LOGGING SETTINGS FOR THIS FILE
// -------------------------------
static char distance_logging_enabled = 1;
static char velocity_logging_enabled = 0;
static char wheel_encoder_logging_enabled = 0;

static const log_module_t wheel_encoder_log_module = {
    "wheel_encoder_log_module", &wheel_encoder_logging_enabled};

static const log_module_t distance_log_module = {"distance_log_module",
                                                 &distance_logging_enabled};

static const log_module_t velocity_log_module = {"velocity_log_module",
                                                 &velocity_logging_enabled};

// The number of black marks on a single wheel
static const uint8_t BLACK_MARK_COUNT = 12;

// The frequency of sampling the velocity. In ms
static const uint16_t VELOCITY_SAMPLING_FREQ = 400;

// Upper bound for the schmitt trigger of the left wheel
static uint16_t left_schmitt_trigger_upper;

// Lower bound for the schmitt trigger of the left wheel
static uint16_t left_schmitt_trigger_lower;

// Lower bound for the schmitt trigger of the right wheel
static uint16_t right_schmitt_trigger_upper;

// Lower bound for the schmitt trigger of the right wheel
static uint16_t right_schmitt_trigger_lower;

/**
 * Struct holding the current velocity for both wheels.
 */
static velocity_t current_velocity = {0, 0};

/**
 * Struct holding the current distance for both wheels.
 */
static distance_t current_distance = {0, 0};

/**
 * The last value of the schmitt trigger of the right wheel.
 */
static char last_state_right_schmitt_trigger = 0;

/**
 * The last value of the schmitt trigger of the left wheel.
 */
static char last_state_left_schmitt_trigger = 0;

/**
 * The number of seen rising edges for the right wheel.
 * Used for velocity calculations.
 */
static uint16_t right_wheel_rising_edge_counter = 0;

/**
 * The number of seen rising edges for the left wheel.
 * Used for velocity calculations.
 */
static uint16_t left_wheel_rising_edge_counter = 0;

/**
 * The tick where the last sampling window was evaluated
 */
static uint32_t last_tick = 0;

const uint16_t WHEEL_ENCODER_SAMPLING_PERIOD = 2;

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

  uint32_t current_tick = HAL_GetTick();

  // Left encoder
  if (left_encoder_value > left_schmitt_trigger_upper &&
      last_state_left_schmitt_trigger == 0) {
    // Rising edge
    last_state_left_schmitt_trigger = 1;
    current_distance.distance_left++;
    left_wheel_rising_edge_counter++;

    LOGGER_LOG(LOG_DEBUG, distance_log_module,
               "distance (left | right), %u, %u",
               current_distance.distance_left, current_distance.distance_right);
  } else if (left_encoder_value < left_schmitt_trigger_lower &&
             last_state_left_schmitt_trigger == 1) {
    last_state_left_schmitt_trigger = 0;
  }

  // Right encoder
  if (right_encoder_value > right_schmitt_trigger_upper &&
      last_state_right_schmitt_trigger == 0) {
    // Rising edge
    last_state_right_schmitt_trigger = 1;
    current_distance.distance_right++;
    right_wheel_rising_edge_counter++;

    LOGGER_LOG(LOG_DEBUG, distance_log_module,
               "distance (left | right), %u, %u",
               current_distance.distance_left, current_distance.distance_right);
  } else if (right_encoder_value < right_schmitt_trigger_lower &&
             last_state_right_schmitt_trigger == 1) {
    last_state_right_schmitt_trigger = 0;
  }

  // Velocity sampling
  if (current_tick - last_tick >= VELOCITY_SAMPLING_FREQ) {
    last_tick = current_tick;
    current_velocity.velocity_left =
        ((float)left_wheel_rising_edge_counter / (float)BLACK_MARK_COUNT) /
        ((float)VELOCITY_SAMPLING_FREQ / 1000.0f) * 60.0f;
    current_velocity.velocity_right =
        ((float)right_wheel_rising_edge_counter / (float)BLACK_MARK_COUNT) /
        ((float)VELOCITY_SAMPLING_FREQ / 1000.0f) * 60.0f;

    // Reset the counters for velocity
    left_wheel_rising_edge_counter = 0;
    right_wheel_rising_edge_counter = 0;

    LOGGER_LOG(LOG_DEBUG, velocity_log_module,
               "velocity (left | right), %u, %u",
               current_velocity.velocity_left, current_velocity.velocity_right);
  }
};

distance_t wheel_encoder_get_current_distance() {
  distance_t tmp = {current_distance.distance_left,
                    current_distance.distance_right};
  return tmp;
};

velocity_t wheel_encoder_get_current_velocity() {
  velocity_t tmp = {current_velocity.velocity_left,
                    current_velocity.velocity_right};
  return tmp;
};

void wheel_encoder_reset() {
  current_distance.distance_left = 0;
  current_distance.distance_right = 0;

  current_velocity.velocity_left = 0;
  current_velocity.velocity_right = 0;

  LOGGER_LOG(LOG_INFO, wheel_encoder_log_module, "Wheel encoder reseted");
}
