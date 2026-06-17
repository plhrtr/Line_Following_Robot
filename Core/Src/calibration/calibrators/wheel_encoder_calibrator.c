#include "calibration/calibrators/wheel_encoder_calibrator.h"
#include "calibration/orchestrator.h"
#include "mission_control/handlers/waypoint_navigation.h"
#include "services/adc_service.h"
#include "services/motor_service.h"
#include "services/wheel_encoder_service.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Numerator and de-numerator representation of the schmitt trigger dead band
// zone
static const uint16_t DEAD_BAND_NUM = 2u;
static const uint16_t DEAD_BAND_DEN = 100u;

static const uint32_t CALIBRATION_DURATION = 500;

static uint32_t calibration_previous_state_tick = 0;
static calibration_state_t calibration_state = NOT_STARTED;

static uint32_t left_encoder_max = 0;
static uint32_t left_encoder_min = UINT32_MAX;
static uint32_t right_encoder_max = 0;
static uint32_t right_encoder_min = UINT32_MAX;

// Drive a square to verify whether the verification was successful
static waypoint_navigation_task_t verificiation_task[] = {
    {DRIVE_STRAIGHT, 200, 0}, {TURN_LEFT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_LEFT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_LEFT, 90, 0},
    {DRIVE_STRAIGHT, 200, 0}, {TURN_LEFT, 90, 0},
};

// Flag holding whether the waypoint navigator has finished
static volatile bool verification_finished = false;

/**
 * Callback that sets the verification finished flag on the finish of the
 * waypoint navigator
 */
static void on_waypoint_finish() { verification_finished = true; }

static inline void update_min_max(uint32_t value, uint32_t *max,
                                  uint32_t *min) {
  if (value > *max) {
    *max = value;
  }
  if (value < *min) {
    *min = value;
  }
}

static inline void compute_deadband_bounds(uint32_t max, uint32_t min,
                                           uint32_t *upper, uint32_t *lower) {
  uint32_t avg = (uint32_t)(((uint64_t)max + (uint64_t)min) / 2u);

  uint32_t up_delta =
      (uint32_t)((((uint64_t)max - (uint64_t)avg) * DEAD_BAND_NUM +
                  (DEAD_BAND_DEN / 2u)) /
                 DEAD_BAND_DEN);
  uint32_t low_delta =
      (uint32_t)((((uint64_t)avg - (uint64_t)min) * DEAD_BAND_NUM +
                  (DEAD_BAND_DEN / 2u)) /
                 DEAD_BAND_DEN);

  *upper = avg + up_delta;
  *lower = avg - low_delta;
}

void wheel_encoder_calibrate() {
  uint32_t current_tick = HAL_GetTick();

  switch (calibration_state) {
  case NOT_STARTED:
    calibration_previous_state_tick = current_tick;
    calibration_state = CALIBRATING;
    wheel_encoder_reset();
    motors_drive_straight(100);
    break;

  case CALIBRATING: {
    /* Read ADC inputs once per invocation and update min/max trackers */
    uint32_t left_encoder_value = adc[ENCODER_LEFT];
    uint32_t right_encoder_value = adc[ENCODER_RIGHT];

    update_min_max(left_encoder_value, &left_encoder_max, &left_encoder_min);
    update_min_max(right_encoder_value, &right_encoder_max, &right_encoder_min);

    /* Transition when duration elapsed */
    if ((current_tick - calibration_previous_state_tick) >=
        CALIBRATION_DURATION) {
      calibration_previous_state_tick = current_tick;

      uint32_t left_upper, left_lower, right_upper, right_lower;

      compute_deadband_bounds(left_encoder_max, left_encoder_min, &left_upper,
                              &left_lower);
      compute_deadband_bounds(right_encoder_max, right_encoder_min,
                              &right_upper, &right_lower);

      wheel_encoder_set_boundaries(left_upper, left_lower, right_upper,
                                   right_lower);

      /* Prepare for verification */
      calibration_state = VERIFYING;
      waypoint_navigation_set_tasks(verificiation_task, 8, &on_waypoint_finish);
      motors_stop();
    }
    break;
  }
  case VERIFYING: {
    if (verification_finished) {
      calibration_state = CALIBRATED;
      waypoint_navigation_set_default();
      verification_finished = false;
      wheel_encoder_reset();
      return;
    }

    waypoint_navigation_run();
  }
  default:
    return;
  }
}

calibration_state_t wheel_encoder_calibrator_get_state() {
  return calibration_state;
}
