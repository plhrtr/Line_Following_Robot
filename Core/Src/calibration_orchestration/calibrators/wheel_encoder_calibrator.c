#include "calibration_orchestrator/calibrators/wheel_encoder_calibrator.h"
#include "calibration_orchestrator/calibration_orchestrator.h"
#include "services/adc_serive.h"
#include "services/motor_service.h"
#include "services/wheel_encoder_service.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

// The calibration duration
static const uint32_t CALIBRATION_DURATION = 800;
// The duration for the verfication process
static const uint16_t VERIFICATION_DURATION = 6600;
// The number of revolutions expected in the [VERIFICATION_DURATION] timeframe.
static const uint8_t EXPECTED_NUMBER_OF_REVOLUTIONS = 5;
// Percentage of derivation from the middle where no switches will happen.
static const float DEAD_BAND_ZONE = 0.05f;

// The tick where the previous state started
static uint32_t calibration_previous_state_tick = 0;

static calibration_states_t calibration_state = NOT_STARTED;

static uint32_t left_encoder_max = 0;
static uint32_t left_encoder_min = UINT32_MAX;
static uint32_t right_encoder_max = 0;
static uint32_t right_encoder_min = UINT32_MAX;

void wheel_encoder_calibrate() {
  uint32_t current_tick = HAL_GetTick();

  switch (calibration_state) {
  case NOT_STARTED:
    calibration_previous_state_tick = HAL_GetTick();
    calibration_state = CALIBRATING;
    motors_drive_straight(100);
    break;
  case CALIBRATING:
    uint32_t left_encoder_value = adc[ENCODER_LEFT];

    if (left_encoder_value > left_encoder_max) {
      left_encoder_max = left_encoder_value;
    } else if (left_encoder_value < left_encoder_min) {
      left_encoder_min = left_encoder_value;
    }

    uint32_t right_encoder_value = adc[ENCODER_RIGHT];

    if (right_encoder_value > right_encoder_max) {
      right_encoder_max = right_encoder_value;
    } else if (right_encoder_value < right_encoder_min) {
      right_encoder_min = right_encoder_value;
    }

    if (current_tick - calibration_previous_state_tick >=
        CALIBRATION_DURATION) {
      calibration_previous_state_tick = current_tick;

      uint32_t average_left = (left_encoder_max + left_encoder_min) / 2;
      uint32_t average_right = (right_encoder_max + right_encoder_min) / 2;

      uint32_t left_uppper =
          average_left +
          DEAD_BAND_ZONE * (float)(left_encoder_max - average_left);
      uint32_t left_lower =
          average_left -
          DEAD_BAND_ZONE * (float)(average_left - left_encoder_min);
      uint32_t right_upper =
          average_right +
          DEAD_BAND_ZONE * (float)(right_encoder_max - average_right);
      uint32_t right_lower =
          average_right -
          DEAD_BAND_ZONE * (float)(average_right - right_encoder_min);

      wheel_encoder_set_boundaries(left_uppper, left_lower, right_upper,
                                   right_lower);

      calibration_state = VERIFYING;
      wheel_encoder_reset();
      motors_drive_straight(25);
    }
    break;
  case VERIFYING:
    if (current_tick - calibration_previous_state_tick >=
        VERIFICATION_DURATION) {
      distance_t current_distance = wheel_encoder_get_current_distance();

      if (current_distance.distance_left == EXPECTED_NUMBER_OF_REVOLUTIONS &&
          current_distance.distance_right == EXPECTED_NUMBER_OF_REVOLUTIONS) {
        calibration_state = CALIBRATED;
      } else {
        calibration_state = FAILED;
      }
      wheel_encoder_reset();
      motors_drive_straight(100);
    }
    break;
  default:
    return;
  }
}

calibration_states_t wheel_encoder_calibrator_get_state() {
  return calibration_state;
}
