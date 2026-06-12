#include "calibration/calibrators/line_sensor_calibrator.h"
#include "calibration/orchestrator.h"
#include "services/adc_service.h"
#include "services/line_sensor_service.h"
#include "stm32l4xx_hal.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>

static const uint32_t CALIBRATION_PERIOD = 3000;
static line_sensor_thresholds_t thresholds = {0};
static uint32_t calibration_previous_state_tick = 0;
static calibration_state_t calibration_state = NOT_STARTED;

static inline void update_min_max(uint32_t value, uint32_t *max,
                                  uint32_t *min) {
  if (value > *max) {
    *max = value;
  }
  if (value < *min) {
    *min = value;
  }
}

void line_sensor_calibrate() {
  uint32_t current_tick = HAL_GetTick();

  switch (calibration_state) {
  case NOT_STARTED:
    /* initialize min/max so mins start high and maxes start low */
    thresholds.left_lower = UINT32_MAX;
    thresholds.left_upper = 0;
    thresholds.middle_lower = UINT32_MAX;
    thresholds.middle_upper = 0;
    thresholds.right_lower = UINT32_MAX;
    thresholds.right_upper = 0;

    calibration_previous_state_tick = current_tick;
    calibration_state = CALIBRATING;
    break;
  case CALIBRATING:
    // Update the thresholds
    uint32_t left = adc[LINE_SENSOR_LEFT];
    update_min_max(left, &thresholds.left_upper, &thresholds.left_lower);
    uint32_t middle = adc[LINE_SENSOR_MIDDLE];
    update_min_max(middle, &thresholds.middle_upper, &thresholds.middle_lower);
    uint32_t right = adc[LINE_SENSOR_RIGHT];
    update_min_max(right, &thresholds.right_upper, &thresholds.right_lower);

    // Calibration period is over
    if ((current_tick - calibration_previous_state_tick) >=
        CALIBRATION_PERIOD) {
      line_sensor_set_thresholds(thresholds);
      calibration_state = CALIBRATED;
    }
    break;
  default:
    return;
  }
}

calibration_state_t line_sensor_calibrator_get_state() {
  return calibration_state;
}
