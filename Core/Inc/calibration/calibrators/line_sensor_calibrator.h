#ifndef LINE_SENSOR_CALIBRATOR_H
#define LINE_SENSOR_CALIBRATOR_H

#include "calibration/orchestrator.h"

/**
 * Calibration function for the line sensor
 *
 * Expects adc service to run
 */
void line_sensor_calibrate();

/**
 * Get the state of the line sensor calibrator
 * @return - The state of the calibrator
 */
calibration_state_t line_sensor_calibrator_get_state();

#endif // LINE_SENSOR_CALIBRATOR_H
