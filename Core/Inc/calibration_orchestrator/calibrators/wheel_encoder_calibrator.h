#ifndef WHEEL_ENCODER_CALIBRATOR_H
#define WHEEL_ENCODER_CALIBRATOR_H

#include "calibration_orchestrator/calibration_orchestrator.h"

/**
 * Calibration function for the wheel encoder
 * Spins the motor for a short period of time to measure the encoder values.
 * Verifies the calibration with a second run
 * Expects the motor service, adc service and wheel encoder service to run
 */
void wheel_encoder_calibrate();

/**
 * Get the state of the wheel encoder calibrator
 * @return - The state of the calibrator
 */
calibration_states_t wheel_encoder_calibrator_get_state();

#endif // WHEEL_ENCODER_CALIBRATOR_H
