#ifndef CALIBRATION_ORCHESTRATOR_H
#define CALIBRATION_ORCHESTRATOR_H

/**
 * The command calibration states for all calibrators
 */
typedef enum calibration_states_t {
  // Calibration hasn't started yet
  NOT_STARTED,
  // Is calibrating right now
  CALIBRATING,
  // Is verifying whether the calibration was successful
  VERIFYING,
  // The sensor is calibrated
  CALIBRATED,
  // Failed calibration
  FAILED,
} calibration_states_t;

#endif // !CALIBRATION_ORCHESTRATOR_H
