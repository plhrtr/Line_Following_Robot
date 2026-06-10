#ifndef CALIBRATION_ORCHESTRATOR_H
#define CALIBRATION_ORCHESTRATOR_H

#include <stdint.h>

/**
 * The running period of the calibration orchestrator
 */
extern const uint16_t CALIBRATION_ORCHESTRATOR_PERIOD;

/**
 * The calibration states for all calibrators
 */
typedef enum {
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
} calibration_state_t;

typedef enum {
  // The orchestrator hasnt been started yet
  CALIBRATION_NOT_STARTED,
  // Indicate the user that a new task is starting
  STARTING_TASK,
  // The orchestrator is processing a task at the moment
  PROCCESSING_TASK,
  // The orchestrator finished a task. Processes the next task or move to
  // [CALIBRATION_FINISHED]
  FINISING_TASK,
  // The orchestrator completed all it's tasks
  CALIBRATION_FINISHED,
} orchestrator_state_t;

/**
 * Represents a single task of the orchestrator
 */
typedef struct {
  // The name of the task
  char name[10];
  // Pointer to a function to run the task
  void (*task_run)();
  // Function to get the state of the task
  calibration_state_t (*get_state)();
} orchestrator_task_t;

/**
 * Set the id so that the orchestrator is able to unschedule itself when it's
 * finished
 */
void calibration_orchestrator_set_unscheduling_id(uint16_t id);

/**
 * Run the calibration orchestrator
 * The running period is defined by [CALIBRATION_ORCHESTRATOR_PERIOD]
 */
void calibration_orchestrator_run();

#endif // CALIBRATION_ORCHESTRATOR_H
