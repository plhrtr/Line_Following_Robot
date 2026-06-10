#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <stdint.h>

/**
 * Defines the states the mission control can be in
 * Each state has it's own handler
 */
typedef enum {
  WAYPOINT_NAVIGATION,
  LINE_FOLLOWING,
  OBSTACLE_AVOIDANCE,
  LINE_LOST,
} mission_control_state_t;

// The scheduling period for the mission control
extern const uint16_t MISSION_CONTROL_PERIOD;

// Initialize the mission control
void mission_control_init();

// Run function to schedule the mission control
void mission_control_run();

/**
 * Update the state of the mission control
 * @param state - The new state you want to set the mission control to
 */
void mission_control_set_state(mission_control_state_t state);

// Get the state of the mission control
mission_control_state_t mission_control_get_state();

#endif // !MISSION_CONTROL_H
