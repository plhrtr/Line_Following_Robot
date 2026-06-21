#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <stdint.h>

/**
 * Defines the states the mission control can be in
 * Each state has it's own handler
 */
typedef enum {
  // Navigate in a waypoint based approach
  YELLOW_LINE_FOLLOWING,
  // Search for the line when losing it
  // Allows for different strategies (e.g. driving straight, driving backwards,
  // rotation around axis)
  LINE_SEARCHING,
  // Follow the line
  LINE_FOLLOWING,
  // Avoid a detected obstacle
  OBSTACLE_AVOIDANCE,
  // Overcome a detected gap in the route
  OVERCOME_GAP,
} mission_control_state_t;

// The scheduling period for the mission control
extern const uint16_t MISSION_CONTROL_PERIOD;

/**
 * Initialize the mission control
 */
void mission_control_init();

/**
 * Run function to schedule the mission control
 */
void mission_control_run();

/**
 * Update the state of the mission control
 * @param state - The new state you want to set the mission control to
 */
void mission_control_set_state(mission_control_state_t state);

/**
 * Get the state of the mission control
 */
mission_control_state_t mission_control_get_state();

#endif // !MISSION_CONTROL_H
