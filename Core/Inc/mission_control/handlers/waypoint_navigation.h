#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <stdint.h>

typedef enum {
  DRIVE_STRAIGHT,
  DRIVE_BACKWARDS,
  TURN_LEFT,
  TURN_RIGHT
} waypoint_navigation_task_type_t;

typedef struct {
  /* The type of the task */
  waypoint_navigation_task_type_t type;
  /* The value for the given task (distance in mm or angle in degrees) */
  uint16_t value;
  // The goal distance for the left wheel
  uint32_t goal_distance_left;
  // The goal distance for the right wheel
  uint32_t goal_distance_right;
} waypoint_navigation_task_t;

// Run function for the waypoint navigation
void waypoint_navigation_run();

// Reset the way point navigation
void waypoint_navigation_reset();

// Set the waypoint navigation to the default task
void waypoint_navigation_set_default();

// Set the tasks the waypoint navigator should execute
// Also resets the current run
void waypoint_navigation_set_tasks(waypoint_navigation_task_t *new_tasks,
                                   uint8_t number_of_tasks,
                                   void (*on_finish)());

#endif // !WAYPOINT_NAVIGATION_H
