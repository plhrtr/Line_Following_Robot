#include "mission_control/handlers/obstacle_avoidance.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/handlers/waypoint_navigation.h"
#include "mission_control/mission_control.h"

// List of waypoints to avoid a detected object
static waypoint_navigation_task_t obstacle_avoidance_tasks[] = {
    {DRIVE_BACKWARDS, 30}, {TURN_RIGHT, 45},      {DRIVE_STRAIGHT, 220},
    {TURN_LEFT, 80},       {DRIVE_STRAIGHT, 260}, {TURN_RIGHT, 35},
};

/**
 * Callback that gets executed when the waypoint navigation finishes
 */
static void on_finish_obstacle_avoidance() {
  waypoint_navigation_set_default();
  line_searching_reset(SEARCH_LEFT);
  mission_control_set_state(LINE_SEARCHING);
}

void obstacle_avoidance_run() {
  waypoint_navigation_set_tasks(obstacle_avoidance_tasks,
                                sizeof(obstacle_avoidance_tasks) /
                                    sizeof(waypoint_navigation_task_t),
                                &on_finish_obstacle_avoidance);
  mission_control_set_state(WAYPOINT_NAVIGATION);
}
