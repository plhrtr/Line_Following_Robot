#include "mission_control/handlers/yellow_line_following.h"
#include "mission_control/handlers/line_searching.h"
#include "mission_control/mission_control.h"
#include "services/waypoint_navigation.h"

// INFO: Subtract 40mm from the last distance for better sensor alignment at the
// end

static waypoint_navigation_task_t yellow_line_tasks[] = {{DRIVE_STRAIGHT, 425},
                                                         {TURN_RIGHT, 150},
                                                         {DRIVE_STRAIGHT, 250},
                                                         {TURN_LEFT, 45},
                                                         {DRIVE_STRAIGHT, 160}};

/**
 * Callback that gets executed when the tasks are finished
 */
static void on_finish_yellow_line_task() {
  line_searching_reset(SEARCH_LEFT);
  mission_control_set_state(LINE_SEARCHING);
}

void yellow_line_following_init() {
  waypoint_navigation_set_tasks(yellow_line_tasks, 5,
                                &on_finish_yellow_line_task);
}

/**
 * Run the yellow line following task
 */
void yellow_line_following_run() { waypoint_navigation_run(); }
