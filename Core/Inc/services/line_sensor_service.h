#ifndef LINE_SENSOR_SERVICE_H
#define LINE_SENSOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t left_lower;
  uint32_t left_upper;
  uint32_t middle_lower;
  uint32_t middle_upper;
  uint32_t right_lower;
  uint32_t right_upper;
} line_sensor_thresholds_t;

/**
 * Set the threshold for the line sensor
 */
void line_sensor_set_thresholds(line_sensor_thresholds_t thresholds);

/**
 * Get the derivation of the line sensors reading from being perfectly on the
 * line.
 * Also updates the value for the is on line check
 *
 * A negative value means the robot is to far on the right.
 * A positive value means the robot is to far on the left.
 */
float line_sensor_get_error();

/**
 * Check whether the robot lost the line
 */
bool line_sensor_is_on_line();

/**
 * Check whether a sharp left turn has been detected
 */
bool line_sensor_sharp_left();

/**
 * Check whether a sharp right turn has be detected
 */
bool line_sensor_sharp_right();

#endif // !LINE_SENSOR_SERVICE_H
