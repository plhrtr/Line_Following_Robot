#ifndef LINE_SENSOR_SERVICE_H
#define LINE_SENSOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint16_t left_lower;
  uint16_t left_upper;
  uint16_t middle_lower;
  uint16_t middle_upper;
  uint16_t right_lower;
  uint16_t right_upper;
} line_sensor_thresholds_t;

void line_sensor_set_thresholds(line_sensor_thresholds_t thresholds);

int16_t line_sensor_get_error();

bool line_sensor_check_line_lost();

bool line_sensor_check_line_ended();

#endif // !LINE_SENSOR_SERVICE_H
