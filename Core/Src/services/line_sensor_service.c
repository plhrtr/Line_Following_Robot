#include "services/line_sensor_service.h"
#include "services/adc_service.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Line Sensor: Higher values means more on black
// TODO: Detect abrupt line ending

typedef struct {
  uint32_t left_sensor_value;
  uint32_t middle_sensor_value;
  uint32_t right_sensor_value;
} line_sensor_reading_t;

static line_sensor_reading_t last_reading;
static line_sensor_thresholds_t thresholds;
static float last_error;
static bool is_on_line;

// Threshold whether a sensor reading is considered on the line
static const float LINE_DETECTED_THRESHOLD = 0.35;

/**
 * Normalize the given value via the given thresholds
 */
static float normalize(uint32_t value, uint32_t lower, uint32_t upper) {
  // Invalid range handling
  if (upper <= lower) {
    return (value > lower) ? 1.0f : 0.0f;
  }

  // Normalize the given value as the position in [-1,1] between the given upper
  // and lower bounds
  float num = (float)(value > lower ? value - lower : 0u);
  float den = (float)(upper - lower);
  float normalized = num / den;

  // Clamp the normalized values
  if (normalized < 0.0f)
    normalized = 0.0f;
  if (normalized > 1.0f)
    normalized = 1.0f;
  return normalized;
}

static void debug_print(float error, float left_n, float middle_n,
                        float right_n) {
  char buf[50];

  /* Work around libraries without float support in printf:
     format value into integer and fractional parts, preserve sign. */
  int negative = (error < 0.0f);
  float aerror = negative ? -error : error; /* absolute value */
  int ipart = (int)aerror;
  int fpart =
      (int)((aerror - (float)ipart) * 1000.0f + 0.5f); /* 3 decimals, rounded */

  /* handle rounding carry (e.g. 0.9995 -> 1.000) */
  if (fpart >= 1000) {
    ipart += 1;
    fpart -= 1000;
  }

  int len;
  if (negative) {
    len = snprintf(buf, sizeof(buf), "-%d.%03d, %d, %d, %d\r\n", ipart, fpart,
                   (int)(left_n * 100), (int)(middle_n * 100),
                   (int)(right_n * 100));
  } else {
    len = snprintf(buf, sizeof(buf), "%d.%03d, %d, %d, %d\r\n", ipart, fpart,
                   (int)(left_n * 100), (int)(middle_n * 100),
                   (int)(right_n * 100));
  }

  if (len > 0) {
    if (len >= (int)sizeof(buf)) {
      /* output was truncated; send up to buffer size - 1 */
      len = sizeof(buf) - 1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 1000);
  }
}

/*
 * Calculate lateral error from a reading.
 */
static float calculate_error(line_sensor_reading_t reading) {
  // If the robot is not on the line max out the error in the direction the
  // robot lost the line
  if (!is_on_line) {
    return last_error < 0 ? -1.0f : 1.0f;
  }

  float left_normalized = normalize(
      reading.left_sensor_value, thresholds.left_lower, thresholds.left_upper);
  float middle_normalized =
      normalize(reading.middle_sensor_value, thresholds.middle_lower,
                thresholds.middle_upper);
  float right_normalized =
      normalize(reading.right_sensor_value, thresholds.right_lower,
                thresholds.right_upper);

  float numerator = (-2.0f * left_normalized) + (2.0f * right_normalized);
  float denominator = left_normalized + middle_normalized + right_normalized;

  if (denominator <= 0.0f) {
    return 0.0f;
  }

  // Clamp the error to [-1,1]
  float error = numerator / denominator;
  if (error < -1.0f)
    error = -1.0f;
  if (error > 1.0f)
    error = 1.0f;
  last_error = error;
  return error;
}

void line_sensor_set_thresholds(line_sensor_thresholds_t thresholds_param) {
  thresholds = thresholds_param;
}

float line_sensor_get_error() {
  line_sensor_reading_t new_reading = {
      .left_sensor_value = adc[LINE_SENSOR_LEFT],
      .middle_sensor_value = adc[LINE_SENSOR_MIDDLE],
      .right_sensor_value = adc[LINE_SENSOR_RIGHT],
  };

  /* Normalize to decide state */
  float left_n = normalize(new_reading.left_sensor_value, thresholds.left_lower,
                           thresholds.left_upper);
  float middle_n = normalize(new_reading.middle_sensor_value,
                             thresholds.middle_lower, thresholds.middle_upper);
  float right_n = normalize(new_reading.right_sensor_value,
                            thresholds.right_lower, thresholds.right_upper);

  is_on_line = (left_n > LINE_DETECTED_THRESHOLD) ||
               (middle_n > LINE_DETECTED_THRESHOLD) ||
               (right_n > LINE_DETECTED_THRESHOLD);

  last_reading = new_reading;

  float error = calculate_error(new_reading);

  /* Calculate and return error */
  return error;
}

bool line_sensor_is_on_line() { return is_on_line; }
