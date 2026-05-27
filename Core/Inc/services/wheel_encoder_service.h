#ifndef WHEELL_ENCODER_SERVICE_H
#define WHEELL_ENCODER_SERVICE_H

#include <stdint.h>

/**
 * Type representing the velocity of the wheels.
 * It's the absolut velocity.
 */
typedef struct {
  // The currenct velocity of the left wheel.
  // The velocity is in revolutions per minute.
  uint16_t velocity_left;
  // The current velocity of the right wheel.
  // The velocity is in revolutions per minute.
  uint16_t velocity_right;
} velocity_t;

typedef struct {
  // The travelled distance of the left wheel.
  // Moving backwards also counts towards the distance.
  // The distance is in segments of the wheel.
  uint16_t distance_left;
  // The travelled distance of the right wheel.
  // Moving backwards also counts towards the distance.
  // The distance is in segments of the wheel.
  uint16_t distance_right;
} distance_t;

/**
 * Sampling frequency for the wheel encoder.
 */
extern const uint16_t WHEEL_ENCODER_SAMPLING_PERIOD;

/**
 * Setter for the boundaries for the two schmitt triggers in use to detect
 * rising edges
 */
void wheel_encoder_set_boundaries(uint16_t left_schmitt_trigger_upper,
                                  uint16_t left_schmitt_trigger_lower,
                                  uint16_t right_schmitt_trigger_upper,
                                  uint16_t right_schmitt_trigger_lower);

/**
 * Update function for the decoder.
 * Takes the new values and updates the internally managed velocity and
 * distance. The running period is determined by
 * [WHEEL_ENCODER_SAMPLING_FREQ]
 * wheel.
 */
void wheel_encoder_update();

/**
 * Get the current velocity of both wheels
 */
velocity_t wheel_encoder_get_current_velocity();

/**
 * Get the current travelled distance for each wheel
 */
distance_t wheel_encoder_get_current_distance();

/**
 * Reset the stored velocity and distance
 */
void wheel_encoder_reset();

#endif // WHELL_ENCODER_SERVICE_H
