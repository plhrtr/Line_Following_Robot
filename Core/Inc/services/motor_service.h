#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#define MAX_MOTOR_CORRECTION_CHANGE_RATE 10
#define MAX_MOTOR_SPEED_CHANGE_RATE 5

#include <stdint.h>

/**
 * Initialize the motors
 * Sets the current speed of both motors to zero
 */
void motors_init();

/**
 * Stop the motors
 */
void motors_stop();

/**
 * Start the motors with the specified speed
 * @param speed - The speed of the motors. Must be between [-100] and [100]
 * @return - Whether the command was executed
 */
int motors_drive_straight(int8_t speed);

/**
 * Type represention the direction of a curve
 */
typedef enum direction_t {
  STRAIGHT,
  LEFT,
  RIGHT,
} direction_t;

/**
 * Correct the current driving direction and speed
 * @param speed - The speed of the motors. Must be between [0] and [100]
 * @param sharpness - The sharpness of the curve where [0] means straight, [50]
 * means only the motor responsible for the direction and [100] means that the
 * other motor is working in the opposite direction (Rotation around current
 * position)
 * @param direction - The direction of the curve. When the direction is straight
 * the sharpness parameter is ignored
 * @return - Whether the command was executed
 *
 */
int motors_drive_curve(int8_t speed, uint8_t sharpness, direction_t direction);

#endif /// MOTOR_SERVICE_H
