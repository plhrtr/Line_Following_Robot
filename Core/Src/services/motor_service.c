
#include "services/motor_service.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim_ex.h"
#include "tim.h"
#include <stdint.h>
#include <stdlib.h>

/**
 * Flag whether the motors have been initialized.
 */
static int motors_initialized;

static speed_t curr_speed = {0, 0};

void set_left_motor_speed(int8_t speed) {
  if (speed < -100 || speed > 100)
    return;

  curr_speed.left = speed;

  uint32_t ccr = (uint32_t)(abs(speed) * 65535 / 100);

  if (speed < 0) {
    HAL_GPIO_WritePin(phase2_L_GPIO_Port, phase2_L_Pin, GPIO_PIN_RESET);
    TIM1->CCR2 = ccr;
  } else {
    HAL_GPIO_WritePin(phase2_L_GPIO_Port, phase2_L_Pin, GPIO_PIN_SET);
    TIM1->CCR2 = 65535 - ccr;
  }
}

void set_right_motor_speed(int8_t speed) {
  if (speed < -100 || speed > 100)
    return;

  curr_speed.right = speed;

  uint32_t ccr = (uint32_t)(abs(speed) * 65535 / 100);

  if (speed < 0) {
    HAL_GPIO_WritePin(phase2_R_GPIO_Port, phase2_R_Pin, GPIO_PIN_SET);
    TIM1->CCR3 = 65535 - ccr;
  } else {
    HAL_GPIO_WritePin(phase2_R_GPIO_Port, phase2_R_Pin, GPIO_PIN_RESET);
    TIM1->CCR3 = ccr;
  }
}

void motors_init() {
  if (!motors_initialized) {
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    motors_drive_straight(0);

    motors_initialized = 1;
  }
}

void motors_stop() { motors_drive_straight(0); };

int motors_drive_straight(int8_t speed) {
  if (speed > 100 || speed < -100)
    return 0;

  set_left_motor_speed(speed);
  set_right_motor_speed(speed);

  return 1;
};

int motors_drive_curve(int8_t speed, uint8_t sharpness, direction_t direction) {
  if (speed > 100 || speed < -100 || sharpness > 100)
    return 0;

  // The reduced speed for the motor that is not responsible for the direction
  // of the curve.
  //
  // Is the linear interpolation from [speed] to [-speed] where the sharpness
  // percentage is the progress of the interpolation.
  uint16_t reduced_speed = speed * (100 - 2 * sharpness) / 100;

  switch (direction) {
  case STRAIGHT:
    set_left_motor_speed(speed);
    set_right_motor_speed(speed);
    break;
  case LEFT:
    set_left_motor_speed(reduced_speed);
    set_right_motor_speed(speed);
    break;
  case RIGHT:
    set_left_motor_speed(speed);
    set_right_motor_speed(reduced_speed);
    break;
  default:
    return 0;
  }

  return 1;
};

speed_t motors_get_speed() { return curr_speed; }
