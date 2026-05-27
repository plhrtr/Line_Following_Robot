#include "services/sound_service.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim_ex.h"
#include "tim.h"
#include <stdint.h>

/**
 * Can't play anything higher than 500Hz and similiar notes get rounded off to
 * the same value
 */
static void play_sound(uint16_t frequency, uint32_t duration) {

  if (frequency == 0)
    return;

  // Initlaize the motors
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  TIM1->CCR2 = 32762;
  TIM1->CCR3 = 32762;

  // Calculate half period in ms
  uint32_t half_period_ms = 1000 / (frequency * 2);

  // Calculate how many cycles are needed to fill the duration
  uint32_t total_cycles = duration / (half_period_ms * 2);

  for (uint32_t i = 0; i < total_cycles; i++) {
    HAL_GPIO_WritePin(phase2_L_GPIO_Port, phase2_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(phase2_R_GPIO_Port, phase2_R_Pin, GPIO_PIN_SET);
    HAL_Delay(half_period_ms);

    HAL_GPIO_WritePin(phase2_L_GPIO_Port, phase2_L_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(phase2_R_GPIO_Port, phase2_R_Pin, GPIO_PIN_RESET);
    HAL_Delay(half_period_ms);
  }

  // Turn off the motors
  HAL_GPIO_WritePin(phase2_L_GPIO_Port, phase2_L_Pin, GPIO_PIN_SET);
  TIM1->CCR2 = 65525;
  HAL_GPIO_WritePin(phase2_R_GPIO_Port, phase2_R_Pin, GPIO_PIN_RESET);
  TIM1->CCR3 = 0;
}

#define MELODY_LENGTH 8

static const uint16_t theme_frequencies[MELODY_LENGTH] = {294, 0,   294, 0,
                                                          147, 220, 350, 500};

static const uint16_t theme_durations[MELODY_LENGTH] = {120, 120, 120, 150,
                                                        200, 200, 200, 500};

void sound_service_start_up_sound() {
  for (size_t i = 0; i < MELODY_LENGTH; i++) {
    play_sound(theme_frequencies[i], theme_durations[i]);

    play_sound(0, 50);
  }
};
