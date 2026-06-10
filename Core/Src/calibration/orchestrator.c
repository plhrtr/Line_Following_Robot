#include "calibration/orchestrator.h"
#include "calibration/calibrators/wheel_encoder_calibrator.h"
#include "mission_control/mission_control.h"
#include "scheduler.h"
#include "services/led_service.h"
#include "services/touch_sensor_service.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

const uint16_t CALIBRATION_ORCHESTRATOR_PERIOD = 2;

static uint16_t unscheduling_id;
static orchestrator_state_t current_state = CALIBRATION_NOT_STARTED;
static uint32_t orchestrator_previous_state_tick = 0;

static char touch_sensor_middle_pressed = 0;
static uint16_t touch_sensor_unsubscribe_id;

static void touch_sensor_callback() { touch_sensor_middle_pressed = 1; }

static const orchestrator_task_t tasks[] = {
    {"wheel_enc", &wheel_encoder_calibrate,
     &wheel_encoder_calibrator_get_state}};
static size_t tasks_len = sizeof(tasks) / sizeof(tasks[0]);

static uint8_t current_task_idx = 0;

void calibration_orchestrator_set_unscheduling_id(uint16_t id) {
  unscheduling_id = id;
}

void calibration_orchestrator_run() {
  switch (current_state) {
  case CALIBRATION_NOT_STARTED:
    orchestrator_previous_state_tick = HAL_GetTick();

    // Subscribte to the touch sensor to recognize middle presses
    touch_sensor_subscription_t subscription = {&touch_sensor_callback,
                                                TOUCH_SENSOR_MIDDLE};
    touch_sensor_unsubscribe_id = touch_sensor_subscribe(subscription);

    current_state = STARTING_TASK;

    break;
  case STARTING_TASK:
    if (touch_sensor_middle_pressed) {
      if (current_task_idx >= tasks_len) {
        orchestrator_previous_state_tick = HAL_GetTick();
        current_state = CALIBRATION_FINISHED;
      } else {

        orchestrator_previous_state_tick = HAL_GetTick();
        current_state = PROCCESSING_TASK;
      }
    }
    break;
  case PROCCESSING_TASK:
    calibration_state_t current_task_state =
        tasks[current_task_idx].get_state();
    if (current_task_state != FAILED && current_task_state != CALIBRATED) {
      tasks[current_task_idx].task_run();
    } else {
      if (current_task_state == FAILED) {

        led_on(LED_LEFT);
        led_off(LED_RIGHT);
      } else {

        led_on(LED_LEFT);
        led_on(LED_RIGHT);
      }

      orchestrator_previous_state_tick = HAL_GetTick();
      current_state = FINISING_TASK;
    }
    break;
  case FINISING_TASK:
    if (HAL_GetTick() - orchestrator_previous_state_tick >= 1000) {
      led_off(LED_LEFT);
      led_off(LED_RIGHT);

      current_task_idx++;
      orchestrator_previous_state_tick = HAL_GetTick();
      touch_sensor_middle_pressed = 0;
      current_state = STARTING_TASK;
    }

    break;
  case CALIBRATION_FINISHED:
    scheduler_unschedule(unscheduling_id);
    touch_sensor_unsubscribe(touch_sensor_unsubscribe_id);

    // Schedule mission control
    mission_control_init();
    task_t mission_control_task = {"mission", &mission_control_run,
                                   MISSION_CONTROL_PERIOD, 0};
    scheduler_schedule(mission_control_task);
    break;
  }
};
