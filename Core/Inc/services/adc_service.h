#ifndef ADC_SERVICE_H
#define ADC_SERVICE_H

#include <stdint.h>

/**
 * Indices for the adc conversions array
 */
typedef enum {
  LINE_SENSOR_MIDDLE,
  ENCODER_LEFT,
  LINE_SENSOR_RIGHT,
  BATTERY,
  ENCODER_RIGHT,
  LINE_SENSOR_LEFT,
} adc_indeces_t;

extern const uint16_t ADC_SERVICE_CONVERSION_PERIOD;

/**
 * @brief Buffer containing the conversions of the ADC.
 *
 * Indices:
 *   * 0 - lineSensor_middle
 *   * 1 - encoder_left
 *   * 2 - lineSensor_right
 *   * 3 - battery
 *   * 4 - encoder_right
 *   * 5 - lineSensor_left
 */
extern volatile uint32_t adc[6];

/**
 * Update function for the ADC unit.
 * The running frequency is determined by [ADC_SERVICE_CONVERSION_FREQ]
 */
void adc_update();

#endif // ADC_SERVICE_H
