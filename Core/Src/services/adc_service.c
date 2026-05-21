#include "adc.h"
#include "services/adc_serive.h"
#include "stm32l4xx_hal_adc.h"
#include <stdint.h>

static volatile int conversion_done_flag = 1;

static volatile uint32_t buffer[6];

const uint16_t ADC_SERVICE_CONVERSION_FREQ = 0;

volatile uint32_t adc[6];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
  {
    for (size_t i = 0; i < 6; i++) {
      adc[i] = buffer[i];
    }
    conversion_done_flag = 1;
  }
}

void adc_update() {
  if (conversion_done_flag) {
    HAL_ADC_Start_DMA(&hadc1, buffer, 6);
    conversion_done_flag = 0;
  }
}
