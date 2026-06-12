#include "services/adc_service.h"
#include "adc.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>

const uint16_t ADC_SERVICE_CONVERSION_PERIOD = 2;

static char conversion_done_flag = 1;

static volatile uint32_t buffer[6];

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
    conversion_done_flag = 0;
    HAL_ADC_Start_DMA(&hadc1, buffer, 6);

    // char buf[100];
    // int len = sprintf(buf, "%lu\n", adc[BATTERY]);
    // HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 1000000);
  }
}
