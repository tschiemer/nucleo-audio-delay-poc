#include "mbed.h"

#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_adc_ex.h"

// extern ADC_HandleTypeDef hadc;

// void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);
// void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);

void adc_init(int32_t bias, bool differential);
void adc_deinit();

void adc_start();
void adc_stop();

int32_t adc_read();