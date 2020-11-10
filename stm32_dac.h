#include "mbed.h"

#include "stm32l4xx_hal_dac.h"
#include "stm32l4xx_hal_dac_ex.h"

extern DAC_HandleTypeDef hdac;

void HAL_DAC_MspInit(DAC_HandleTypeDef *hadc);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hadc);

void dac_init();
void dac_deinit();

void dac_start();
void dac_stop();

void dac_write(uint32_t sample);