#include <mbed.h>


#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_tim_ex.h"

void audiotimer6_init(uint32_t samplerate_hz);

void audiotimer6_start();
void audiotimer6_stop();

uint32_t audiotimer6_get_samplerate(void);

void audiotimer6_callback(void);