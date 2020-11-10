#include "mbed.h"

// #include "stm32l4xx_hal_sdmmc.h"
#include "stm32l4xx_hal_sd.h"

extern SD_HandleTypeDef hsd;

extern HAL_SD_CardCIDTypeDef pCID;
extern HAL_SD_CardCSDTypeDef pCSD;
// HAL_SD_CardStatusTypeDef pStatus;
extern HAL_SD_CardInfoTypeDef pCardInfo;

void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef sd_init();
void sd_deinit();

void sd_stat();

void sd_access();


void sd_test(Timer &t);