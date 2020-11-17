#include "mbed.h"

// #include "stm32l4xx_hal_sdmmc.h"
#include "stm32l4xx_hal_sd.h"

// extern SD_HandleTypeDef hsd;

extern HAL_SD_CardCIDTypeDef pCID;
extern HAL_SD_CardCSDTypeDef pCSD;
// HAL_SD_CardStatusTypeDef pStatus;
extern HAL_SD_CardInfoTypeDef pCardInfo;

void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);

HAL_StatusTypeDef sd_init();
void sd_deinit();

void sd_info();

HAL_SD_StateTypeDef sd_state();

inline bool sd_ready(){
    return sd_state() == HAL_SD_STATE_READY;
}

bool sd_write(uint32_t address, uint8_t * data, uint8_t nblocks);
bool sd_read(uint32_t address, uint8_t * data, uint8_t nblocks);

void sd_erase(uint32_t from, uint32_t to);

// void sd_access();


// void sd_test(Timer &t);