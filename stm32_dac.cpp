#include "stm32_dac.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_gpio.h"


#define CHANNEL     DAC_CHANNEL_1
#define ALIGNMENT   DAC_ALIGN_12B_R

static DAC_HandleTypeDef mhdac;

static  bool minitialized = false;

static uint32_t mbias;

void HAL_DAC_MspInit(DAC_HandleTypeDef *hadc);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hadc);

void HAL_DAC_MspInit(DAC_HandleTypeDef *hadc){
    
    // __HAL_DAC_ENABLE(hadc, CHANNEL);

    __HAL_RCC_DAC1_CLK_ENABLE();
    


	GPIO_InitTypeDef PORT;
    
	PORT.Mode = GPIO_MODE_ANALOG; // Alternative function mode
	PORT.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //GPIO_Speed_40MHz; // High speed // GPIO_SPEED_FREQ_HIGH
	// // PORT.OType = GPIO_OType_PP; // Output push-pull
	PORT.Pull = GPIO_NOPULL;//GPIO_NOPULL;//GPIO_PULLUP; // Pull-up
    // PORT.Alternate = GPIO_AF12_SDMMC1;

    PORT.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOA, &PORT);

    // PORT.Pin = GPIO_PIN_5;
	// HAL_GPIO_Init(GPIOA, &PORT);

    // printf("analog out init\n");
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hadc){

}

void dac_init(int32_t bias){

    if (minitialized){
        dac_deinit();
    }

    mbias = bias;

    //   (+) DAC APB clock must be enabled to get write access to DAC
    //       registers using HAL_DAC_Init()
    //   (+) Configure DAC_OUTx (DAC_OUT1: PA4, DAC_OUT2: PA5) in analog mode.
    //   (+) Configure the DAC channel using HAL_DAC_ConfigChannel() function.
    //   (+) Enable the DAC channel using HAL_DAC_Start() or HAL_DAC_Start_DMA() functions.

    mhdac.Instance = DAC;

    
    HAL_StatusTypeDef status = HAL_DAC_Init(&mhdac);

    if (status != HAL_OK){
        printf("dac init: %d\n", status);
        while(1);
    }
    
    DAC_ChannelConfTypeDef sConfig;

#if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined(STM32L4S9xx)
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;        
  /*!< Specifies the frequency interface mode
                                              This parameter can be a value of @ref DAC_HighFrequency */
#endif /* STM32L4R5xx STM32L4R7xx STM32L4R9xx STM32L4S5xx STM32L4S7xx STM32L4S9xx */

  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;            
  /*!< Specifies whether the DAC mode.
                                              This parameter can be a value of @ref DAC_SampleAndHold */

  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;//DAC_TRIGGER_NONE;//DAC_TRIGGER_NONE;//DAC_TRIGGER_SOFTWARE;                  
  /*!< Specifies the external trigger for the selected DAC channel.
                                              This parameter can be a value of @ref DAC_trigger_selection */

  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;             
  /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                               This parameter can be a value of @ref DAC_output_buffer */

  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE; 
  /*!< Specifies whether the DAC output is connected or not to on chip peripheral .
                                              This parameter can be a value of @ref DAC_ConnectOnChipPeripheral */

  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;             
  /*!< Specifies the trimming mode
                                              This parameter must be a value of @ref DAC_UserTrimming
                                              DAC_UserTrimming is either factory or user trimming */

  sConfig.DAC_TrimmingValue = 1;             
  /*!< Specifies the offset trimming value
                                               i.e. when DAC_SampleAndHold is DAC_TRIMMING_USER.
                                               This parameter must be a number between Min_Data = 1 and Max_Data = 31 */


  /*!< Specifies the Sample time for the selected channel.
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 1023 */

  sConfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 0;  


              /*!< Specifies the hold time for the selected channel
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 1023 */

  sConfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 0;  

           /*!< Specifies the refresh time for the selected channel
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 255 */

  sConfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 0;  
    /*!< Sample and Hold settings */


    status = HAL_DAC_ConfigChannel(&mhdac, &sConfig, CHANNEL);


    if (status != HAL_OK){
        printf("dac config: %d\n", status);
        while(1);
    }

    status = HAL_DACEx_SelfCalibrate(&mhdac, &sConfig, CHANNEL);

    if (status != HAL_OK){
        printf("dac calib: %d\n", status);
        while(1);
    }

    minitialized = true;
}

void dac_deinit(){
    if (!minitialized){
        return;
    }

    HAL_StatusTypeDef status = HAL_DAC_DeInit(&mhdac);

    if (status != HAL_OK){
        printf("dac deinit: %d\n", status);
        while(1);
    }

    minitialized = false;
}

void dac_start(){
    
    if (!minitialized){
        printf("dac not yet initialized\n");
        while(1);
    }

    HAL_StatusTypeDef status = HAL_DAC_Start(&mhdac, CHANNEL);

    if (status != HAL_OK){
        printf("dac start: %d\n", status);
        while(1);
    }
}

void dac_stop(){

    if (!minitialized){
        return;
    }

    HAL_StatusTypeDef status = HAL_DAC_Stop(&mhdac, CHANNEL);

    if (status != HAL_OK){
        printf("dac stop: %d\n", status);
        while(1);
    }
}

void dac_write(int32_t sample){

    if (!minitialized){
        return;
    }

    int32_t biased_sample = (sample + mbias);


    HAL_StatusTypeDef status = HAL_DAC_SetValue(&mhdac, CHANNEL, ALIGNMENT, biased_sample);


    if (status != HAL_OK){
        // printf("dac write: %d\n", status);
        while(1);
    }

}
