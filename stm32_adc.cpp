#include "stm32_adc.h"

#include "PeripheralNames.h"
#include "stm32_hal_legacy.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"

#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_gpio_ex.h"

static bool initialized = false;

ADC_HandleTypeDef hadc;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc){

    // (#) Enable the ADC interface
    //     (++) As prerequisite, ADC clock must be configured at RCC top level.

    // __HAL_RCC_ADC_CLK_ENABLE();

    //     (++) Two clock settings are mandatory:
    //          (+++) ADC clock (core clock, also possibly conversion clock).

    //          (+++) ADC clock (conversions clock).
    //                Two possible clock sources: synchronous clock derived from APB clock
    //                or asynchronous clock derived from system clock, PLLSAI1 or the PLLSAI2
    //                running up to 80MHz.

    //          (+++) Example:
    //                Into HAL_ADC_MspInit() (recommended code location) or with
    //                other device clock parameters configuration:
    //            (+++) __HAL_RCC_ADC_CLK_ENABLE();                  (mandatory)
    __HAL_RCC_ADC_CLK_ENABLE();

    //            RCC_ADCCLKSOURCE_PLL enable:                       (optional: if asynchronous clock selected)
    //            (+++) RCC_PeriphClkInitTypeDef   RCC_PeriphClkInit;
    //            (+++) PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    //            (+++) PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_PLL;
    //            (+++) HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);

    // RCC_PeriphCLKInitTypeDef   RCC_PeriphClkInit;

    // HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);

    // RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    // RCC_PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_PLLSAI1;//RCC_ADCCLKSOURCE_SYSCLK;

    // HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    //     (++) ADC clock source and clock prescaler are configured at ADC level with
    //          parameter "ClockPrescaler" using function HAL_ADC_Init().

    // (#) ADC pins configuration
    //      (++) Enable the clock for the ADC GPIOs
    //           using macro __HAL_RCC_GPIOx_CLK_ENABLE()

    __HAL_RCC_GPIOA_CLK_ENABLE();

    //      (++) Configure these ADC pins in analog mode
    //           using function HAL_GPIO_Init()

	GPIO_InitTypeDef PORT;
    
	PORT.Mode = GPIO_MODE_ANALOG_ADC_CONTROL; // Alternative function mode
	PORT.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //GPIO_Speed_40MHz; // High speed // GPIO_SPEED_FREQ_HIGH
	// // PORT.OType = GPIO_OType_PP; // Output push-pull
	PORT.Pull = GPIO_NOPULL;//GPIO_NOPULL;//GPIO_PULLUP; // Pull-up
    // PORT.Alternate = GPIO_AF12_SDMMC1;

    PORT.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOA, &PORT);

    PORT.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOA, &PORT);


    // (#) Optionally, in case of usage of ADC with interruptions:
    //      (++) Configure the NVIC for ADC
    //           using function HAL_NVIC_EnableIRQ(ADCx_IRQn)
    //      (++) Insert the ADC interruption handler function HAL_ADC_IRQHandler()
    //           into the function of corresponding ADC interruption vector
    //           ADCx_IRQHandler().

    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    // (#) Optionally, in case of usage of DMA:
    //      (++) Configure the DMA (DMA channel, mode normal or circular, ...)
    //           using function HAL_DMA_Init().
    //      (++) Configure the NVIC for DMA
    //           using function HAL_NVIC_EnableIRQ(DMAx_Channelx_IRQn)
    //      (++) Insert the ADC interruption handler function HAL_ADC_IRQHandler()
    //           into the function of corresponding DMA interruption vector
    //           DMAx_Channelx_IRQHandler().
    // printf("msp_adc_init: end\n");
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc){

    // (#) Disable the ADC interface
    //   (++) ADC clock can be hard reset and disabled at RCC top level.
    //     (++) Hard reset of ADC peripherals
    //          using macro __ADCx_FORCE_RESET(), __ADCx_RELEASE_RESET().
    
    // __ADCx_FORCE_RESET();

    //     (++) ADC clock disable
    //          using the equivalent macro/functions as configuration step.
    //          (+++) Example:
    //                Into HAL_ADC_MspDeInit() (recommended code location) or with
    //                other device clock parameters configuration:
    //            (+++) RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI14;
    //            (+++) RCC_OscInitStructure.HSI14State = RCC_HSI14_OFF; (if not used for system clock)
    //            (+++) HAL_RCC_OscConfig(&RCC_OscInitStructure);

    __HAL_RCC_ADC_CLK_DISABLE();

    // (#) ADC pins configuration
    //      (++) Disable the clock for the ADC GPIOs
    //           using macro __HAL_RCC_GPIOx_CLK_DISABLE()

    // (#) Optionally, in case of usage of ADC with interruptions:
    //      (++) Disable the NVIC for ADC
    //           using function HAL_NVIC_EnableIRQ(ADCx_IRQn)

    // (#) Optionally, in case of usage of DMA:
    //      (++) Deinitialize the DMA
    //           using function HAL_DMA_Init().
    //      (++) Disable the NVIC for DMA
    //           using function HAL_NVIC_EnableIRQ(DMAx_Channelx_IRQn)

}

void adc_init(){

    hadc.Instance = ADC2;

    /////////////////////////////////////////////////////
    // (#) Configure the ADC parameters (resolution, data alignment, ...)
    //     and regular group parameters (conversion trigger, sequencer, ...)
    //     using function HAL_ADC_Init().


    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
            /*!< Select ADC clock source (synchronous clock derived from APB clock or asynchronous clock derived from system clock or PLL (Refer to reference manual for list of clocks available)) and clock prescaler.
                                       This parameter can be a value of @ref ADC_HAL_EC_COMMON_CLOCK_SOURCE.
                                       Note: The ADC clock configuration is common to all ADC instances.
                                       Note: In case of usage of channels on injected group, ADC frequency should be lower than AHB clock frequency /4 for resolution 12 or 10 bits,
                                             AHB clock frequency /3 for resolution 8 bits, AHB clock frequency /2 for resolution 6 bits.
                                       Note: In case of synchronous clock mode based on HCLK/1, the configuration must be enabled only
                                             if the system clock has a 50% duty clock cycle (APB prescaler configured inside RCC
                                             must be bypassed and PCLK clock must have 50% duty cycle). Refer to reference manual for details.
                                       Note: In case of usage of asynchronous clock, the selected clock must be preliminarily enabled at RCC top level.
                                       Note: This parameter can be modified only if all ADC instances are disabled. */

  hadc.Init.Resolution = ADC_RESOLUTION_12B;
              /*!< Configure the ADC resolution.
                                       This parameter can be a value of @ref ADC_HAL_EC_RESOLUTION */

  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
               /*!< Specify ADC data alignment in conversion data register (right or left).
                                       Refer to reference manual for alignments formats versus resolutions.
                                       This parameter can be a value of @ref ADC_HAL_EC_DATA_ALIGN */

  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
            /*!< Configure the sequencer of ADC groups regular and injected.
                                       This parameter can be associated to parameter 'DiscontinuousConvMode' to have main sequence subdivided in successive parts.
                                       If disabled: Conversion is performed in single mode (one channel converted, the one defined in rank 1).
                                                    Parameters 'NbrOfConversion' and 'InjectedNbrOfConversion' are discarded (equivalent to set to 1).
                                       If enabled:  Conversions are performed in sequence mode (multiple ranks defined by 'NbrOfConversion' or 'InjectedNbrOfConversion' and rank of each channel in sequencer).
                                                    Scan direction is upward: from rank 1 to rank 'n'.
                                       This parameter can be a value of @ref ADC_Scan_mode */

  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
            /*!< Specify which EOC (End Of Conversion) flag is used for conversion by polling and interruption: end of unitary conversion or end of sequence conversions.
                                       This parameter can be a value of @ref ADC_EOCSelection. */

  hadc.Init.LowPowerAutoWait = DISABLE;
   /*!< Select the dynamic low power Auto Delay: new conversion start only when the previous
                                       conversion (for ADC group regular) or previous sequence (for ADC group injected) has been retrieved by user software,
                                       using function HAL_ADC_GetValue() or HAL_ADCEx_InjectedGetValue().
                                       This feature automatically adapts the frequency of ADC conversions triggers to the speed of the system that reads the data. Moreover, this avoids risk of overrun
                                       for low frequency applications.
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: Do not use with interruption or DMA (HAL_ADC_Start_IT(), HAL_ADC_Start_DMA()) since they clear immediately the EOC flag
                                             to free the IRQ vector sequencer.
                                             Do use with polling: 1. Start conversion with HAL_ADC_Start(), 2. Later on, when ADC conversion data is needed:
                                             use HAL_ADC_PollForConversion() to ensure that conversion is completed and HAL_ADC_GetValue() to retrieve conversion result and trig another conversion start.
                                             (in case of usage of ADC group injected, use the equivalent functions HAL_ADCExInjected_Start(), HAL_ADCEx_InjectedGetValue(), ...). */

  hadc.Init.ContinuousConvMode = ENABLE;
   /*!< Specify whether the conversion is performed in single mode (one conversion) or continuous mode for ADC group regular,
                                       after the first ADC conversion start trigger occurred (software start or external trigger).
                                       This parameter can be set to ENABLE or DISABLE. */

  hadc.Init.NbrOfConversion = 1;
         /*!< Specify the number of ranks that will be converted within the regular group sequencer.
                                       To use the regular group sequencer and convert several ranks, parameter 'ScanConvMode' must be enabled.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 16.
                                       Note: This parameter must be modified when no conversion is on going on regular group (ADC disabled, or ADC enabled without
                                       continuous mode or external trigger that could launch a conversion). */

  hadc.Init.DiscontinuousConvMode = DISABLE;
   /*!< Specify whether the conversions sequence of ADC group regular is performed in Complete-sequence/Discontinuous-sequence
                                       (main sequence subdivided in successive parts).
                                       Discontinuous mode is used only if sequencer is enabled (parameter 'ScanConvMode'). If sequencer is disabled, this parameter is discarded.
                                       Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded.
                                       This parameter can be set to ENABLE or DISABLE. */

  hadc.Init.NbrOfDiscConversion = 1;  
   /*!< Specifies the number of discontinuous conversions in which the main sequence of ADC group regular (parameter NbrOfConversion) will be subdivided.
                                       If parameter 'DiscontinuousConvMode' is disabled, this parameter is discarded.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 8. */

  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;//ADC_SOFTWARE_START;//ADC_EXTERNALTRIG_T6_TRGO;//ADC_SOFTWARE_START;     
   /*!< Select the external event source used to trigger ADC group regular conversion start.
                                       If set to ADC_SOFTWARE_START, external triggers are disabled and software trigger is used instead.
                                       This parameter can be a value of @ref ADC_regular_external_trigger_source.
                                       Caution: external trigger source is common to all ADC instances. */

  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; 
   /*!< Select the external event edge used to trigger ADC group regular conversion start.
                                       If trigger source is set to ADC_SOFTWARE_START, this parameter is discarded.
                                       This parameter can be a value of @ref ADC_regular_external_trigger_edge */

  hadc.Init.DMAContinuousRequests = DISABLE; 
  /*!< Specify whether the DMA requests are performed in one shot mode (DMA transfer stops when number of conversions is reached)
                                       or in continuous mode (DMA transfer unlimited, whatever number of conversions).
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: In continuous mode, DMA must be configured in circular mode. Otherwise an overrun will be triggered when DMA buffer maximum pointer is reached. */

  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;  
               /*!< Select the behavior in case of overrun: data overwritten or preserved (default).
                                       This parameter applies to ADC group regular only.
                                       This parameter can be a value of @ref ADC_HAL_EC_REG_OVR_DATA_BEHAVIOR.
                                       Note: In case of overrun set to data preserved and usage with programming model with interruption (HAL_Start_IT()): ADC IRQ handler has to clear
                                       end of conversion flags, this induces the release of the preserved data. If needed, this data can be saved in function
                                       HAL_ADC_ConvCpltCallback(), placed in user program code (called before end of conversion flags clear).
                                       Note: Error reporting with respect to the conversion mode:
                                             - Usage with ADC conversion by polling for event or interruption: Error is reported only if overrun is set to data preserved. If overrun is set to data
                                               overwritten, user can willingly not read all the converted data, this is not considered as an erroneous case.
                                             - Usage with ADC conversion by DMA: Error is reported whatever overrun setting (DMA is expected to process all data from data register). */

  hadc.Init.OversamplingMode = ENABLE;      
   /*!< Specify whether the oversampling feature is enabled or disabled.
                                               This parameter can be set to ENABLE or DISABLE.
                                               Note: This parameter can be modified only if there is no conversion is ongoing on ADC groups regular and injected */

  hadc.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;                         /*!< Configures the oversampling ratio.
                                               This parameter can be a value of @ref ADC_HAL_EC_OVS_RATIO */

  hadc.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;                 /*!< Configures the division coefficient for the Oversampler.
                                               This parameter can be a value of @ref ADC_HAL_EC_OVS_SHIFT */

  hadc.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;          
         /*!< Selects the regular triggered oversampling mode.
                                               This parameter can be a value of @ref ADC_HAL_EC_OVS_DISCONT_MODE */

  hadc.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;    
       /*!< Selects the regular oversampling mode.
                                               The oversampling is either temporary stopped or reset upon an injected
                                               sequence interruption.
                                               If oversampling is enabled on both regular and injected groups, this parameter
                                               is discarded and forced to setting "ADC_REGOVERSAMPLING_RESUMED_MODE"
                                               (the oversampling buffer is zeroed during injection sequence).
                                               This parameter can be a value of @ref ADC_HAL_EC_OVS_SCOPE_REG */

   /*!< Specify the Oversampling parameters.
                                               Caution: this setting overwrites the previous oversampling configuration if oversampling is already enabled. */

#if defined(ADC_CFGR_DFSDMCFG) &&defined(DFSDM1_Channel0)
//   hadc.Init.DFSDMConfig;         
    /*!< Specify whether ADC conversion data is sent directly to DFSDM.
                                       This parameter can be a value of @ref ADC_HAL_EC_REG_DFSDM_TRANSFER.
                                       Note: This parameter can be modified only if there is no conversion is ongoing (both ADSTART and JADSTART cleared). */

#endif

    HAL_StatusTypeDef status = HAL_ADC_Init(&hadc);

    if (HAL_OK != status){
        printf("HAL error: %d\n", status);
        while(1);
    }

    // printf("init\n");

    ////////////////////////////////////////////////////////////////////////////////
    // (#) Configure the channels for regular group parameters (channel number,
    //     channel rank into sequencer, ..., into regular group)
    //     using function HAL_ADC_ConfigChannel().
    ADC_ChannelConfTypeDef sConfig;


  sConfig.Channel = ADC_CHANNEL_5;               
   /*!< Specify the channel to configure into ADC regular group.
                                        This parameter can be a value of @ref ADC_HAL_EC_CHANNEL
                                        Note: Depending on devices and ADC instances, some channels may not be available on device package pins. Refer to device datasheet for channels availability. */

  sConfig.Rank = ADC_REGULAR_RANK_1;                  
   /*!< Specify the rank in the regular group sequencer.
                                        This parameter can be a value of @ref ADC_HAL_EC_REG_SEQ_RANKS
                                        Note: to disable a channel or change order of conversion sequencer, rank containing a previous channel setting can be overwritten by
                                        the new channel setting (or parameter number of conversions adjusted) */

  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;          
   /*!< Sampling time value to be set for the selected channel.
                                        Unit: ADC clock cycles
                                        Conversion time is the addition of sampling time and processing time
                                        (12.5 ADC clock cycles at ADC resolution 12 bits, 10.5 cycles at 10 bits, 8.5 cycles at 8 bits, 6.5 cycles at 6 bits).
                                        This parameter can be a value of @ref ADC_HAL_EC_CHANNEL_SAMPLINGTIME
                                        Caution: This parameter applies to a channel that can be used into regular and/or injected group.
                                                 It overwrites the last setting.
                                        Note: In case of usage of internal measurement channels (VrefInt/Vbat/TempSensor),
                                              sampling time constraints must be respected (sampling time can be adjusted in function of ADC clock frequency and sampling time setting)
                                              Refer to device datasheet for timings values. */

  sConfig.SingleDiff = LL_ADC_SINGLE_ENDED;//LL_ADC_DIFFERENTIAL_ENDED;            
   /*!< Select single-ended or differential input.
                                        In differential mode: Differential measurement is carried out between the selected channel 'i' (positive input) and channel 'i+1' (negative input).
                                                              Only channel 'i' has to be configured, channel 'i+1' is configured automatically.
                                        This parameter must be a value of @ref ADC_HAL_EC_CHANNEL_SINGLE_DIFF_ENDING
                                        Caution: This parameter applies to a channel that can be used in a regular and/or injected group.
                                                 It overwrites the last setting.
                                        Note: Refer to Reference Manual to ensure the selected channel is available in differential mode.
                                        Note: When configuring a channel 'i' in differential mode, the channel 'i+1' is not usable separately.
                                        Note: This parameter must be modified when ADC is disabled (before ADC start conversion or after ADC stop conversion).
                                              If ADC is enabled, this parameter setting is bypassed without error reporting (as it can be the expected behavior in case
                                        of another parameter update on the fly) */

  sConfig.OffsetNumber = ADC_OFFSET_NONE;          
   /*!< Select the offset number
                                        This parameter can be a value of @ref ADC_HAL_EC_OFFSET_NB
                                        Caution: Only one offset is allowed per channel. This parameter overwrites the last setting. */

  sConfig.Offset = 0;                
   /*!< Define the offset to be subtracted from the raw converted data.
                                        Offset value must be a positive number.
                                        Depending of ADC resolution selected (12, 10, 8 or 6 bits), this parameter must be a number between Min_Data = 0x000 and Max_Data = 0xFFF,
                                        0x3FF, 0xFF or 0x3F respectively.
                                        Note: This parameter must be modified when no conversion is on going on both regular and injected groups (ADC disabled, or ADC enabled
                                              without continuous mode or external trigger that could launch a conversion). */



    status = HAL_ADC_ConfigChannel(&hadc, &sConfig);

    if (HAL_OK != status){
        printf("configchannel error: %d\n", status);
        while(1);
    }

    // (#) Optionally, configure the analog watchdog parameters (channels
    //     monitored, thresholds, ...)
    //     using function HAL_ADC_AnalogWDGConfig().



    // HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);
    // HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
    // HAL_SYSCFG_EnableVREFBUF();

    status = HAL_ADCEx_Calibration_Start(&hadc, LL_ADC_SINGLE_ENDED);//LL_ADC_DIFFERENTIAL_ENDED);

    if (status != HAL_OK)
    {
        printf("calib error: %d\n", status);
        while(1);
    }

    initialized = true;
}

void adc_deinit(){
    if (!initialized){
        return;
    }

    HAL_ADC_DeInit(&hadc);
}

void adc_start(){
    if (!initialized){
        printf("adc not yet initialized\n");
        while(1);
    }

    HAL_StatusTypeDef status = HAL_ADC_Start(&hadc);//HAL_ADC_Start(&hadc);

    if (status != HAL_OK){
        printf("hal start error: %d\n", status);
        while(1);
    }
    
}

void adc_stop(){
    if (!initialized){
        return;
    }

    HAL_StatusTypeDef status = HAL_ADC_Stop(&hadc);

    if (status != HAL_OK){
        printf("hal stop error: %d\n", status);
        while(1);
    }
}

uint32_t adc_read(){
    if (!initialized){
        return 0;
    }
    return HAL_ADC_GetValue(&hadc) & 0x0fff;

	// uint16_t value = 0;

	// if (HAL_ADC_Start(&hadc) != HAL_OK)
	// {
	//    return HAL_ERROR;
	// }

	// if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	// if((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) !=  HAL_ADC_STATE_REG_EOC)
	// {
	// 	return HAL_ERROR;
	// }

	// value = HAL_ADC_GetValue(&hadc);

	// return value&0x0fff;// 12 bit result
}