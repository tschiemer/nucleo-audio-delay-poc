#include "audiotimer6.h"

// TIM6 handle
TIM_HandleTypeDef audiotimer6;

// actual timer trigger-rate
static volatile uint32_t samplerate;




static void audiotimer6_interrupt(void);

uint32_t audiotimer6_get_samplerate(void){
    return samplerate;
}

__weak void audiotimer6_callback(void){
    UNUSED(samplerate);
}

void audiotimer6_interrupt(void) {
    if (__HAL_TIM_GET_FLAG(&audiotimer6, TIM_FLAG_UPDATE) != RESET) {
        // if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&audiotimer6, TIM_IT_UPDATE);
            // __HAL_TIM_CLEAR_FLAG(&audiotimer, TIM_FLAG_UPDATE);

            audiotimer6_callback();
        }
    }
}

void audiotimer6_init(uint32_t samplerate_hz){

     __TIM6_CLK_ENABLE();

    // PCLK1 is the clock connected to TIM6
    // if (HAL_RCC_GetPCLK1Freq() != 80000000){
    //     printf("ERROR: timer setup assuming PLCK1 frequency of 80MHz, please adjust prescaler/period\n");
    //     while(1);
    // }


    audiotimer6.Instance = TIM6;

    /*!< Specifies the prescaler value used to divide the TIM clock.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

    /*!< Specifies the period value to be loaded into the active
                                    Auto-Reload Register at the next update event.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

    // Compute the prescaler value so TIM6 triggers at freq-Hz
    uint32_t period = HAL_RCC_GetPCLK1Freq()/samplerate_hz;  
    uint32_t prescaler = 1;
    while (period > 0xffff) {
        period >>= 1;
        prescaler <<= 1;
    }

    samplerate = HAL_RCC_GetPCLK1Freq() / (period * prescaler);

    //printf("target sr = %d\n", samplerate_hz);
    //printf("actual sr = %d\n", samplerate);


    // printf("psc = %d\n", prescaler);
    // printf("arr = %d\n",period);

    audiotimer6.Init.Period = period - 1;
    audiotimer6.Init.Prescaler = prescaler - 1;

    audiotimer6.Init.CounterMode = TIM_COUNTERMODE_UP;       
    /*!< Specifies the counter mode.
                                    This parameter can be a value of @ref TIM_Counter_Mode */

    

    audiotimer6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     
    /*!< Specifies the clock division.
                                    This parameter can be a value of @ref TIM_ClockDivision */

    audiotimer6.Init.RepetitionCounter = 0;
    /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                        GP timers: this parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.
                                        Advanced timers: this parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

    audiotimer6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    /*!< Specifies the auto-reload preload.
                                    This parameter can be a value of @ref TIM_AutoReloadPreload */

    HAL_StatusTypeDef status = HAL_TIM_Base_Init(&audiotimer6);

    if (status != HAL_OK){
        printf("ERR tim6 init: %d\n", status);
        while(1);
    }

    TIM_MasterConfigTypeDef sMasterConfig;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;   
    /*!< Trigger output (TRGO) selection
                                        This parameter can be a value of @ref TIM_Master_Mode_Selection */
    //sMasterConfig.MasterOutputTrigger2;
    /*!< Trigger output2 (TRGO2) selection
                                        This parameter can be a value of @ref TIM_Master_Mode_Selection_2 */
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;       
    /*!< Master/slave mode selection
                                        This parameter can be a value of @ref TIM_Master_Slave_Mode */

    status = HAL_TIMEx_MasterConfigSynchronization(&audiotimer6,&sMasterConfig);

    if (status != HAL_OK){
        printf("ERR tim6 config: %d\n", status);
        while(1);
    }

    NVIC_SetVector(TIM6_DAC_IRQn, (uint32_t)audiotimer6_interrupt);

    // __HAL_TIM_ENABLE_IT(&audiotimer, TIM_IT_UPDATE);
}

void audiotimer6_start(){
    HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&audiotimer6);

    if (status != HAL_OK){
        printf("ERR tim6 start: %d\n", status);
        while(1);
    }

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void audiotimer6_stop(){

    NVIC_DisableIRQ(TIM6_DAC_IRQn);

    HAL_StatusTypeDef status = HAL_TIM_Base_Stop(&audiotimer6);

    if (status != HAL_OK){
        printf("ERR tim6 start: %d\n", status);
        while(1);
    }
}


