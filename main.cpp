#include "arm_hal_interrupt_private.h"
#include "mbed.h"

// #include "stm32_sdio.h"
#include "mbed_wait_api.h"
#include "stm32_adc.h"
#include "stm32_dac.h"
#include "stm32l476xx.h"

#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_tim_ex.h"
#include <cstdint>

#define SAMPLERATE_HZ 48000
#define DELAY_SAMPLES (SAMPLERATE_HZ>>2)

// typedef enum {
//     SampleRate_48k,
//     SampleRate_96k
// } SampleRate_t;

using namespace std::chrono;

Ticker worker;

InterruptIn cd(PA_9, PullUp);
DigitalOut led(LED1);

volatile uint32_t sample = 0;

TIM_HandleTypeDef audiotimer;
uint32_t samplerate;


uint16_t delayline[DELAY_SAMPLES];
volatile uint32_t delaylinei = 0;

// DigitalOut led(LED1);

// AnalogIn   ain_1(PB_0);
// AnalogIn   ain_2(PB_1);
// AnalogOut  aout_1(PA_4);
// AnalogOut  aout_2(PA_5);

// AnalogOut dac(PA_4);

// PwmOut pwm(PA_8);


Timer t;

void clock_init(){
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    uint32_t pFLatency;


    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    // if uncommented we get a sysclk of 48MHz which is perfect to get a a precise TIM6 interrupt at 48kHz ..
    //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency);

    printf("SYSCLKSource = %d (MSI = %d, HSI = %d, HSE = %d, PLL = %d)\n", RCC_ClkInitStruct.SYSCLKSource, RCC_SYSCLKSOURCE_MSI, RCC_SYSCLKSOURCE_HSI, RCC_SYSCLKSOURCE_HSE, RCC_SYSCLKSOURCE_PLLCLK);
    // printf("AHBCLKDivider = %d\n", RCC_ClkInitStruct.AHBCLKDivider);
    // printf("APB1CLKDivider = %d\n", RCC_ClkInitStruct.APB1CLKDivider);
    // printf("APB2CLKDivider = %d\n", RCC_ClkInitStruct.APB2CLKDivider);


    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    printf("sysclk = %d\n", sysclk);
    printf("hclk = %d\n", hclk);
    printf("pclk1 = %d\n", pclk1);
    printf("pclk2 = %d\n", pclk2);
}



void audiotimer_interrupt(void) {
    if (__HAL_TIM_GET_FLAG(&audiotimer, TIM_FLAG_UPDATE) != RESET) {
        // if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&audiotimer, TIM_IT_UPDATE);
            // __HAL_TIM_CLEAR_FLAG(&audiotimer, TIM_FLAG_UPDATE);


            sample = adc_read();

            delayline[delaylinei] = sample;

            delaylinei = (delaylinei + 1) % DELAY_SAMPLES;

            uint32_t mix = (sample + delayline[delaylinei]) / 2;

            dac_write(sample);




            static uint32_t c = 0;

            if (++c >= SAMPLERATE_HZ){
                c = 0;
                //led = !led;
            } 

        }
    }
}

void audiotimer_init(uint32_t samplerate_hz){


     __TIM6_CLK_ENABLE();

    // PCLK1 is the clock connected to TIM6
    // if (HAL_RCC_GetPCLK1Freq() != 80000000){
    //     printf("ERROR: timer setup assuming PLCK1 frequency of 80MHz, please adjust prescaler/period\n");
    //     while(1);
    // }


    audiotimer.Instance = TIM6;

    /*!< Specifies the prescaler value used to divide the TIM clock.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

    /*!< Specifies the period value to be loaded into the active
                                    Auto-Reload Register at the next update event.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */
    // switch(sr){
    //     case SampleRate_48k: 
    //         // 80'000'000 / 10'000 * 6 = 48'000
    //         audiotimer.Init.Prescaler = 1665;//9999;
    //         audiotimer.Init.Period = 0;
    //         break;
    //     case SampleRate_96k:
    //         // 80'000'000 / 10'000 * 12 = 48'000
    //         audiotimer.Init.Prescaler = 9999;
    //         audiotimer.Init.Period = 11;
    //         break;
    // }

    // Compute the prescaler value so TIM6 triggers at freq-Hz
    uint32_t period = HAL_RCC_GetPCLK1Freq()/samplerate_hz;  
    uint32_t prescaler = 1;
    while (period > 0xffff) {
        period >>= 1;
        prescaler <<= 1;
    }

    samplerate = HAL_RCC_GetPCLK1Freq() / (period * prescaler);

    printf("target sr = %d\n", samplerate_hz);
    printf("actual sr = %d\n", samplerate);

    // if (samplerate_hz == HAL_RCC_GetPCLK1Freq() / (period * prescaler)){
    //     led = 1;
    // }

    // printf("psc = %d\n", prescaler);
    // printf("arr = %d\n",period);

    audiotimer.Init.Period = period - 1;
    audiotimer.Init.Prescaler = prescaler - 1;

    audiotimer.Init.CounterMode = TIM_COUNTERMODE_UP;       
    /*!< Specifies the counter mode.
                                    This parameter can be a value of @ref TIM_Counter_Mode */

    

    audiotimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;     
    /*!< Specifies the clock division.
                                    This parameter can be a value of @ref TIM_ClockDivision */

    audiotimer.Init.RepetitionCounter = 0;
    /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                    reaches zero, an update event is generated and counting restarts
                                    from the RCR value (N).
                                    This means in PWM mode that (N+1) corresponds to:
                                        - the number of PWM periods in edge-aligned mode
                                        - the number of half PWM period in center-aligned mode
                                        GP timers: this parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF.
                                        Advanced timers: this parameter must be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF. */

    audiotimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    /*!< Specifies the auto-reload preload.
                                    This parameter can be a value of @ref TIM_AutoReloadPreload */

    HAL_StatusTypeDef status = HAL_TIM_Base_Init(&audiotimer);

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

    status = HAL_TIMEx_MasterConfigSynchronization(&audiotimer,&sMasterConfig);

    if (status != HAL_OK){
        printf("ERR tim6 config: %d\n", status);
        while(1);
    }

    NVIC_SetVector(TIM6_DAC_IRQn, (uint32_t)audiotimer_interrupt);

    // __HAL_TIM_ENABLE_IT(&audiotimer, TIM_IT_UPDATE);
}

void audiotimer_start(){
    HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&audiotimer);

    if (status != HAL_OK){
        printf("ERR tim6 start: %d\n", status);
        while(1);
    }

    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void audiotimer_stop(){

    NVIC_DisableIRQ(TIM6_DAC_IRQn);

    HAL_StatusTypeDef status = HAL_TIM_Base_Stop(&audiotimer);

    if (status != HAL_OK){
        printf("ERR tim6 start: %d\n", status);
        while(1);
    }
}

// SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);
// DigitalOut spi_cs(SPI_CS);



// volatile bool inserted = false;

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
// {
//     // Read & Update The ADC Result
//     dac_write(adc_read());
// }

// main() runs in its own thread in the OS
int main()
{
    clock_init();

    for(int i = 0; i < DELAY_SAMPLES; i++){
        delayline[i] = 678;
    }

    printf("\nRESTART delayer\n");

    led = 0;


    audiotimer_init(SAMPLERATE_HZ);

    adc_init();
    dac_init();

    adc_start();
    dac_start();

    audiotimer_start();




    // worker.attach([](){
    //     sample = adc_read();

    //     dac_write(sample);
    // }, std::chrono::microseconds(11));

    // while(1){
    //     wait_us(500000);
    //     // sample = adc_read();
    //     printf("%d\n", sample);
    // }

    // cd.rise([](){
    //     led = 0;
    //     inserted = false;
    // });

    // cd.fall([](){
    //     led = 1;
    //     inserted = true;
    // });

    // if (cd == 0){
    //     printf("already inserted\n");
    // } else {

    //     printf("wait for card..\n");

    //     // wait for insertion
    //     while(inserted == false);
    //     // cd.mode(PullUp);
        
    //     printf("inserted..\n");
    // }


    // wait_us(1000);



    // int pos_min = 900;
    // int pos_max = 2100;
    // int center = 1600;

    // int extrema[2] = {1450, 1650};

    // pwm.period(1.0 / 50.0);

    // pwm.resume();

    // int p = 0;


    // while(1){
    //     pwm.pulsewidth_us(extrema[p]);

    //     p = (p + 1) % 2;

    //     wait_us(1000000/2);
    // }

    // while(1){
    //     for(int i = 0; i < sizeof(pos); i++){
    //         pwm.pulsewidth_us(pos[i]);
    //         wait_us(1000000);
    //     }
    // }


//     // pwm.pulsewidth(0.0001);

//     // dac = f;

//     while(1){
//         wait_us(1000000);
        
//         // f = (float)((((int)f * 100) % 100) + 10) / 10.0;

//         // dac = f;

//         // printf("%d\n", (int)(100*f));
//     }

//     t.start();
//     printf("Hello World!\n");
//     t.stop();
//     printf("The time taken was %llu milliseconds\n", duration_cast<milliseconds>(t.elapsed_time()).count());

//         // wait_us(1000000);

// // Call the SDBlockDevice instance initialisation method
//     if (0 != sd.init()) {
//         printf("Init failed \n");
//         return -1;
//     }
//     printf("sd size: %llu\n",         sd.size());
//     printf("sd read size: %llu\n",    sd.get_read_size());
//     printf("sd program size: %llu\n", sd.get_program_size());
//     printf("sd erase size: %llu\n",   sd.get_erase_size());

// // Set the frequency
//     if (0 != sd.frequency(50000000)) {
//         printf("Error setting frequency \n");
//     }


//     for (int i = 0; i < 10; i++){

//     if (0 != sd.erase(i*128, sd.get_erase_size())) {
//         printf("Error Erasing block \n");
//     }

//     t.reset();
//     t.start();
//     // Write data block to the device
//     if (0 == sd.program(block, i*512, 512)) {
//     // if (0 == sd.read(block, 0, 512)) {
//         // Read the data block from the device
//         if (0 == sd.read(block, i*512, 512)) {
//         //     // Print the contents of the block
//             // printf("%s", block);
//         }
//     }
//     t.stop();

//     printf("i: %llu ms\n", duration_cast<milliseconds>(t.elapsed_time()).count());



//     }

//     // Call the SDBlockDevice instance de-initialisation method
//     sd.deinit();

    // uint8_t cmd[6];

    // cmd[0] = 

    // led = 1;

    // while (true) {
    //     // spi.select();
    //     // wait_us(500000);
    //     // spi.deselect();

    //     wait_us(1000000);

    //     // led = !led;
    // }


/*
    uint32_t msi = __HAL_RCC_GET_MSI_RANGE();

    printf("msi = %d (~100kHz = %d, ... ~48MHz = %d)\n", msi, RCC_MSIRANGE_0, RCC_MSIRANGE_11);

    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    printf("sysclk = %d\n", sysclk);
    printf("hclk = %d\n", hclk);
    printf("pclk1 = %d\n", pclk1);
    printf("pclk2 = %d\n", pclk2);

    RCC_OscInitTypeDef RCC_OscInitStruct;
    
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    printf("HSEState = %d\n", RCC_OscInitStruct.HSEState);
    printf("LSEState = %d\n", RCC_OscInitStruct.LSEState);
    printf("HSIState = %d\n", RCC_OscInitStruct.HSIState);
    printf("LSIState = %d\n", RCC_OscInitStruct.LSIState);
    printf("MSIState = %d\n", RCC_OscInitStruct.MSIState);
    printf("HSI48State = %d\n", RCC_OscInitStruct.HSI48State);

    printf("PLL.PLLState = %d\n", RCC_OscInitStruct.PLL.PLLState);
    printf("PLL.PLLSource = %d (MSI = %d, HSI = %d, HSE = %d)\n", RCC_OscInitStruct.PLL.PLLSource, RCC_PLLSOURCE_MSI, RCC_PLLSOURCE_HSI, RCC_PLLSOURCE_HSE);
    printf("PLL.PLLM = %d (Division factor for PLL VCO input clock)\n", RCC_OscInitStruct.PLL.PLLM);
    printf("PLL.PLLN = %d (Multiplication factor for PLL VCO output clock)\n", RCC_OscInitStruct.PLL.PLLN);
    printf("PLL.PLLP = %d (Division factor for SAI clock)\n", RCC_OscInitStruct.PLL.PLLP);
    printf("PLL.PLLQ = %d (Division factor for SDMMC1, RNG and USB clocks)\n", RCC_OscInitStruct.PLL.PLLQ);
    printf("PLL.PLLR = %d (Division for the main system clock)\n", RCC_OscInitStruct.PLL.PLLR);

    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    uint32_t pFLatency;

    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    printf("SYSCLKSource = %d (MSI = %d, HSI = %d, HSE = %d, PLL = %d)\n", RCC_ClkInitStruct.SYSCLKSource, RCC_SYSCLKSOURCE_MSI, RCC_SYSCLKSOURCE_HSI, RCC_SYSCLKSOURCE_HSE, RCC_SYSCLKSOURCE_PLLCLK);
    printf("AHBCLKDivider = %d\n", RCC_ClkInitStruct.AHBCLKDivider);
    printf("APB1CLKDivider = %d\n", RCC_ClkInitStruct.APB1CLKDivider);
    printf("APB2CLKDivider = %d\n", RCC_ClkInitStruct.APB2CLKDivider);

    while(1);

*/
}

