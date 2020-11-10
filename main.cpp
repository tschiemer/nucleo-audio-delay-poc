#include "arm_hal_interrupt_private.h"
#include "mbed.h"

// #include "stm32_sdio.h"
#include "mbed_wait_api.h"

#include "stm32_adc.h"
#include "stm32_dac.h"
#include "audiotimer6.h"

#include "stm32l476xx.h"

#include "stm32l4xx_hal_rcc.h"
#include <cstdint>


#define SAMPLERATE_HZ       48000

#define ADC_BIAS            687
#define ADC_DIFFERENTIAL    false

#define DAC_BIAS            ADC_BIAS


#define DELAY_SAMPLES (SAMPLERATE_HZ>>2)

using namespace std::chrono;


InterruptIn cd(PA_9, PullUp);

uint16_t delayline[DELAY_SAMPLES];
volatile uint32_t delaylinei = 0;

// Timer t;

void clock_init(){
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    uint32_t pFLatency;


    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    // if uncommented we get a sysclk of 48MHz which is perfect to get a a precise TIM6 interrupt at 48kHz ..
    // because at the max (default) sysclk of 80MHz we don't get a precise 
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

int32_t sample;
// called at roughly SAMPLERATE_HZ
// ATTENTION: interrupt context
void audiotimer6_callback(){
    
     sample = adc_read();

    // delayline[delaylinei] = sample;

    // delaylinei = (delaylinei + 1) % DELAY_SAMPLES;

    // uint32_t mix = (sample + delayline[delaylinei]) / 2;

    dac_write(sample);
}


// volatile bool inserted = false;


// main() runs in its own thread in the OS
int main()
{
    printf("\nRESTART delayer\n");

    // clock_init();

    audiotimer6_init(SAMPLERATE_HZ);

    adc_init(ADC_BIAS, ADC_DIFFERENTIAL);
    dac_init(DAC_BIAS);

    adc_start();
    dac_start();

    audiotimer6_start();


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




}

