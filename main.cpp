#include "arm_hal_interrupt_private.h"
#include "mbed.h"

// #include "stm32_sdio.h"
#include "mbed_wait_api.h"

#include "audiotimer6.h"
#include "stm32_adc.h"
#include "stm32_dac.h"

#include "stm32_sdio.h"
//#include "bsp_driver_sd.h"

#include "stm32l476xx.h"

#include "stm32l4xx_hal_rcc.h"
#include <cstring>



#define SAMPLERATE_HZ       (48000>>0)

#define ADC_BIAS            687
#define ADC_DIFFERENTIAL    false

#define DAC_BIAS            ADC_BIAS


#define DELAY_MAX_MSEC      1000

Timer stopwatch;

DigitalOut led(LED1);

using namespace std::chrono;


InterruptIn cd(PA_9, PullUp);

struct {
    uint32_t nblocks;

    volatile uint32_t inblock;       // next input block to write (on SD)
    volatile int8_t in[2][512];     // input buffers
    volatile uint16_t inb;           // current input buffer
    volatile uint16_t ini;           // current input sample
    volatile bool infull;   // input buffer is full (write to SD)

    volatile uint32_t outblock;      // next uotput block to read (from SD)
    volatile int8_t out[2][512];    // output buffers
    volatile uint16_t outb;          // current output buffer
    volatile uint16_t outi;          // current output sample
    volatile bool outempty; // output buffer is empty (read new from SD)
} sDelay;


volatile bool sd_busy = false;

void Error_Handler(void){
    printf("Error\n");
    while(1);
}

void delay_init(){
    sDelay.inblock = 0;
    sDelay.inb = 0;
    sDelay.ini = 0;
    sDelay.infull = false;
    memset((uint8_t*)sDelay.in[0], 0, 1024);

    sDelay.outblock = 0;
    sDelay.outb = 0;
    sDelay.outi = 0;
    sDelay.outempty = false;
    memset((uint8_t*)sDelay.out[0], 0, 1024);

    // if (BSP_SD_Init() != HAL_OK){
    if (sd_init() != HAL_OK){
        printf("SD init err\n");
        while(1);
    }

    // sd_stat();

    // compute total number of blocks required
    uint32_t samples_total = SAMPLERATE_HZ * DELAY_MAX_MSEC / 1000;
    sDelay.nblocks = samples_total / 512 + 2;

    sDelay.outblock = 1;

    printf("nblocks = %d\n", sDelay.nblocks);

    // erase used blocks on SD
    // BSP_SD_Erase(0, sDelay.nblocks);
    sd_erase(0, sDelay.nblocks);

}

inline void delay_push(int32_t sample){
    sDelay.in[sDelay.inb][sDelay.ini] = sample;

    sDelay.ini++;

    if (sDelay.ini >= 512){
        sDelay.ini = 0;
        sDelay.inb = (sDelay.inb + 1) % 2;

        if (sDelay.infull){
            led = !led;
            // while(1);
        }

        sDelay.infull = true;
    }
}

inline int32_t delay_pull(){
    int32_t sample = sDelay.out[sDelay.outb][sDelay.outi];


    // sample = sDelay.in[ (sDelay.inb + 1) % 2][sDelay.outi];

    // if (sDelay.out[sDelay.outb][sDelay.outi] < 0 && sample > 0){
    //     led = 1;
    //     while(1);
    // }

    sDelay.outi++;

    if (sDelay.outi >= 512){
        sDelay.outi = 0;
        sDelay.outb = (sDelay.outb + 1) % 2;

        if (sDelay.outempty){
        //     led = 1;

            led = !led;
        //     while(1);
        }

        sDelay.outempty = true;
    }

    return sample;
}

void delay_loop(){
    if (sd_busy == true){
        return;
    }
    if (sd_ready() == false){
        return;
    }

    
    if (sDelay.infull){

        // if (sDelay.inblock == 0){
        //     printf("i ");
        //     for(int i = 0; i < 4; i++){
        //         printf("%d ", sDelay.in[(sDelay.inb + 1) % 2][i]);
        //     }
        //     printf(" / ");
        //     for(int i = 0; i < 4; i++){
        //         printf("%d ", sDelay.in[(sDelay.inb + 1) % 2][128 + i]);
        //     }
        //     printf("\n");
        // }
        // while(1);
        stopwatch.reset();
        stopwatch.start();
        if (!sd_write(sDelay.inblock, (uint8_t*)sDelay.in[(sDelay.inb + 1) % 2], 1)){
        //  if (BSP_SD_WriteBlocks((uint8_t*)sDelay.in[(sDelay.inb + 1) % 2], sDelay.inblock, 1, SD_DATATIMEOUT) != SD_TRANSFER_OK){
        //if (BSP_SD_WriteBlocks_DMA((uint8_t*)&sDelay.in[(sDelay.inb + 1) % 2][0], sDelay.inblock, 1) != SD_TRANSFER_OK){
            stopwatch.stop();
            return;
        }
        stopwatch.stop();

        // printf("w %d\n", stopwatch)//

        // sd_busy = true;
        sDelay.infull = false;

        // printf("b\n");

        // sd_write(sDelay.inblock, (uint8_t*)sDelay.in[(sDelay.inb + 1) % 2]);

        sDelay.inblock = (sDelay.inblock + 1) % sDelay.nblocks;

        // printf("%d\n", sDelay.inblock);

        return;
    }
    else if (sDelay.outempty){
        // return;
    // else {
        // if (sDelay.outblock == 1){
        //     printf("o ");
        //     for(int i = 0; i < 4; i++){
        //         printf("%d ", sDelay.out[sDelay.outb][i]);
        //     }
        //     printf(" / ");
        //     for(int i = 0; i < 4; i++){
        //         printf("%d ", sDelay.out[sDelay.outb][128 + i]);
        //     }
        //     printf("\n");
        // }

        if (!sd_read(sDelay.outblock, (uint8_t*)sDelay.out[(sDelay.outb + 1) % 2], 1)){
        // if (BSP_SD_ReadBlocks((uint8_t*)sDelay.out[(sDelay.outb + 1) % 2], sDelay.outblock, 1, SD_DATATIMEOUT) != SD_TRANSFER_OK){
        // if (BSP_SD_ReadBlocks_DMA((uint8_t*)&sDelay.out[(sDelay.outb + 1) % 2][0], sDelay.outblock, 1) != SD_TRANSFER_OK){
            return;
        }

        // sd_busy = true;
        sDelay.outempty = false;

        // sd_read(sDelay.outblock, (uint8_t*)sDelay.out[(sDelay.outb + 1) % 2]);


        sDelay.outblock = (sDelay.outblock + 1) % sDelay.nblocks;

        // printf(" %d\n", sDelay.outblock);

    }
    // else {
    //     return;
    // }
    // printf("io %d %d\n", sDelay.inblock, sDelay.outblock);
}

void clock_init(){
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    uint32_t pFLatency;


    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

    // if uncommented we get a sysclk of 48MHz which is perfect to get a a precise TIM6 interrupt at 48kHz ..
    // because at the max (default) sysclk of 80MHz we don't get a precise 
    //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency);

    printf("SYSCLKSource = %d (MSI = %ld, HSI = %ld, HSE = %ld, PLL = %ld)\n", RCC_ClkInitStruct.SYSCLKSource, RCC_SYSCLKSOURCE_MSI, RCC_SYSCLKSOURCE_HSI, RCC_SYSCLKSOURCE_HSE, RCC_SYSCLKSOURCE_PLLCLK);
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
    
     sample = adc_read() / 2;

    delay_push(sample);

    int32_t wet = delay_pull();

    int32_t out = (sample + wet) / 2;


    dac_write(out);

    // if (sDelay.inblock < sDelay.outblock && sDelay.inblock + 5 < sDelay.outblock){
    //     led = 1;
    //     while(1);
    // }
    // if (sDelay.inblock > sDelay.outblock && sDelay.inblock + 5 < sDelay.outblock + sDelay.nblocks){
    //     led = 1;
    //     while(1);
    // }
}



// void BSP_SD_ErrorCallback(void){
//     // led = true;
// }

void BSP_SD_WriteCpltCallback(void){
    sd_busy = false;
    // led = !led;
}
void BSP_SD_ReadCpltCallback(void){
    sd_busy = false;
    // led = !led;
    // led = 1;
}

// main() runs in its own thread in the OS
int main()
{
    printf("\n===== RESTART delayer =====\n");

    // clock_init();

    delay_init();
    
    // sd_stat();

    wait_us(1000);

    // uint16_t block[1024];
    // uint8_t *u8 = (uint8_t*)block;
    // for(int i = 0; i < 512; i++){
    //     block[i] = i;
    // }

    // // sd_busy = true;
    // // int s = BSP_SD_WriteBlocks((uint32_t*)block, 0, 1, SD_DATATIMEOUT);
    // int s = 1;
    
    // s = sd_write(0, u8, 1);

    // printf("status = %d\n", s);

    // // while(!s);

    // // led = 1;
    // // // while(sd_busy);

    // printf("written\n");

    // // led = 0;

    // memset(block, 0, sizeof(block));

    // wait_us(1000);

    // // // sd_busy = true;
    // // s = BSP_SD_ReadBlocks((uint32_t*)block, 0, 1, SD_DATATIMEOUT);
    // s = sd_read(0, (uint8_t*)&block[0], 1);
    

    // printf("status = %d\n", s);
    // while(!s);

    // // // while(sd_busy);
    // // *u16 = (uint16_t*)block;
    // for(int i = 0; i < 512; i++){
    //     printf("%d ", block[i]);
    // }
    // printf("\n");

    // while(1);

    // int I = 1;

    // for (int i = 0; i < I; i++){
    //     printf("1 %d\n", i);

    //     // while(BSP_SD_WriteBlocks((uint32_t*)block, i, 1, SD_DATATIMEOUT) == false){
    //     busy = true;
    //     while(busy == true){
    //         BSP_SD_WriteBlocks_DMA((uint32_t*)block, i, 1);

    //         printf(".\n");
    //         wait_us(1000000);
    //     }

    //     wait_us(1000);
    // }

    
    // for (int i = 0; i < I; i++){

    //     memset(block, 0, sizeof(block));

    //     printf("2 %d\n", i);
    //     busy = true;
    //     // while(BSP_SD_ReadBlocks((uint32_t*)block, i, 1, SD_DATATIMEOUT) == false){
    //     int s = BSP_SD_ReadBlocks_DMA((uint32_t*)block, i, 1);

    //     printf("state = %d\n", s);
    //     while(busy == true){
    //         // int s = BSP_SD_ReadBlocks_DMA((uint32_t*)block, i+1, 1);

    //         printf(".\n");
    //         wait_us(1000000);
    //     }

    //     wait_us(1000000);
    //     for(int i = 0; i < 6; i++){
    //         printf("%02X ", block[i]);
    //     }
    //     printf("\n");
    // }


//     wait_us(1000);

//     memcpy(block, "byebye", 7);

//     printf("1\n");
//     while(sd_write(0, block) == false){
//         wait_us(1000000);
//     }
// wait_us(1000);

//     memset(block, 0, sizeof(block));

//     printf("2\n");
//     while(sd_read(0, block) == false){
//         wait_us(1000000);
//     }

//     printf("%s\n", block);
    // printf("3\n");

    audiotimer6_init(SAMPLERATE_HZ);

    adc_init(ADC_BIAS, ADC_DIFFERENTIAL);
    dac_init(DAC_BIAS);

    adc_start();
    dac_start();

    audiotimer6_start();

    while(1){
        delay_loop();
    }

}

