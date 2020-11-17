#include "mbed.h"

#include "stm32_sdio.h"

#include "stm32l476xx.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_rcc_ex.h"

#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_gpio_ex.h"

#include "stm32l4xx_hal_sd.h"
#include "stm32l4xx_ll_sdmmc.h"

// https://github.com/akospasztor/stm32-bootloader/blob/master/projects/STM32L496-CustomHw/source/bsp_driver_sd.c

#define SDMMC_IRQ_PRIO  1
#define SD_DMA_IRQ_PRIO 2

using namespace std::chrono;




static SD_HandleTypeDef hsd;

// static DMA_HandleTypeDef hdmatx;
// static DMA_HandleTypeDef hdmarx;


HAL_SD_CardCIDTypeDef pCID;
HAL_SD_CardCSDTypeDef pCSD;
// HAL_SD_CardStatusTypeDef pStatus;
HAL_SD_CardInfoTypeDef pCardInfo;

#define check_error( op, msg )   if ( (op) != HAL_OK) { \
                                printf("HAL ERROR: %s\n", msg); \
                                while(1); \
                            }


/**
* @brief This function handles SDIO global interrupt.
*/
// void _SDIO_IRQHandler(void)
// {
//     HAL_SD_IRQHandler(&hsd);
// }


// void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
// {
//   UNUSED(hsd);
// }

// /**
//   * @brief Rx Transfer completed callbacks
//   * @param hsd: Pointer SD handle
//   * @retval None
//   */
// void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
// {
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(hsd);
// }
/**
* @brief This function handles DMAx stream_n global interrupt. DMA Rx
*/
// void _DMA_Stream_Rx_IRQHandler(void)
// {
//   HAL_DMA_IRQHandler(hsd.hdmarx);
// }

// /**
// * @brief This function handles DMAx stream_n global interrupt. DMA Tx
// */
// void _DMA_Stream_Tx_IRQHandler(void)
// {
//   HAL_DMA_IRQHandler(hsd.hdmatx);
// }

/**
 * @brief DMA2 Channel5 ISR
 * @note  SDMMC DMA Tx, Rx
 */
// void DMA2_Channel5_IRQHandler(void)
// {
//     if((hsd.Context == (SD_CONTEXT_DMA | SD_CONTEXT_READ_SINGLE_BLOCK)) ||
//        (hsd.Context == (SD_CONTEXT_DMA | SD_CONTEXT_READ_MULTIPLE_BLOCK)))
//     {
//         HAL_DMA_IRQHandler(hsd.hdmarx);
//     }
//     else if((hsd.Context ==
//              (SD_CONTEXT_DMA | SD_CONTEXT_WRITE_SINGLE_BLOCK)) ||
//             (hsd.Context ==
//              (SD_CONTEXT_DMA | SD_CONTEXT_WRITE_MULTIPLE_BLOCK)))
//     {
//         HAL_DMA_IRQHandler(hsd.hdmatx);
//     }

// }

/**
 * @brief SDMMC ISR
 */
void __SDMMC1_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd);
}

/**
 * @brief Configure the DMA to receive data from the SD card
 * @retval
 *  HAL_ERROR or HAL_OK
 */
static HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef* hsd)
{
    static DMA_HandleTypeDef hdma_rx;
    HAL_StatusTypeDef        status = HAL_ERROR;

    /* Configure DMA Rx parameters */
    hdma_rx.Instance                 = DMA2_Channel5;
    hdma_rx.Init.Request             = DMA_REQUEST_7;
    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsd, hdmarx, hdma_rx);

    /* Stop any ongoing transfer and reset the state */
    HAL_DMA_Abort(&hdma_rx);

    /* Deinitialize the Channel for new transfer */
    HAL_DMA_DeInit(&hdma_rx);

    /* Configure the DMA Channel */
    status = HAL_DMA_Init(&hdma_rx);

    /* NVIC configuration for DMA transfer complete interrupt */
    // HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, SD_DMA_IRQ_PRIO, 0);
    // NVIC_SetVector(DMA2_Channel5_IRQn, (uint32_t) &DMA2_Channel5_IRQHandler);
    // HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

    return status;
}

/**
 * @brief Configure the DMA to transmit data to the SD card
 * @retval
 *  HAL_ERROR or HAL_OK
 */
static HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef* hsd)
{
    static DMA_HandleTypeDef hdma_tx;
    HAL_StatusTypeDef        status;

    /* Configure DMA Tx parameters */
    hdma_tx.Instance                 = DMA2_Channel5;
    hdma_tx.Init.Request             = DMA_REQUEST_7;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsd, hdmatx, hdma_tx);

    /* Stop any ongoing transfer and reset the state */
    HAL_DMA_Abort(&hdma_tx);

    /* Deinitialize the Channel for new transfer */
    HAL_DMA_DeInit(&hdma_tx);

    /* Configure the DMA Channel */
    status = HAL_DMA_Init(&hdma_tx);

    /* NVIC configuration for DMA transfer complete interrupt */
    // HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, SD_DMA_IRQ_PRIO, 0);
    // NVIC_SetVector(DMA2_Channel5_IRQn, (uint32_t) &DMA2_Channel5_IRQHandler);
    // HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

    return status;
}

// (#)Initialize the SDMMC low level resources by implementing the HAL_SD_MspInit() API:
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    // printf("MspInit\n");

    // wait_us(1000);

//         (##) Call the function HAL_RCCEx_PeriphCLKConfig with RCC_PERIPHCLK_SDMMC1 for
//         PeriphClockSelection and select SDMMC1 clock source (MSI, main PLL or PLLSAI1)

    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);

    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC1;
    RCC_PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_MSI;//RCC_SDMMC1CLKSOURCE_PLLSAI1;//RCC_SDMMC1CLKSOURCE_MSI;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    // RCC_PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;

// #if defined(RCC_HSI48_SUPPORT)
// #define RCC_SDMMC1CLKSOURCE_HSI48      0x00000000U  /*!< HSI48 clock selected as SDMMC1 clock          */
// #else
// #define RCC_SDMMC1CLKSOURCE_NONE       0x00000000U  /*!< No clock selected as SDMMC1 clock             */
// #endif /* RCC_HSI48_SUPPORT */
// #define RCC_SDMMC1CLKSOURCE_PLLSAI1    RCC_CCIPR_CLK48SEL_0     /*!< PLLSAI1 "Q" clock selected as SDMMC1 clock    */
// #define RCC_SDMMC1CLKSOURCE_PLL        RCC_CCIPR_CLK48SEL_1     /*!< PLL "Q" clock selected as SDMMC1 clock        */
// #define RCC_SDMMC1CLKSOURCE_MSI        RCC_CCIPR_CLK48SEL       /*!< MSI clock selected as SDMMC1 clock            */
// #if defined(RCC_CCIPR2_SDMMCSEL)
// #define RCC_SDMMC1CLKSOURCE_PLLP       RCC_CCIPR2_SDMMCSEL      /*!< PLL "P" clock selected as SDMMC1 kernel clock */
// #endif /* RCC_CCIPR2_SDMMCSEL */



    // printf("MspInit DONE\n");
    // pClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_MSI;


    check_error(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit), "init: clk");


//         (##) Enable the SDMMC interface clock using __HAL_RCC_SDMMC1_CLK_ENABLE();

    __HAL_RCC_SDMMC1_CLK_ENABLE();


//         (##) SDMMC pins configuration for SD card
//             (+++) Enable the clock for the SDMMC GPIOs using the functions __HAL_RCC_GPIOx_CLK_ENABLE();
//             (+++) Configure these SDMMC pins as alternate function pull-up using HAL_GPIO_Init()
//                   and according to your pin assignment;

	GPIO_InitTypeDef PORT;

    __HAL_RCC_GPIOC_CLK_ENABLE(); // PC8-12
    __HAL_RCC_GPIOD_CLK_ENABLE(); // PD2

	PORT.Mode = GPIO_MODE_AF_PP; // Alternative function mode
	PORT.Speed = GPIO_SPEED_FREQ_VERY_HIGH; //GPIO_Speed_40MHz; // High speed // GPIO_SPEED_FREQ_HIGH
	// PORT.OType = GPIO_OType_PP; // Output push-pull
	PORT.Pull = GPIO_PULLUP;//GPIO_NOPULL;//GPIO_PULLUP; // Pull-up
    PORT.Alternate = GPIO_AF12_SDMMC1;
    
    
    PORT.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOC, &PORT);

    PORT.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOC, &PORT);

    PORT.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOC, &PORT);

    PORT.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOC, &PORT);

    PORT.Pin = GPIO_PIN_12;
    // PORT.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &PORT);

    PORT.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOD, &PORT);

//         (##) On STM32L4Rx/STM32L4Sxx devices, no DMA configuration is need, an internal DMA for SDMMC Peripheral is used.

//         (##) On other devices, perform DMA configuration if you need to use DMA process (HAL_SD_ReadBlocks_DMA()
//              and HAL_SD_WriteBlocks_DMA() APIs).
//             (+++) Enable the DMAx interface clock using __HAL_RCC_DMAx_CLK_ENABLE();
//             (+++) Configure the DMA using the function HAL_DMA_Init() with predeclared and filled.
//         (##) NVIC configuration if you need to use interrupt process when using DMA transfer.
//             (+++) Configure the SDMMC and DMA interrupt priorities using functions
//                   HAL_NVIC_SetPriority(); DMA priority is superior to SDMMC's priority
//             (+++) Enable the NVIC DMA and SDMMC IRQs using function HAL_NVIC_EnableIRQ()
//             (+++) SDMMC interrupts are managed using the macros __HAL_SD_ENABLE_IT()
//                   and __HAL_SD_DISABLE_IT() inside the communication process.
//             (+++) SDMMC interrupts pending bits are managed using the macros __HAL_SD_GET_IT()
//                   and __HAL_SD_CLEAR_IT()


    // HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    // NVIC_SetVector(SDMMC1_IRQn, (uint32_t) &_SDIO_IRQHandler);
    // // HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    // __SDMMC_DMA_ENABLE(SDMMC1);

    // Configure and enable DMA IRQ Channel
    // SDIO IRQ should have a higher priority than DMA IRQ because it needs to
    // preempt the DMA irq handler to set a flag indicating the end of transfer.
    // HAL_NVIC_SetPriority(SDIO_IRQn, IRQ_PRI_SDIO, IRQ_SUBPRI_SDIO);
    // HAL_NVIC_EnableIRQ(SDIO_IRQn);

    HAL_NVIC_SetPriority(SDMMC1_IRQn, SDMMC_IRQ_PRIO, 0);
    NVIC_SetVector(SDMMC1_IRQn, (uint32_t) &__SDMMC1_IRQHandler);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    // HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

    // HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

//         (##) NVIC configuration if you need to use interrupt process (HAL_SD_ReadBlocks_IT()
//              and HAL_SD_WriteBlocks_IT() APIs).
//             (+++) Configure the SDMMC interrupt priorities using function HAL_NVIC_SetPriority();
//             (+++) Enable the NVIC SDMMC IRQs using function HAL_NVIC_EnableIRQ()
//             (+++) SDMMC interrupts are managed using the macros __HAL_SD_ENABLE_IT()
//                   and __HAL_SD_DISABLE_IT() inside the communication process.
//             (+++) SDMMC interrupts pending bits are managed using the macros __HAL_SD_GET_IT()
//                   and __HAL_SD_CLEAR_IT()
//     (#) At this stage, you can perform SD read/write/erase operations after SD card initialization

    // printf("MspInit DONE\n");

    // wait_us(1000);
}

/**
  * @brief  De-Initialize SD MSP.
  * @param  hsd: Pointer to SD handle
  * @retval None
  */
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{

    /* Disable NVIC for SDIO interrupts */
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

    __HAL_RCC_SDMMC1_CLK_DISABLE();
}


HAL_StatusTypeDef sd_init(){

    // printf("sd_init(): start\n");

    hsd.Instance = SDMMC1;
//   uint32_t ClockEdge;            /*!< Specifies the clock transition on which the bit capture is made.
//                                       This parameter can be a value of @ref SDMMC_LL_Clock_Edge                 */
    hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;

#if !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
//   uint32_t ClockBypass;          /*!< Specifies whether the SDMMC Clock divider bypass is
//                                       enabled or disabled.
//                                       This parameter can be a value of @ref SDMMC_LL_Clock_Bypass               */
    hsd.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
#endif /* !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */
    


//   uint32_t ClockPowerSave;       /*!< Specifies whether SDMMC Clock output is enabled or
//                                       disabled when the bus is idle.
//                                       This parameter can be a value of @ref SDMMC_LL_Clock_Power_Save           */
    hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE; 

//   uint32_t BusWide;              /*!< Specifies the SDMMC bus width.
//                                       This parameter can be a value of @ref SDMMC_LL_Bus_Wide                   */
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;

//   uint32_t HardwareFlowControl;  /*!< Specifies whether the SDMMC hardware flow control is enabled or disabled.
//                                       This parameter can be a value of @ref SDMMC_LL_Hardware_Flow_Control      */
    hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;//SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;


//   uint32_t ClockDiv;             /*!< Specifies the clock frequency of the SDMMC controller.
//                                       This parameter can be a value between Min_Data = 0 and Max_Data = 1023   */
    hsd.Init.ClockDiv = 0;//4;//SDMMC_INIT_CLK_DIV >> 4;

#if defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || defined(STM32L4S9xx)
//   uint32_t Transceiver;          /*!< Specifies whether external Transceiver is enabled or disabled.
//                                       This parameter can be a value of @ref SDMMC_LL_Transceiver */
    hsd.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
#endif /* STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

    // SDMMC1->DTIMER = 0;
    // SDMMC1->DLEN = 0;
    // SDMMC1->DCTRL = 0;

    // hsd.hdmatx = &hdmatx;
    // hsd.hdmarx = &hdmarx;

    HAL_StatusTypeDef status = HAL_SD_Init(&hsd);

    // printf("init = %d\n", state);

    // check_error(state, "Init")

    if (status == HAL_OK){
        check_error(HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_4B), "Wide");
        // check_error(HAL_SD_ConfigSpeedBusOperation(&hsd, SDMMC_SPEED_MODE_HIGH), "Speed");
    }



    // __HAL_RCC_DMA2_CLK_ENABLE();


//     hdmatx.Instance = DMA2_Channel4;

    
//     hdmatx.Init.Request = DMA_REQUEST_0;                  
//      /*!< Specifies the request selected for the specified channel.
//                                            This parameter can be a value of @ref DMA_request */

//     hdmatx.Init.Direction = DMA_MEMORY_TO_PERIPH;                
//    /*!< Specifies if the data will be transferred from memory to peripheral,
//                                            from memory to memory or from peripheral to memory.
//                                            This parameter can be a value of @ref DMA_Data_transfer_direction */

//     hdmatx.Init.PeriphInc = DMA_PINC_DISABLE;                 
//   /*!< Specifies whether the Peripheral address register should be incremented or not.
//                                            This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

//     hdmatx.Init.MemInc = DMA_MINC_DISABLE;                   
//    /*!< Specifies whether the memory address register should be incremented or not.
//                                            This parameter can be a value of @ref DMA_Memory_incremented_mode */

//     hdmatx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      
//    /*!< Specifies the Peripheral data width.
//                                            This parameter can be a value of @ref DMA_Peripheral_data_size */

//     hdmatx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;         
//    /*!< Specifies the Memory data width.
//                                            This parameter can be a value of @ref DMA_Memory_data_size */

//     hdmatx.Init.Mode = DMA_NORMAL;                     
//    /*!< Specifies the operation mode of the DMAy Channelx.
//                                            This parameter can be a value of @ref DMA_mode
//                                            @note The circular buffer mode cannot be used if the memory-to-memory
//                                                  data transfer is configured on the selected Channel */

//     hdmatx.Init.Priority = DMA_PRIORITY_HIGH;                 
//    /*!< Specifies the software priority for the DMAy Channelx.
//                                            This parameter can be a value of @ref DMA_Priority_level */

//     status = HAL_DMA_Init(&hdmatx);

//     if (status != HAL_OK){
//         printf("dma init err: %d\n", status);
//         while(1);
//     }




//     hdmarx.Instance = DMA2_Channel5;

    
//     hdmarx.Init.Request = DMA_REQUEST_1;                  
//      /*!< Specifies the request selected for the specified channel.
//                                            This parameter can be a value of @ref DMA_request */

//     hdmarx.Init.Direction = DMA_PERIPH_TO_MEMORY;                
//    /*!< Specifies if the data will be transferred from memory to peripheral,
//                                            from memory to memory or from peripheral to memory.
//                                            This parameter can be a value of @ref DMA_Data_transfer_direction */

//     hdmarx.Init.PeriphInc = DMA_PINC_DISABLE;                 
//   /*!< Specifies whether the Peripheral address register should be incremented or not.
//                                            This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

//     hdmarx.Init.MemInc = DMA_MINC_DISABLE;                   
//    /*!< Specifies whether the memory address register should be incremented or not.
//                                            This parameter can be a value of @ref DMA_Memory_incremented_mode */

//     hdmarx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      
//    /*!< Specifies the Peripheral data width.
//                                            This parameter can be a value of @ref DMA_Peripheral_data_size */

//     hdmarx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;         
//    /*!< Specifies the Memory data width.
//                                            This parameter can be a value of @ref DMA_Memory_data_size */

//     hdmarx.Init.Mode = DMA_NORMAL;                     
//    /*!< Specifies the operation mode of the DMAy Channelx.
//                                            This parameter can be a value of @ref DMA_mode
//                                            @note The circular buffer mode cannot be used if the memory-to-memory
//                                                  data transfer is configured on the selected Channel */

//     hdmarx.Init.Priority = DMA_PRIORITY_HIGH;                 
//    /*!< Specifies the software priority for the DMAy Channelx.
//                                            This parameter can be a value of @ref DMA_Priority_level */

//     status = HAL_DMA_Init(&hdmarx);

//     if (status != HAL_OK){
//         printf("dma init err: %d\n", status);
//         while(1);
//     }


    // HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

    // printf("sd_init(): exit\n");

    return status;
}

void sd_deinit(){
    check_error(HAL_SD_DeInit(&hsd), "DeInit");
// void              HAL_SD_MspDeInit(&hsd);
}

void sd_info(){

// HAL_StatusTypeDef       HAL_SD_SendSDStatus (&hsd, uint32_t *pSDstatus);

    HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState (&hsd);

    printf("sd card state = %d\n", state);

// #define HAL_SD_CARD_READY          0x00000001U  /*!< Card state is ready                     */
// #define HAL_SD_CARD_IDENTIFICATION 0x00000002U  /*!< Card is in identification state         */
// #define HAL_SD_CARD_STANDBY        0x00000003U  /*!< Card is in standby state                */
// #define HAL_SD_CARD_TRANSFER       0x00000004U  /*!< Card is in transfer state               */
// #define HAL_SD_CARD_SENDING        0x00000005U  /*!< Card is sending an operation            */
// #define HAL_SD_CARD_RECEIVING      0x00000006U  /*!< Card is receiving operation information */
// #define HAL_SD_CARD_PROGRAMMING    0x00000007U  /*!< Card is in programming state            */
// #define HAL_SD_CARD_DISCONNECTED   0x00000008U  /*!< Card is disconnected                    */
// #define HAL_SD_CARD_ERROR          0x000000FFU  /*!< Card response Error                     */

    check_error(HAL_SD_GetCardCID(&hsd, &pCID), "getCID");

    printf("pCID.ManufacturerID = %02X\n", pCID.ManufacturerID);
    printf("pCID.OEM_AppliID = %04X\n", pCID.OEM_AppliID);
    printf("pCID.ProdName1 = %08X\n", pCID.ProdName1);
    printf("pCID.ProdName2 = %02X\n", pCID.ProdName2);
    printf("pCID.ProdRev = %02X\n", pCID.ProdRev);
    printf("pCID.ProdSN = %08X\n", pCID.ProdSN);
    printf("pCID.ManufactDate = %04X\n", pCID.ManufactDate);
    printf("pCID.CID_CRC = %02X\n", pCID.CID_CRC);

    check_error(HAL_SD_GetCardCSD(&hsd, &pCSD), "getCSD");

    printf("pCSD.SysSpecVersion = %d\n", pCSD.SysSpecVersion);
    printf("pCSD.CardComdClasses = %d\n", pCSD.CardComdClasses);
    printf("pCSD.DeviceSize = %d\n", pCSD.DeviceSize);
    printf("pCSD.DeviceSizeMul = %d\n", pCSD.DeviceSizeMul);
    printf("pCSD.MaxBusClkFrec = %d\n", pCSD.MaxBusClkFrec);
    printf("pCSD.TAAC (read time) = %d\n", pCSD.TAAC);
    printf("pCSD.RdBlockLen = %d\n", pCSD.RdBlockLen);
    printf("pCSD.MaxWrBlockLen = %d\n", pCSD.MaxWrBlockLen);
    printf("pCSD.EraseGrSize = %d\n", pCSD.EraseGrSize);
    printf("pCSD.EraseGrMul = %d\n", pCSD.EraseGrMul);
    printf("pCSD.WrProtectGrSize = %d\n", pCSD.EraseGrSize);
    printf("pCSD.WrProtectGrEnable = %d\n", pCSD.WrProtectGrEnable);

    // check_error(HAL_SD_GetCardStatus(&hsd, &pStatus), "getStatus");

    check_error(HAL_SD_GetCardInfo(&hsd, &pCardInfo), "getInfo");
    printf("info.CardType = %d\n", pCardInfo.CardType);
    printf("info.CardVersion = %d\n", pCardInfo.CardVersion);
    printf("info.Class = %d\n", pCardInfo.Class);
    printf("info.RelCardAdd = %d\n", pCardInfo.RelCardAdd);
    printf("info.BlockNbr = %d\n", pCardInfo.BlockNbr);
    printf("info.BlockSize = %d\n", pCardInfo.BlockSize);
    printf("info.LogBlockNbr = %d\n", pCardInfo.LogBlockNbr);
    printf("info.LogBlockSize = %d\n", pCardInfo.LogBlockSize);
    // printf("info.CardSpeed = %d\n", pCardInfo.CardSpeed);

}


//   HAL_SD_STATE_RESET                  = ((uint32_t)0x00000000U),  /*!< SD not yet initialized or disabled  */
//   HAL_SD_STATE_READY                  = ((uint32_t)0x00000001U),  /*!< SD initialized and ready for use    */
//   HAL_SD_STATE_TIMEOUT                = ((uint32_t)0x00000002U),  /*!< SD Timeout state                    */
//   HAL_SD_STATE_BUSY                   = ((uint32_t)0x00000003U),  /*!< SD process ongoing                  */
//   HAL_SD_STATE_PROGRAMMING            = ((uint32_t)0x00000004U),  /*!< SD Programming State                */
//   HAL_SD_STATE_RECEIVING              = ((uint32_t)0x00000005U),  /*!< SD Receiving State                  */
//   HAL_SD_STATE_TRANSFER               = ((uint32_t)0x00000006U),  /*!< SD Transfert State                  */
//   HAL_SD_STATE_ERROR   

HAL_SD_StateTypeDef sd_state(){
    return HAL_SD_GetState(&hsd);
}


bool sd_write(uint32_t address, uint8_t * data, uint8_t nblocks){

    HAL_StatusTypeDef status = HAL_OK;

    // /* Invalidate the dma rx handle*/
    // hsd.hdmarx = NULL;

    // /* Prepare the dma channel for a read operation */
    // status = SD_DMAConfigTx(&hsd);
    // if (status != HAL_OK)  printf("status1 = %d\n", status);

    // if (status == HAL_OK){
    //     status = HAL_SD_WriteBlocks_DMA(&hsd, data, address, 1);
    // }
    status = HAL_SD_WriteBlocks(&hsd, data, address, nblocks, 150);

    // if (status != HAL_OK ) printf("wstatus2 = %d %08X\n", status, hsd.ErrorCode);

    

    return status == HAL_OK;
}

bool sd_read(uint32_t address, uint8_t * data, uint8_t nblocks){

    // if (hsd.State == )

    HAL_StatusTypeDef status = HAL_OK;

    // /* Invalidate the dma tx handle*/
    // hsd.hdmatx = NULL;

    // /* Prepare the dma channel for a read operation */
    // status  = SD_DMAConfigRx(&hsd);
    // if (status != HAL_OK) printf("status1 = %d\n", status);

    // if (status == HAL_OK){
    //     status = HAL_SD_ReadBlocks_DMA(&hsd, data, address, 1);
    //     if (status != HAL_OK) printf("status2 = %d\n", status);
    // }
    status = HAL_SD_ReadBlocks(&hsd, data, address, nblocks, 150);

    // if (status != HAL_OK) printf("rstatus2 = %d %08X\n", status, hsd.ErrorCode);


    return status == HAL_OK;
}

void sd_erase(uint32_t from, uint32_t to){
    if (HAL_SD_Erase(&hsd, from, to) != HAL_OK){
        printf("Erase err\n");
        while(1);
    }
}


// void sd_access(){

// // HAL_StatusTypeDef HAL_SD_ReadBlocks     (&hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
// // HAL_StatusTypeDef HAL_SD_WriteBlocks    (&hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
// // HAL_StatusTypeDef HAL_SD_Erase          (&hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd);
// }

// void sd_test(Timer &t){

//     HAL_StatusTypeDef state = sd_init();

//     if (state != HAL_OK){
//         printf("FAIL!\n");
//         while(1);
//     }

//     // HAL_StatusTypeDef state = HAL_ERROR;
//     // for (int i = 0; i < 1 && state != HAL_OK; i++){
//     //     state = sd_init();
//     //     if (state != HAL_OK){
//     //         wait_us(100);
//     //     }
//     // }
//     // if (state != HAL_OK){
//     //     printf("FAIL!\n");
//     //     while(1);
//     // }

//     wait_us(1000);

//     // sd_stat();

//     t.start();
//     HAL_SD_WriteBlocks(&hsd, blocks, ADDRESS, BLOCK_N, 1000000);
//     t.stop();
//     printf("The time taken was %llu usec\n", duration_cast<microseconds>(t.elapsed_time()).count());

//     t.reset();

//     t.start();
//     HAL_SD_ReadBlocks(&hsd, blocks, ADDRESS, BLOCK_N, 1000000);
//     t.stop();
//     printf("The time taken was %llu usec\n", duration_cast<microseconds>(t.elapsed_time()).count());

//     printf("%s\n", blocks);

//     while(1){


//     }

//     sd_deinit();
// }