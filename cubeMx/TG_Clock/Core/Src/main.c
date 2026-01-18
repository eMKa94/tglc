/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "rtc.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g0xx_ll_tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void dispResetLow(void)
{
    LL_GPIO_ResetOutputPin(DISP_RESET_GPIO_Port, DISP_RESET_Pin);
}

void dispResetHigh(void)
{
    LL_GPIO_SetOutputPin(DISP_RESET_GPIO_Port, DISP_RESET_Pin);
}


void pushOutLow(void)
{
    LL_GPIO_ResetOutputPin(PUSH_OUT_GPIO_Port, PUSH_OUT_Pin);
}

void pushOutHigh(void)
{
    LL_GPIO_SetOutputPin(PUSH_OUT_GPIO_Port, PUSH_OUT_Pin);
}


void uiTimeBaseTimerInit(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM16);

    LL_TIM_InitTypeDef timerConfig = {
        // .Autoreload = 59999,
        .Autoreload        = 99,
        .ClockDivision     = LL_TIM_CLOCKDIVISION_DIV2,
        .CounterMode       = LL_TIM_COUNTERMODE_UP,
        .Prescaler         = 31,
        .RepetitionCounter = 0

    };

    LL_TIM_Init(TIM16, &timerConfig);

    LL_TIM_EnableIT_UPDATE(TIM16);

    NVIC_EnableIRQ(TIM16_IRQn);
}

void uiTimeBaseTimerStart(void)
{
    LL_TIM_EnableCounter(TIM16);
}

void uiTimeBaseTimerStop(void)
{
    LL_TIM_DisableCounter(TIM16);
    LL_TIM_SetCounter(TIM16, 0);
}
#define DIGIT_0 0b10101111
#define DIGIT_1 0b00000110
#define DIGIT_2 0b00111011
#define DIGIT_3 0b00011111
#define DIGIT_4 0b10010110
#define DIGIT_5 0b10011101
#define DIGIT_6 0b10111101
#define DIGIT_7 0b00000111
#define DIGIT_8 0b10111111
#define DIGIT_9 0b10011111

#define PWM_STEPS 100

static volatile uint8_t targetPwmPerBit[32]       = {0};
static volatile uint8_t currentPwmPerBit[32]      = {0};
static volatile uint8_t currentGammaPwmPerBit[32] = {0};


static volatile uint8_t pwmLutA[PWM_STEPS][4] = {0};
static volatile uint8_t pwmLutB[PWM_STEPS][4] = {0};

static volatile uint8_t (*activeLut)[4] = pwmLutA;
static uint8_t (*buildLut)[4]           = pwmLutB;

static volatile uint8_t currentPwmLut = 0;


static volatile uint8_t displayMask[4] = {0xff, 0xfF, 0xff, 0xff};

static volatile uint8_t spiIndex       = 0;
static volatile uint8_t pseudoPwmIndex = 0;
static volatile uint8_t pwmValue       = 100;

__attribute__((interrupt)) void TIM16_IRQHandler(void)
{
    if (!LL_TIM_IsActiveFlag_UPDATE(TIM16))
        return;

    if (++pseudoPwmIndex >= PWM_STEPS)
        pseudoPwmIndex = 0;

    LL_SPI_EnableIT_TXE(SPI1);
    LL_TIM_ClearFlag_UPDATE(TIM16);
}

// FxEGDCBA


void enterCriticalSection(void)
{
    __disable_irq();
}
void exitCriticalSection(void)
{
    __enable_irq();
}
void swapPwmLut(void)
{
    enterCriticalSection();
    activeLut = buildLut;
    buildLut  = (buildLut == pwmLutA) ? pwmLutB : pwmLutA;
    exitCriticalSection();
}

void setDisplayFourDigitNumber2(uint16_t number, uint8_t maxPwm)
{
    uint8_t digitValues[4];

    digitValues[0] = (number >= 1000) ? (number / 1000) % 10 : 0;
    digitValues[1] = (number >= 100) ? (number / 100) % 10 : 0;
    digitValues[2] = (number >= 10) ? (number / 10) % 10 : 0;
    digitValues[3] = number % 10;

    static const uint8_t digitPatterns[10] = {DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4,
                                              DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9};

    if (maxPwm > 255)
        maxPwm = 255;

    /* build into targetPwmPerBit buffer */
    for (uint8_t bit = 0; bit < 32; bit++)
    {
        uint8_t digit   = bit / 8;
        uint8_t pattern = 0;

        if (digit < 4 && digitValues[digit] != 0xFF)
            pattern = digitPatterns[digitValues[digit]];

        /* decimal point on digit 2 */
        if (digit == 2)
            pattern |= 0x40;

        targetPwmPerBit[bit] = (pattern & (1 << (bit % 8))) ? maxPwm : 0;
    }
}
void setDisplayFourDigitNumber(uint16_t number, uint8_t maxPwm)

{
    uint8_t digitValues[4];

    digitValues[0] = (number >= 1000) ? (number / 1000) % 10 : 0xFF;
    digitValues[1] = (number >= 100) ? (number / 100) % 10 : 0xFF;
    digitValues[2] = (number >= 10) ? (number / 10) % 10 : 0xFF;
    digitValues[3] = number % 10;

    static const uint8_t digitPatterns[10] = {DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3, DIGIT_4,
                                              DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9};

    if (maxPwm > PWM_STEPS)
        maxPwm = PWM_STEPS;

    /* build into inactive LUT — no IRQ lock needed */
    for (uint8_t pwm = 0; pwm < PWM_STEPS; pwm++)
    {
        for (uint8_t digit = 0; digit < 4; digit++)
        {
            uint8_t pattern = 0;

            if (digitValues[digit] != 0xFF)
                pattern = digitPatterns[digitValues[digit]];

            /* decimal point on digit 3 */
            if (digit == 2)
                pattern |= 0x40;

            uint8_t out          = (pwm < maxPwm) ? pattern : 0;
            buildLut[pwm][digit] = out;
        }
    }

    /* atomic LUT swap (VERY short critical section) */
    enterCriticalSection();
    activeLut = buildLut;
    buildLut  = (buildLut == pwmLutA) ? pwmLutB : pwmLutA;
    exitCriticalSection();
}
static const uint8_t gammaLut100[101] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0,   1,   1,   1,   1,   1,   2,   2,   2,   3,   3,  4,
    4,  5,  5,  6,  7,  7,  8,  9,  10, 11, 12,  13,  14,  15,  16,  17,  18,  20,  21,  23,  24,  26, 27,
    29, 31, 32, 34, 36, 38, 40, 42, 44, 46, 48,  50,  52,  55,  57,  59,  62,  64,  67,  69,  72,  74, 77,
    79, 82, 84, 87, 89, 92, 94, 96, 98, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

// Update buildPwmLut based on currentPwmPerBit
void updatePwmLUTs(void)
{
    // enterCriticalSection();

    uint8_t nextPwmLut = (currentPwmLut + 1) % 2;
    for (uint8_t pwmStep = 0; pwmStep < PWM_STEPS; pwmStep++)
    {
        for (uint8_t digit = 0; digit < 4; digit++)
        {
            uint8_t byteValue = 0;
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                uint8_t globalBitIndex = digit * 8 + bit;
                if (currentGammaPwmPerBit[globalBitIndex] > pwmStep)
                {
                    byteValue |= (1 << bit);
                }
            }
            buildLut[pwmStep][digit] = byteValue;
        }
    }
}

__attribute__((interrupt)) void SPI1_IRQHandler(void)
{

    if (LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        if (spiIndex == 0)
        {
            pushOutLow();
        }


        if (spiIndex < 4)
        {

            LL_SPI_TransmitData8(SPI1, activeLut[pseudoPwmIndex][spiIndex]);


            spiIndex++;
        }
        else
        {
            // All bytes have been sent, disable TXE interrupt
            LL_SPI_DisableIT_TXE(SPI1);
            pushOutHigh();


            spiIndex = 0; // Reset index for next transmission
        }
    }
}


#define COUNTER_INIT_VALUE 1234
#define COUNTER_MAX_VALUE  2389

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */
    uint8_t hours   = 20;
    uint8_t minutes = 39;

    uint16_t counter     = COUNTER_INIT_VALUE;
    uint8_t  loopCounter = 0;
    pwmValue             = 90;

    RTC_TimeTypeDef rtcTime = {0};
    RTC_DateTypeDef rtcDate = {0};

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, 3);

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    // disable rtc write protection

    rtcTime.Hours   = hours;
    rtcTime.Minutes = minutes;
    HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
    // HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BCD);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    dispResetHigh();

    LL_GPIO_ResetOutputPin(PWM_DIG_1_GPIO_Port, PWM_DIG_1_Pin);
    LL_GPIO_ResetOutputPin(PWM_DIG_2_GPIO_Port, PWM_DIG_2_Pin);
    LL_GPIO_ResetOutputPin(PWM_DIG_3_GPIO_Port, PWM_DIG_3_Pin);
    LL_GPIO_ResetOutputPin(PWM_DIG_4_GPIO_Port, PWM_DIG_4_Pin);

    uiTimeBaseTimerInit();
    uiTimeBaseTimerStart();
    while (1)
    {
        HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

        hours   = rtcTime.Hours;
        minutes = rtcTime.Minutes;
        counter = hours * 100 + minutes;
        setDisplayFourDigitNumber2(counter, pwmValue);
        if (pwmValue > 100)
        {
            pwmValue = 0;
        }
        loopCounter++;
        if (loopCounter == 200)
        {
            loopCounter = 0;
        }

        LL_mDelay(10);

        // follow currentPwmPerBit to targetPwmPerBit
        for (uint8_t i = 0; i < 32; i++)
        {
            if (currentPwmPerBit[i] < targetPwmPerBit[i])
            {
                currentPwmPerBit[i]++;
            }
            else if (currentPwmPerBit[i] > targetPwmPerBit[i])
            {
                currentPwmPerBit[i]--;
            }
            currentGammaPwmPerBit[i] = gammaLut100[currentPwmPerBit[i]];
        }


        updatePwmLUTs();
        swapPwmLut();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
    {
    }

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1)
    {
    }

    LL_PWR_EnableBkUpAccess();
    /* LSE configuration and activation */
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
    LL_RCC_LSE_Enable();
    while (LL_RCC_LSE_IsReady() != 1)
    {
    }

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    LL_Init1msTick(64000000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
