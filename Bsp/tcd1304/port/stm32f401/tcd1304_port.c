/**
 *******************************************************************************
 * @file    : tcd1304_port.c
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-22
 * @brief   : Portable layer of the driver for the CCD sensor chip from Toshiba
 *
 * This portable driver is implemented for STM32F746-Discovery board using
 * pins on the Arduino Uno connector:
 * A0 = ICG
 * A2 = SH
 * A3 = fM
 * A5 = Sensor Output
 *
 *******************************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2018 Dung Do Dang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *******************************************************************************
 */

/**
 ***************************** Revision History ********************************
 * 2018-06-22 revision 0: Initial version
 *
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "tcd1304_port.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t f_master;
    uint32_t t_int_us;
    uint32_t t_icg_us;
    uint32_t f_adc;
} PORT_TIMER_CONF_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim8;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim5;
static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;
static PORT_TIMER_CONF_t timer_conf;

/* Private function prototypes -----------------------------------------------*/
static void TCD_PORT_EnableADCTrigger(void);
static void TCD_PORT_DisableADCTrigger(void);
static void TCD_PORT_ICG_SetDelay(uint32_t cnt);
static void TCD_PORT_SH_SetDelay(uint32_t cnt);

/**
 *******************************************************************************
 *                        PUBLIC IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Start the timers to generate fM, ICG and SH pulses
 * @param   None
 * @retval  None
 ******************************************************************************/
void TCD_PORT_Run(void)
{
    TCD_PORT_ICG_SetDelay( CFG_ICG_DEFAULT_PULSE_DELAY_CNT );
    TCD_PORT_SH_SetDelay( CFG_SH_DEFAULT_PULSE_DELAY_CNT );

    /* Disable the DMA transfer half complete interrupt */
    __HAL_DMA_DISABLE_IT( &hdma_adc1, DMA_IT_HT );

    /* Reset the timer counters to 0 */
    htim2.Instance->CNT = 0U;
    htim5.Instance->CNT = 0U;
    htim3.Instance->CNT = 0U;

    __HAL_TIM_ENABLE( &htim2 );         /* ICG TIMER */
    __HAL_TIM_ENABLE( &htim5 );        /* SH TIMER */
    __HAL_TIM_ENABLE( &htim3 );        /* fM TIMER */
}

/*******************************************************************************
 * @Brief   Stop the timers
 * @param   None
 * @retval  None
 ******************************************************************************/
void TCD_PORT_Stop(void)
{
    htim2.Instance->CR1 &= ~TIM_CR1_CEN;         /* ICG TIMER */
    htim5.Instance->CR1 &= ~TIM_CR1_CEN;        /* SH TIMER */
    htim3.Instance->CR1 &= ~TIM_CR1_CEN;        /* fM TIMER */
}

/*******************************************************************************
 * @brief   Configure the CCD master clock
 * @param   freq, uint32_t: Clock frequency in MHz
 * @retval  Error code
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigMasterClock(uint32_t freq)
{
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t period = HAL_RCC_GetSysClockFreq() / freq - 1U;
    timer_conf.f_master = freq;

    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* TIM3 GPIO Configuration. PB0------> TIM3_CH3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = period;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#ifdef TIM_AUTORELOAD_PRELOAD_DISABLE
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

    if ( HAL_TIM_Base_Init( &htim3 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim3 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 2U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim3, &sConfigOC, TIM_CHANNEL_3 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Check the parameters */
    assert_param( IS_TIM_CCX_INSTANCE(htim3.Instance, TIM_CHANNEL_3) );

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd( htim3.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE );

    if ( IS_TIM_ADVANCED_INSTANCE( htim3.Instance ) != RESET )
    {
        /* Enable the main output */
        __HAL_TIM_MOE_ENABLE( &htim3 );
    }

    return err;
}

/*******************************************************************************
 * @brief   Configure the ICG pulse generator with interrupt request
 * @param   t_int_us, uint32_t: Sensor readout period in microseconds
 * @retval  Error code
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigICGClock(const uint32_t t_icg_us)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = HAL_RCC_GetSysClockFreq() / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = (uint32_t) ((uint64_t) t_icg_us * CFG_FM_FREQUENCY_HZ / 1000000U) - 1U;
    uint32_t pulse = CFG_ICG_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;
    timer_conf.t_icg_us = t_icg_us;

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TIM2 GPIO Configuration. PA0------> TIM2_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = period;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#ifdef TIM_AUTORELOAD_PRELOAD_DISABLE
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

    if ( HAL_TIM_Base_Init( &htim2 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if ( HAL_TIM_ConfigClockSource( &htim2, &sClockSourceConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim2 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if ( HAL_TIMEx_MasterConfigSynchronization( &htim2, &sMasterConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim2, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Set TIM2 interrupt level and enable interrupt for TIM2 */
    HAL_NVIC_SetPriority( TIM2_IRQn, TIM_ICG_INTERRUPT_LEVEL, 0 );
    HAL_NVIC_EnableIRQ( TIM2_IRQn );

    /* Check the parameters */
    assert_param( IS_TIM_CCX_INSTANCE(htim2.Instance, TIM_CHANNEL_1) );

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd( htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE );

    if ( IS_TIM_ADVANCED_INSTANCE( htim2.Instance ) != RESET )
    {
        /* Enable the main output */
        __HAL_TIM_MOE_ENABLE( &htim2 );
    }

    /* Enable the TIM Capture/Compare 1 interrupt */
    __HAL_TIM_ENABLE_IT( &htim2, TIM_IT_CC1 );

    return err;
}

/*******************************************************************************
 * @brief   Configure the SH pulse generator
 * @param   t_int_us, uint32_t: Integration time for the CCD in microseconds
 * @retval  Error code
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigSHClock(const uint32_t t_int_us)
{
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / 2U) / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = t_int_us * CFG_FM_FREQUENCY_HZ / 1000000U - 1U;
    uint32_t pulse = CFG_SH_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;
    timer_conf.t_int_us = t_int_us;

    /* Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TIM5 GPIO Configuration. PA1------> TIM5_CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = prescaler;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = period;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
#ifdef TIM_AUTORELOAD_PRELOAD_DISABLE
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
#endif

    if ( HAL_TIM_Base_Init( &htim5 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim5 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim5, &sConfigOC, TIM_CHANNEL_2 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Check the parameters */
    assert_param( IS_TIM_CCX_INSTANCE(htim3.Instance, TIM_CHANNEL_2) );

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd( htim5.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE );

    if ( IS_TIM_ADVANCED_INSTANCE( htim5.Instance ) != RESET )
    {
        /* Enable the main output */
        __HAL_TIM_MOE_ENABLE( &htim5 );
    }

    return err;
}

/*******************************************************************************
 * @brief   Configure the ADC trigger as One-Pulse-Timer with 3693 repetitions
 * @param   Fs, uint32_t: ADC sampling frequency
 * @retval  None
 *
 * Background:
 * The ICG timer generates an ICG pulse. When this timer overflows, an interrupt is
 * generated. In this interrupt handler, the ADC Trigger timer is started.
 * Remember that the DMA is connected to the ADC in CIRCULAR mode. When the DMA has
 * transfered 3694 ADC samples to RAM, a new interrupt is generated. In this
 * interrupt handler the ADC trigger timer MUST be disabled to avoid false restart of
 * ADC sampling.
 *
 * The problem:
 * The ADC runs up to 1 MSamples/s with 4 MHz master clock. This means that when the
 * last sample of the ADC is transfered by the DMA, we have 1 us to disable the ADC
 * trigger timer to stop the DMA. The STM32F7 running at 216 MHz from FLASH needs
 * approximately 100 ns to start execution in interrupt handlers.
 *
 * In an MCU system it can possibly be many interrupts enabled and with different
 * interrupt priority levels. Also, the interrupts can be temporary disabled for
 * a short time to handle a critical section in the software.
 * This can lead to the situation where the DMA has restarted a new transfer to RAM.
 * How far the DMA has moved the data is dependent on how long time the
 * interrupt handler is delayed. This causes misaligned spectrum data.
 *
 * Solution:
 * The STM32 timers has a function called One-Pulse-Timer. This timer is programmed
 * to generate 3694 pulses when triggered. We use software trigger, by enabling the
 * CEN bit in timer CR1 register. When this timer has generated 3694 pulses, it
 * automatically shuts down the counter by disabling the CEN bit in its own CR1
 * register. This eliminates the above problem, and the interrupt response time is no
 * longer an issue. This commit utilize this nice hardware function to avoid
 * any potential issues with data alignment.
 *
 ******************************************************************************/
void TCD_PORT_ConfigADCTrigger(uint32_t f_adc)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    uint32_t period = HAL_RCC_GetSysClockFreq() / f_adc - 1U;
    timer_conf.f_adc = f_adc;

    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TIM1 GPIO Configuration. PA8------> TIM1_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    htim8.Instance = TIM1;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = period;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
#ifdef TIM_AUTORELOAD_PRELOAD_DISABLE
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
#endif

    if ( HAL_TIM_Base_Init( &htim8 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if ( HAL_TIM_ConfigClockSource( &htim8, &sClockSourceConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim8 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;

    if ( HAL_TIMEx_MasterConfigSynchronization( &htim8, &sMasterConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 10U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if ( HAL_TIM_PWM_ConfigChannel( &htim8, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if ( HAL_TIMEx_ConfigBreakDeadTime( &htim8, &sBreakDeadTimeConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd( htim8.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE );

    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE( &htim8 );
}

/*******************************************************************************
 * @brief   Configure the ADC with 12 bit and 3 cycles sampling time
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
int32_t TCD_PORT_InitADC(void)
{
    ADC_ChannelConfTypeDef sConfig;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;

    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ADC input pin is connected to PC0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    /* ADC3 DMA Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if ( HAL_DMA_Init( &hdma_adc1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    __HAL_LINKDMA( &hadc1, DMA_Handle, hdma_adc1 );

    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DMA2_Stream0_IRQn, DMA_ADC_INTERRUPT_LEVEL, 0 );
    HAL_NVIC_EnableIRQ( DMA2_Stream0_IRQn );

    /**
     * Configure the global features of the ADC
     * (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

    if ( HAL_ADC_Init( &hadc1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /**
     * Configure for the selected ADC regular channel its
     * corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_10;
#ifdef ADC_REGULAR_RANK_1
    sConfig.Rank = ADC_REGULAR_RANK_1;
#else
    sConfig.Rank = 1U;
#endif

    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    return err;
}

/*******************************************************************************
 * @brief   Start the ADC with DMA transfer
 * @param   *dataBuffer, uint16_t: Pointer to the data location in RAM
 * @retval  Error codes
 *
 ******************************************************************************/
int32_t TCD_PORT_StartADC(uint16_t *dataBuffer)
{
    if ( dataBuffer == NULL )
    {
        return -1;
    }

    return (int32_t) HAL_ADC_Start_DMA( &hadc1, (uint32_t *) dataBuffer, CFG_CCD_NUM_PIXELS );
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/**
 * The compiler is smart to inline these functions calls into where it is called
 * to remove function call overheads.
 */
/*******************************************************************************
 * @brief   Enable the timer that generates ADC trigger signal
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_EnableADCTrigger(void)
{
    TCD_ADC_TRIG_TIMER->CR1 |= TIM_CR1_CEN;
}

/*******************************************************************************
 * @brief   Disable the timer that generates ADC trigger signal
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_DisableADCTrigger(void)
{
    TCD_ADC_TRIG_TIMER->CR1 &= ~TIM_CR1_CEN;
}

/*******************************************************************************
 * @brief   Set a delay to the ICG pulse with the given timer counter value
 * @param   cnt, uint32_t. Timer counter value to delay
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_ICG_SetDelay(uint32_t cnt)
{
    TCD_ICG_TIMER->CNT = cnt;
}

/*******************************************************************************
 * @brief   Set a delay to the SH pulse with the given timer counter value
 * @param   cnt, uint32_t. Timer counter value to delay
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_SH_SetDelay(uint32_t cnt)
{
    TCD_ICG_TIMER->CNT = cnt;
}

/**
 *******************************************************************************
 *                         INTERRUPT HANDLERS
 *******************************************************************************
 */

/*******************************************************************************
 * @brief   This function handles ICG PULSE interrupt.
 * @param   None
 * @retval  None
 *
 * When the ICG TIMER has generated an ICG pulse to start moving out charges
 * from the CCD, an interrupt request (TIMx_IRQ) is generated.
 *
 * This interrupt handler function is to start the ADC+DMA acquisition of
 * CFG_CCD_NUM_PIXELS ADC samples. This is 3694 for the TCD1304 sensor.
 * When this acquisition is finished, a new interrupt is generated;
 * TCD_CCD_ADC_INTERRUPT_HANDLER().
 *
 ******************************************************************************/
void TCD_ICG_TIMER_INTERRUPT_HANDLER(void)
{
    TCD_PORT_EnableADCTrigger();

    HAL_TIM_IRQHandler( &htim2 );
}

/*******************************************************************************
 * @brief   This function handles ADC+DMA acquisition complete interrupt.
 * @param   None
 * @retval  None
 *
 * The ADC+DMA acquisition of CFG_CCD_NUM_PIXELS samples were started in
 * TCD_ICG_TIMER_INTERRUPT_HANDLER().
 *
 * When the DMA transfer has completed an interrupt request (DMAX_StreamX_IRQ)
 * is generated.
 * This is the interrupt handler for that request. Following is done:
 * 1) Disable the ADC trigger signal (TCD_ADC_TRIG_TIMER).
 * 2) Let the HAL layer handle the DMA interrupt request
 * 3) Call the user callback function to deal with the acquired ADC samples.
 *
 ******************************************************************************/
void TCD_CCD_ADC_INTERRUPT_HANDLER(void)
{
    TCD_PORT_DisableADCTrigger();
    
    if ( hdma_adc1.Instance->NDTR != CFG_CCD_NUM_PIXELS )
    {
        __BKPT( 0U );
    }
    
    HAL_DMA_IRQHandler( &hdma_adc1 );

    /* Do something with the acquired AD samples in RAM */
    TCD_ReadCompletedCallback();
}

/**
 * The user application must provide an implementation to trap the application
 * for debugging purposes.
 */
__weak void _Error_Handler(char *file, int line)
{
    /* User should implement this function some else */
}
/****************************** END OF FILE ***********************************/
