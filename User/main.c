/**
  ******************************************************************************
  * @file    main.c
  * @brief   08-adc-dma：双通道 ADC + DMA + 定时器触发
  *
  * 硬件：
  *     光敏电阻 AO → PA0（ADC1_IN0）
  *     热敏电阻 AO → PA1（ADC1_IN1）
  *     OLED        → PB8/PB9
  *
  * 数据流：
  *     TIM3（1kHz）──TRGO──→ ADC1（扫 PA0 → PA1）
  *                              │ 每转完一个通道发 DMA 请求
  *                              ↓
  *                        DMA1_Channel1
  *                              ↓
  *                        adc_buf[0] / adc_buf[1]
  *
  * 要点：
  *     · 通道固定：ADC1 只能用 DMA1_Channel1
  *     · 外设地址不自增、内存地址自增、半字、循环模式
  *     · 必须开 ADC_DMACmd 和 ADC_ExternalTrigConvCmd
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================================
 * 一、全局变量
 * ==========================================================================*/

/**
  * @brief DMA 搬运目标：adc_buf[0] = 光敏，adc_buf[1] = 热敏
  * @note  volatile 必须加：DMA 在后台修改，主循环每次都要读内存
  */
volatile uint16_t adc_buf[2] = {0, 0};

/* ============================================================================
 * 二、DMA 初始化
 * ==========================================================================*/
static void DMA_Init_ADC(void)
{
    DMA_InitTypeDef dma;

    /* ① DMA1 挂在 AHB 总线上 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* ② 复位通道（清掉残留配置） */
    DMA_DeInit(DMA1_Channel1);              /* ADC1 固定用通道 1 */

    /* ③ 配置搬运规则 */
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;    /* 源：ADC 数据寄存器 */
    dma.DMA_MemoryBaseAddr     = (uint32_t)adc_buf;      /* 目的：数组首地址 */
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;  /* 外设 → 内存 */
    dma.DMA_BufferSize         = 2;                      /* 搬 2 个（两个通道） */

    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable; /* DR 地址不变 */
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;      /* 数组下标递增 */

    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* 16 位 */
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;

    dma.DMA_Mode               = DMA_Mode_Circular;      /* 搬完从头再来 */
    dma.DMA_Priority           = DMA_Priority_Medium;
    dma.DMA_M2M                = DMA_M2M_Disable;        /* 非内存到内存 */

    DMA_Init(DMA1_Channel1, &dma);

    /* ④ 使能通道 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

/* ============================================================================
 * 三、ADC 初始化：双通道扫描 + 定时器触发
 * ==========================================================================*/
static void ADC_Init_Dual(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef  adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    /* ADC 时钟不能超过 14MHz：72/6 = 12MHz */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* 两个引脚都配成模拟输入 */
    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* 扫描开、连续关（由定时器触发）、外部触发选 TIM2_TRGO */
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = ENABLE;                        /* 多通道扫描 */
    adc.ADC_ContinuousConvMode = DISABLE;                       /* 单次，靠触发 */
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  /* TIM3 触发 */
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 2;                             /* 两个通道 */
    ADC_Init(ADC1, &adc);

    /* 扫描序列：rank1 = PA0，rank2 = PA1 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);

    /* 使能 + 校准 */
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);

    /* ★ 允许 ADC 发 DMA 请求 */
    ADC_DMACmd(ADC1, ENABLE);

    /* ★ 允许 ADC 接受外部（定时器）触发 */
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);
}

/* ============================================================================
 * 四、TIM3：1kHz，输出更新事件到 TRGO
 * ==========================================================================*/
static void TIM_Init_Trigger(void)
{
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 72MHz / 72 = 1MHz，1000 个计数 = 1ms → 1kHz */
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_Period            = 1000 - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    /* 把"更新事件"送到 TRGO，供 ADC 当触发源 */
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

    TIM_Cmd(TIM3, ENABLE);
}

/* ============================================================================
 * 五、main
 * ==========================================================================*/
int main(void)
{
    uint16_t raw0, raw1;
    uint32_t mv0, mv1;

    OLED_Init();
    OLED_ShowString(1, 1, "L:");       /* 光敏原始值 */
    OLED_ShowString(2, 1, "LmV:");     /* 光敏电压 */
    OLED_ShowString(3, 1, "T:");       /* 热敏原始值 */
    OLED_ShowString(4, 1, "TmV:");     /* 热敏电压 */

    DMA_Init_ADC();          /* 先配搬运路线 */
    ADC_Init_Dual();         /* 再配采集 */
    TIM_Init_Trigger();      /* 最后启动节拍 */

    while (1)
    {
        /* DMA 在后台不断更新 adc_buf，这里直接读最新值 */
        raw0 = adc_buf[0];
        raw1 = adc_buf[1];

        mv0 = (uint32_t)raw0 * 3300 / 4095;
        mv1 = (uint32_t)raw1 * 3300 / 4095;

        OLED_ShowNum(1, 3, raw0, 4);
        OLED_ShowNum(2, 5, mv0, 4);
        OLED_ShowNum(3, 3, raw1, 4);
        OLED_ShowNum(4, 5, mv1, 4);

        Delay_ms(100);
    }
}
