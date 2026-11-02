/**
  ******************************************************************************
  * @file    main.c
  * @brief   07-adc：ADC1 通道 0（PA0）单次转换，OLED 显示原始值和电压
  *
  * 硬件：
  *     光敏电阻模块 VCC → 3.3V
  *     光敏电阻模块 GND → GND
  *     光敏电阻模块 AO  → PA0（ADC1_IN0）   ★ 是 AO，不是 DO
  *     OLED              → PB8/PB9
  *
  * 换算：
  *     12 位 ADC → 0~4095
  *     电压(mV) = 原始值 × 3300 / 4095
  *
  * 现象：
  *     遮住 / 放开光敏电阻，Raw、mV、V 都会变化
  *     （变化方向取决于模块，可能越暗越大，也可能越小）
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================================
 * 一、ADC 初始化：ADC1 通道 0（PA0）
 * ==========================================================================*/
static void ADC_Init_PA0(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef  adc;

    /* ---------- ① 开时钟 ----------
       ADC1 挂 APB2，GPIOA 也挂 APB2 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    /* ---------- ② ADC 时钟分频：不能超过 14MHz ----------
       72MHz / 6 = 12MHz ✅
       如果选 Div2（36MHz），ADC 超频，读数不准 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* ---------- ③ PA0 配成模拟输入 ----------
       必须 AIN：断开数字输入电路，避免上拉/下拉干扰模拟信号 */
    gpio.GPIO_Pin  = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ④ ADC 基本参数：独立 + 单次 + 单通道 + 软件触发 + 右对齐 ---------- */
    adc.ADC_Mode               = ADC_Mode_Independent;      /* 只用 ADC1 */
    adc.ADC_ScanConvMode       = DISABLE;                   /* 不扫描（单通道） */
    adc.ADC_ContinuousConvMode = DISABLE;                   /* 单次转换 */
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; /* 软件触发 */
    adc.ADC_DataAlign          = ADC_DataAlign_Right;       /* 右对齐：读出来就是 0~4095 */
    adc.ADC_NbrOfChannel       = 1;                         /* 规则组 1 个通道 */
    ADC_Init(ADC1, &adc);

    /* ---------- ⑤ 规则通道：通道 0、第 1 个、采样 55.5 周期 ----------
       光敏电阻内阻较大，采样时间给长一点，让采样电容充满 */
    ADC_RegularChannelConfig(ADC1,
                             ADC_Channel_0,
                             1,
                             ADC_SampleTime_55Cycles5);

    /* ---------- ⑥ 使能 ADC1 ---------- */
    ADC_Cmd(ADC1, ENABLE);

    /* ---------- ⑦ 校准（必须） ----------
       复位校准 → 等完成 → 开始校准 → 等完成 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

/**
  * @brief  读一次 ADC
  * @return 0~4095
  */
static uint16_t ADC_Read(void)
{
    /* ① 软件启动一次转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* ② 等转换结束（EOC = End Of Conversion） */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    /* ③ 读数据（读 DR 会自动清 EOC） */
    return ADC_GetConversionValue(ADC1);
}

/* ============================================================================
 * 二、main
 * ==========================================================================*/
int main(void)
{
    uint16_t raw;       /* 0~4095 */
    uint32_t mv;        /* 毫伏 0~3300 */

    OLED_Init();
    OLED_ShowString(1, 1, "Raw:");
    OLED_ShowString(2, 1, "mV:");
    OLED_ShowString(3, 1, "V:");
    OLED_ShowString(4, 1, "PA0 Light");

    ADC_Init_PA0();

    while (1)
    {
        raw = ADC_Read();                       /* 读原始值 0~4095 */
        mv  = (uint32_t)raw * 3300 / 4095;      /* 换算成毫伏 */

        /* 第 1 行：原始值（4 位） */
        OLED_ShowNum(1, 6, raw, 4);

        /* 第 2 行：毫伏（4 位） */
        OLED_ShowNum(2, 5, mv, 4);

        /* 第 3 行：电压，保留两位小数，例如 2.15 */
        OLED_ShowNum(3, 4, mv / 1000, 1);
        OLED_ShowString(3, 5, ".");
        OLED_ShowNum(3, 6, (mv % 1000) / 10, 2);

        Delay_ms(100);                          /* 100ms 刷新一次 */
    }
}
