/**
  ******************************************************************************
  * @file    main.c
  * @brief   06-encoder（前置验证）：EC11 旋转编码器 + TIM3 编码器模式
  *
  * 硬件（EC11 模块 5 个脚）：
  *     EC11 VCC → 3.3V
  *     EC11 GND → GND
  *     EC11 C   → GND      ★ 公共端，不接就完全不动
  *     EC11 A   → PA6      （TIM3_CH1）
  *     EC11 B   → PA7      （TIM3_CH2）
  *     OLED     → PB8/PB9
  *
  * 现象：
  *     顺时针拧一格 → Clk +1，Dir 显示 CW
  *     逆时针拧一格 → Clk -1，Dir 显示 CCW
  *     不拧         → 数字不动
  *
  * 原理：
  *     TIM3 配成编码器模式 3（4 倍频）
  *     硬件根据 A、B 的先后自动给 CNT 加 1 或减 1
  *     一个"咔哒"档位 = 一个完整周期 = 4 个边沿 = 4 个计数
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================================
 * 一、编码器初始化：TIM3，PA6/PA7，4 倍频
 * ==========================================================================*/
static void Encoder_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef       ic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* ---------- ① PA6、PA7 上拉输入 ----------
       编码器是"开关"：闭合时接地，断开时悬空
       必须上拉，否则悬空时电平不定，计数乱跳 */
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ② 时基：不分频，满量程 ----------
       编码器模式下 PSC 一般设 0：每个边沿都算
       ARR 设最大：让 CNT 有足够计数范围 */
    tb.TIM_Prescaler         = 0;
    tb.TIM_Period            = 65535;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    /* ---------- ③ 编码器模式 3：TI1、TI2 边沿都计数（4 倍频） ----------
       模式 1/2 = 2 倍频（只数一路）
       模式 3   = 4 倍频（两路四个边沿都数），分辨率最高 */
    TIM_EncoderInterfaceConfig(TIM3,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    /* ---------- ④ 输入滤波 ----------
       EC11 是机械触点，转动时有 1~5ms 抖动
       不滤波会"拧一格跳好几下"
       0x0F 是最强滤波 */
    TIM_ICStructInit(&ic);
    ic.TIM_Channel     = TIM_Channel_1;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x0F;
    TIM_ICInit(TIM3, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &ic);

    /* ---------- ⑤ 启动 ---------- */
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/* ============================================================================
 * 二、main
 * ==========================================================================*/
int main(void)
{
    uint16_t last  = 0;      /* 上次读到的 CNT */
    int32_t  total = 0;      /* 累计计数（正=正转，负=反转） */
    int16_t  delta;          /* 本次增量（带方向） */
    uint32_t abs_total;

    OLED_Init();
    OLED_ShowString(1, 1, "Cnt:");   /* 累计计数（4 倍频） */
    OLED_ShowString(2, 1, "Dir:");   /* 方向 */
    OLED_ShowString(3, 1, "Clk:");   /* 档位数 */
    OLED_ShowString(4, 1, "EC11");

    Encoder_Init();

    while (1)
    {
        uint16_t now;

        Delay_ms(20);                        /* 20ms 采样一次 */

        now   = TIM_GetCounter(TIM3);
        delta = (int16_t)(now - last);       /* 带方向的增量 */
        last  = now;

        total += delta;                      /* 累加 */

        abs_total = (uint32_t)(total < 0 ? -total : total);

        /* ---------- 显示 ---------- */
        OLED_ShowNum(1, 5, abs_total, 5);

        if (delta > 0)      OLED_ShowString(2, 5, "CW ");
        else if (delta < 0) OLED_ShowString(2, 5, "CCW");
        else                OLED_ShowString(2, 5, "---");

        /* 一个"咔哒"档位 = 4 个计数（4 倍频） */
        OLED_ShowNum(3, 5, abs_total / 4, 5);
    }
}
