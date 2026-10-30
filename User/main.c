/**
  ******************************************************************************
  * @file    main.c
  * @brief   05-capture（软件捕获版）：EXTI 双边沿 + TIM4 读时间，测频率/占空比
  *
  * 硬件：
  *     PA1 —— TIM2_CH2 输出 PWM（信号源，1kHz）
  *     PA6 —— EXTI6 双边沿中断（测量脚）
  *     PA1 用杜邦线接到 PA6
  *     OLED —— PB8/PB9
  *
  * 思路（这是"软件捕获"）：
  *     TIM4 配成 1MHz 自由计数（1 格 = 1µs），只当秒表，不产生中断
  *     PA6 双边沿触发 EXTI6
  *     中断里读 TIM4->CNT：
  *         上升沿 → 与上次上升沿相减 = 周期
  *         下降沿 → 与本次上升沿相减 = 高电平时间
  *     频率   = 1,000,000 / 周期
  *     占空比 = 高电平时间 / 周期
  *
  * 注意：
  *     · 边沿发生后 CPU 要过几微秒才进中断，所以读数"偏晚"
  *       → 低频没事，高频误差会变大（对比 05 教程里的硬件捕获）
  *     · PA6 属于 EXTI 线 6，它没有独立向量，
  *       必须用 EXTI9_5_IRQn（线 5~9 共享一个中断）
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

#define PWM_PERIOD      1000      /* ARR+1，1kHz */
#define GLITCH_US       10        /* 小于 10µs 的间隔当毛刺扔掉 */

/* ---------- 测量结果（中断写，主循环读） ---------- */
volatile uint32_t g_period_us = 0;   /* 周期（µs） */
volatile uint32_t g_high_us   = 0;   /* 高电平时间（µs） */
volatile uint32_t g_freq_hz   = 0;   /* 频率（Hz） */
volatile uint32_t g_duty      = 0;   /* 占空比（%） */

/* ---------- 中断内部状态 ---------- */
static volatile uint16_t s_last_rise = 0;   /* 上次上升沿的时刻 */
static volatile uint8_t  s_have_rise = 0;   /* 是否已经见过上升沿 */

/* ============================================================================
 * 一、PWM 输出：TIM2_CH2（PA1），1kHz
 * ==========================================================================*/
static void PWM_Out_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* PA1 复用推挽：波形由定时器产生 */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 1MHz 计数，1000 个一周期 → 1kHz */
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_Period            = PWM_PERIOD - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    /* 通道 2 输出 PWM */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 500;                  /* 初始 50% */
    TIM_OC2Init(TIM2, &oc);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/** @brief 设置输出占空比，0~1000 */
static void PWM_SetDuty(uint16_t duty)
{
    if (duty > PWM_PERIOD) duty = PWM_PERIOD;
    TIM_SetCompare2(TIM2, duty);
}

/* ============================================================================
 * 二、秒表：TIM4 自由计数，1MHz（不产生任何中断）
 * ==========================================================================*/
static void TimeBase_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    tb.TIM_Prescaler         = 72 - 1;    /* 1MHz → 1 格 1µs */
    tb.TIM_Period            = 65535;     /* 16 位满量程，回绕后继续数 */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);                /* 一直数，不清零 */
}

/* ============================================================================
 * 三、EXTI6：PA6 双边沿中断
 * ==========================================================================*/
static void Exti_Init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    /* ① 开 GPIOA + AFIO 时钟（EXTI 选路必须开 AFIO） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* ② PA6 上拉输入（和 05 一样，空闲保持高电平） */
    gpio.GPIO_Pin  = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* ③ AFIO 选路：EXTI6 ← PA6 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

    /* ④ EXTI6：上升沿和下降沿都触发 */
    exti.EXTI_Line    = EXTI_Line6;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    /* ⑤ NVIC：★ 线 6 没有独立向量，用 EXTI9_5_IRQn（线 5~9 共用） */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);
}

/* ============================================================================
 * 四、中断：每次边沿进来，读 TIM4 算时间
 * ==========================================================================*/
void EXTI9_5_IRQHandler(void)
{
    /* 线 5~9 共用一个中断，先确认是线 6 触发的 */
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        uint16_t now = TIM_GetCounter(TIM4);            /* 当前时刻（µs） */
        uint8_t  lvl = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);

        if (lvl)
        {
            /* ===== 上升沿：算一个完整周期 ===== */
            if (s_have_rise)
            {
                uint16_t period = (uint16_t)(now - s_last_rise);

                /* 太短说明是毛刺，丢弃 */
                if (period >= GLITCH_US)
                {
                    g_period_us = period;
                    g_freq_hz   = 1000000UL / period;
                }
            }

            s_last_rise = now;
            s_have_rise = 1;
        }
        else
        {
            /* ===== 下降沿：算高电平时间 ===== */
            if (s_have_rise)
            {
                uint16_t high = (uint16_t)(now - s_last_rise);

                /* 必须比一个周期短，否则是干扰 */
                if ((g_period_us >= GLITCH_US) && (high <= g_period_us))
                {
                    g_high_us = high;
                    g_duty    = (uint32_t)high * 100UL / g_period_us;
                }
            }
        }

        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

/* ============================================================================
 * 五、main：扫描输出占空比 + 显示测量结果
 * ==========================================================================*/
int main(void)
{
    uint16_t set_duty = 500;      /* 输出占空比 0~1000 */
    int8_t   step     = 25;

    OLED_Init();
    OLED_ShowString(1, 1, "Set:");   /* 我设置的 */
    OLED_ShowString(2, 1, "F:");     /* 测到的频率 */
    OLED_ShowString(3, 1, "D:");     /* 测到的占空比 */
    OLED_ShowString(4, 1, "T:");     /* 测到的周期 */

    PWM_Out_Init();
    TimeBase_Init();
    Exti_Init();

    while (1)
    {
        /* 让输出在 10%~90% 之间来回扫 */
        if (set_duty >= 900) step = -25;
        if (set_duty <= 100) step =  25;
        set_duty += step;

        PWM_SetDuty(set_duty);

        /* ---- 显示 ---- */
        OLED_ShowNum(1, 5, (uint32_t)set_duty * 100 / PWM_PERIOD, 3);
        OLED_ShowString(1, 8, "%");

        OLED_ShowNum(2, 3, g_freq_hz, 5);
        OLED_ShowString(2, 9, "Hz");

        OLED_ShowNum(3, 3, g_duty, 3);
        OLED_ShowString(3, 7, "%");

        OLED_ShowNum(4, 3, g_period_us, 5);
        OLED_ShowString(4, 9, "us");

        Delay_ms(100);
    }
}
