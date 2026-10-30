/**
  ******************************************************************************
  * @file    main.c
  * @brief   05-capture（硬件捕获版）：TIM3 PWM 输入模式测频率/占空比
  *
  * 硬件：
  *     PA1 —— TIM2_CH2 输出 PWM（信号源，1kHz）
  *     PA6 —— TIM3_CH1 输入捕获（测量脚）
  *     PA1 用杜邦线接到 PA6
  *     OLED —— PB8/PB9
  *
  * 硬件捕获原理（PWM 输入模式 PWMI）：
  *     上升沿到达时，定时器硬件做两件事：
  *         ① 把当前 CNT 存入 CCR1（周期的终点）
  *         ② 通过从模式 Reset，把 CNT 清零（下一个周期的起点）
  *     CH2 被 TIM_PWMIConfig 自动配成"下降沿"，把下降沿时刻存入 CCR2
  *         ↓
  *     CCR1 = 一个完整周期的计数（1MHz 下 1 计数 = 1µs）
  *     CCR2 = 高电平的计数
  *         ↓
  *     频率   = 1,000,000 / CCR1
  *     占空比 = CCR2 / CCR1
  *
  * 和"软件捕获（EXTI + 读 CNT）"的区别：
  *     边沿时刻由硬件记录，不受中断延迟影响 → 高频也准
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

#define PWM_PERIOD   1000       /* ARR + 1，1kHz */

/* ---------- 测量结果（中断写，主循环读） ---------- */
volatile uint32_t g_period_us = 0;   /* 周期（µs） */
volatile uint32_t g_freq_hz   = 0;   /* 频率（Hz） */
volatile uint32_t g_duty      = 0;   /* 占空比（%） */

/* ============================================================================
 * 一、信号源：TIM2_CH2（PA1）输出 1kHz PWM
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

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 500;               /* 初始 50% */
    TIM_OC2Init(TIM2, &oc);

    /* 预装载：改 CCR 在周期边界生效，波形不撕裂 */
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
 * 二、测量端：TIM3_CH1（PA6）PWM 输入模式
 * ==========================================================================*/
static void IC_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef       ic;
    NVIC_InitTypeDef        nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* ---------- ① PA6 上拉输入 ----------
       捕获是"读引脚"，所以是输入模式（不是 AF_PP） */
    gpio.GPIO_Pin  = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ② 时基：1MHz，满量程 65535 ----------
       1 计数 = 1µs；能测的最长周期 65.5ms（最低约 15Hz） */
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_Period            = 65535;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    /* ---------- ③ 配置捕获通道 ----------
       TIM_PWMIConfig 会自动：
         CH1 → 上升沿、直连（TI1 → IC1）
         CH2 → 下降沿、交叉（TI1 → IC2）
       即"同一个引脚的两个边沿，被两个通道分别记录" */
    TIM_ICStructInit(&ic);
    ic.TIM_Channel     = TIM_Channel_1;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;    /* 每个边沿都捕获 */
    ic.TIM_ICFilter    = 0x0F;              /* 数字滤波，抗毛刺 */
    TIM_PWMIConfig(TIM3, &ic);

    /* ---------- ④ 从模式：上升沿时清零计数器 ----------
       ★ 关键：不清零的话，CCR1 只是"某个时刻的计数值"，
         清零之后 CCR1 才正好等于"一个周期的长度" */
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);     /* 触发源 = CH1 */
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);  /* 触发时复位 CNT */

    /* ---------- ⑤ 开捕获中断 ---------- */
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);

    TIM_Cmd(TIM3, ENABLE);
}

/* ============================================================================
 * 三、捕获中断：每来一个上升沿进一次
 * ==========================================================================*/
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
    {
        /* 上升沿时硬件已经把两个值准备好了：
             CCR1 = 从上次清零到这次上升沿的计数 = 周期
             CCR2 = 从这次上升沿到下降沿的计数   = 高电平时间 */
        uint16_t period = TIM_GetCapture1(TIM3);
        uint16_t high   = TIM_GetCapture2(TIM3);

        if (period != 0)
        {
            g_period_us = period;
            g_freq_hz   = 1000000UL / period;              /* 1µs/格 */
            g_duty      = (uint32_t)high * 100UL / period;
        }

        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
}

/* ============================================================================
 * 四、main：输出占空比扫描 + 显示测量值
 * ==========================================================================*/
int main(void)
{
    uint16_t set_duty = 500;      /* 输出占空比（0~1000） */
    int8_t   step     = 25;

    OLED_Init();
    OLED_ShowString(1, 1, "Set:");   /* 我设置的占空比 */
    OLED_ShowString(2, 1, "F:");     /* 测到的频率 */
    OLED_ShowString(3, 1, "D:");     /* 测到的占空比 */
    OLED_ShowString(4, 1, "T:");     /* 测到的周期 */

    PWM_Out_Init();
    IC_Init();

    while (1)
    {
        /* 让输出占空比在 10%~90% 之间来回变化，便于观察跟踪 */
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
