/**
  ******************************************************************************
  * @file    main.c
  * @brief   04-pwm：TIM2 通道 2（PA1）输出 1kHz PWM，做呼吸灯
  *
  * 硬件：
  *     PA1 —— LED（低电平点亮）
  *
  * 时间计算：
  *     72MHz ÷ (PSC+1) = 72MHz ÷ 72 = 1MHz   → 每格 1µs
  *     (ARR+1) × 1µs = 1000 × 1µs = 1ms      → 1kHz
  *
  * 占空比：
  *     CCR = 0    → 0%
  *     CCR = 1000 → 100%
  *     占空比 = CCR / 1000
  *
  * 注意：
  *     本板 LED 低电平点亮，所以 CCR 越大（高电平越多）灯越暗。
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "Delay.h"

/* 一个 PWM 周期的计数值 = ARR + 1 */
#define PWM_PERIOD   3000

/* ============================================================================
 * PWM 初始化：TIM2_CH2（PA1），1kHz
 * ==========================================================================*/
static void PWM_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    /* ---------- ① 开时钟 ----------
       GPIOA 在 APB2，TIM2 在 APB1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* ---------- ② PA1 配成复用推挽 ----------
       波形由定时器产生，引脚只是"出口"，必须 AF_PP */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ③ 时基：1MHz 计数，1000 个一周期 ---------- */
    tb.TIM_Prescaler         = 72 - 1;          /* 1MHz，1 格 1µs */
    tb.TIM_Period            = PWM_PERIOD - 1;  /* 1000 格 = 1ms = 1kHz */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    /* ---------- ④ 配置通道 2 为 PWM 模式 ----------
       ★ 关键是 OC2：对应 CH2 → PA1。
         如果写成 OC1，改的就是 PA0 的通道，PA1 不会有反应。 */
    TIM_OCStructInit(&oc);                       /* 先给所有成员默认值 */
    oc.TIM_OCMode      = TIM_OCMode_PWM1;        /* CNT < CCR 输出有效电平 */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;    /* 有效电平 = 高 */
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;                      /* 初始 CCR = 0 */
    TIM_OC2Init(TIM2, &oc);

    /* ---------- ⑤ 预装载：改 CCR 下个周期生效，避免毛刺 ---------- */
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* ---------- ⑥ 启动 ---------- */
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  设置占空比
  * @param  duty 0 ~ 1000，对应 0% ~ 100%
  */
static void PWM_SetDuty(uint16_t duty)
{
    if (duty > PWM_PERIOD)
    {
        duty = PWM_PERIOD;          /* 防止越界 */
    }
    TIM_SetCompare2(TIM2, duty);    /* ★ 通道 2 对应 PA1 */
}

/* ============================================================================
 * main：呼吸灯
 * ==========================================================================*/
int main(void)
{
    uint16_t duty;

    PWM_Init();

    while (1)
    {
        /* 渐亮：CCR 从 0 加到 1000（低电平变少 → 灯变暗？见下方说明） */
        for (duty = 0; duty <= PWM_PERIOD; duty += 10)
        {
            PWM_SetDuty(duty);
            Delay_ms(10);
        }

        /* 渐暗：CCR 从 1000 减到 0 */
        for (duty = PWM_PERIOD; duty > 0; duty -= 10)
        {
            PWM_SetDuty(duty);
            Delay_ms(10);
        }
    }
}
