/**
  ******************************************************************************
  * @file    main.c
  * @brief   03-timer：定时器中断（LED 1Hz + 秒计数）
  *
  * 硬件：
  *     PA1 —— LED（低电平点亮）
  *     OLED —— PB8/PB9
  *
  * 时间计算：
  *     72MHz ÷ (PSC+1) = 72MHz ÷ 7200 = 10kHz  → 每格 100µs
  *     (ARR+1) × 100µs = 5000 × 100µs = 500ms  → 每 500ms 中断一次
  *
  * 现象：
  *     每 500ms 翻转一次 LED  → 亮 0.5s、灭 0.5s，1 秒一个周期（1Hz）
  *     每 2 次中断（1 秒）→ n +1，OLED 显示秒数
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"

/* ============================================================================
 * 全局变量
 *
 * ★ 全部加 volatile：
 *   它们会被中断修改、主循环读取。不加的话编译器可能把值缓存在寄存器里，
 *   主循环永远看到 0（即使中断里已经改了）。
 * ==========================================================================*/
volatile unsigned int n = 0;   /* 秒计数（每 2 次中断 +1） */
volatile unsigned int a = 0;   /* 500ms 计数，满 2 次算 1 秒 */
volatile unsigned int b = 1;   /* LED 当前状态：1=亮，0=灭 */

/* ============================================================================
 * LED 初始化：PA1 推挽输出
 * ==========================================================================*/
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_1);      /* 默认高电平 → LED 灭 */
}

/* ============================================================================
 * TIM4 初始化：每 500ms 产生一次更新中断
 * ==========================================================================*/
static void TIM4_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef        nvic;

    /* ---------- ① 开时钟 ----------
       ★ 必须和下面配置的定时器一致：配 TIM4 就开 TIM4。
       原来写成 TIM3 是致命错误：TIM4 没电，永远不计数。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* ---------- ② 配时基 ----------
       72MHz ÷ 7200 = 10kHz → 每格 100µs
       5000 格 × 100µs = 500ms
       注意 PSC / ARR 都要写"目标值 - 1"（硬件自动加 1） */
    tb.TIM_Prescaler         = 7200 - 1;   /* 10kHz */
    tb.TIM_Period            = 5000 - 1;   /* 500ms */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    /* ---------- ③ 清掉初始化产生的更新事件 ----------
       TIM_TimeBaseInit 内部会置一次 UG，不清的话一开中断就多进一次。 */
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);

    /* ---------- ④ 允许"更新中断" ----------
       对应 DIER 的 UIE 位："数满一圈时允许发中断请求"。 */
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    /* ---------- ⑤ 配置 NVIC ---------- */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic.NVIC_IRQChannel                   = TIM4_IRQn;   /* 中断号 30 */
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);

    /* ---------- ⑥ 最后启动定时器 ----------
       放在最后，保证前面的配置都已生效。 */
    TIM_Cmd(TIM4, ENABLE);
}

/* ============================================================================
 * main
 * ==========================================================================*/
int main(void)
{
    OLED_Init();

    LED_Init();
    TIM4_Init();          /* 启动 500ms 心跳 */

    OLED_ShowString(1, 1, "N:");

    while (1)
    {
        /* 主循环只负责显示，不做计时 */
        OLED_ShowNum(1, 3, n, 5);
    }
}

/* ============================================================================
 * TIM4 中断服务函数：每 500ms 进一次
 * ==========================================================================*/
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        /* ---------- 每 500ms：翻转 LED ---------- */
        if (b)
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);   /* 亮（低电平） */
            b = 0;
        }
        else
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_1);     /* 灭（高电平） */
            b = 1;
        }

        /* ---------- 数 2 次 500ms = 1 秒，n +1 ----------
           ★ 原来的 a += a 是错的：a 初始为 0，0+0 永远还是 0，
             所以永远进不了"n++"那个分支。 */
        a++;
        if (a >= 2)
        {
            a = 0;
            n++;
        }

        /* ★ 必须清标志，否则退出后立刻又进来，主程序卡死 */
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
