/**
  ******************************************************************************
  * @file    main.c
  * @brief   02-exti：外部中断按键 + 定时器测量按下时长
  *
  * 硬件连接：
  *     PA1 —— LED（低电平点亮）
  *     PB1 —— 按键（一端接 PB1，另一端接 GND）   ★ 注意是 PB1，不是 PB0
  *
  * 为什么用 PB1 / EXTI1，而不是 PB0 / EXTI0：
  *     本工程 Hardware\IR.c 里已经定义了 EXTI0_IRQHandler（红外解码），
  *     同一个中断向量只能有一个服务函数，否则链接报
  *     "EXTI0_IRQHandler multiply defined"。
  *     所以按键改用 EXTII1（PB1），两者互不干扰。
  *
  * 实现的功能：
  *     1) 按下瞬间：LED 点亮
  *     2) 松开瞬间：LED 熄灭，并计算"按下持续了多久"
  *     3) 时长显示在 OLED 上
  *     4) 区分短按（<300ms）与长按（>=300ms）
  *
  * 第四个要求的实现思路：
  *     · TIM3 配成 10kHz 自由计数（每格 100µs），永不清零
  *     · EXTI1 配成【双边沿】触发：下降沿=按下，上升沿=松开
  *     · 中断里读 TIM3：按下记起点，松开时相减得到时长
  *     · 16 位减法天然处理计数器回绕
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"

/* ============================================================================
 * 一、全局变量
 * ==========================================================================*/

/** @brief 最近一次"按下→松开"的时长，单位毫秒（中断写、主循环读） */
volatile uint16_t PressMs    = 0;

/** @brief 累计完成的按键次数 */
volatile uint16_t PressCount = 0;

/** @brief 有新的一次按键完成，通知主循环刷新显示 */
volatile uint8_t  NewPress   = 0;

/* ---- 内部状态（只在中断里使用） ---- */

/** @brief 按下瞬间的定时器计数值 */
static volatile uint16_t pressTick = 0;

/** @brief 上一次"有效边沿"的计数值，用于消抖 */
static volatile uint16_t lastEdge  = 0;

/** @brief 是否已经记录过有效边沿 */
static volatile uint8_t  haveEdge  = 0;

/* ============================================================================
 * 二、参数
 * ==========================================================================*/

/** @brief 消抖门限：两次边沿间隔 < 20ms 视为抖动（10kHz 下 20ms = 200 tick） */
#define DEBOUNCE_TICKS   200

/* ============================================================================
 * 三、LED：PA1 推挽输出
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
 * 四、按键：PB1 上拉输入 + EXTI1 双边沿中断
 * ==========================================================================*/
static void Key_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    /* ① 开时钟：GPIOB（引脚）+ AFIO（EXTI 选路） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* ② PB1 上拉输入：松手=高，按下=低（按键另一端接 GND） */
    gpio.GPIO_Pin  = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    /* ③ AFIO 选路：EXTI1 ← PB1 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    /* ④ EXTI1：双边沿触发（按下和松开都要中断） */
    exti.EXTI_Line    = EXTI_Line1;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    /* ⑤ NVIC：填 EXTI1_IRQn（中断号 7），不是 EXTI_Line1 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic.NVIC_IRQChannel                   = EXTI1_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);
}

/* ============================================================================
 * 五、TIM3：10kHz 自由计数
 * ==========================================================================*/
static void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 72MHz / 7200 = 10kHz → 每格 100µs，16 位满量程约 6.55 秒 */
    tb.TIM_Prescaler         = 7200 - 1;
    tb.TIM_Period            = 0xFFFF;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);                /* 一直数，不清零 */
}

/* ============================================================================
 * 六、EXTI1 中断
 * ==========================================================================*/
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        uint16_t now = TIM_GetCounter(TIM3);
        uint8_t  lvl = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);

        /* 消抖：距上次有效边沿不足 20ms 的，忽略 */
        if (!haveEdge || (uint16_t)(now - lastEdge) >= DEBOUNCE_TICKS)
        {
            haveEdge = 1;
            lastEdge = now;

            if (lvl == 0)
            {
                /* 下降沿：按下 → 记起点，点灯 */
                pressTick = now;
                GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            }
            else
            {
                /* 上升沿：松开 → 算时长，灭灯 */
                uint16_t ticks = (uint16_t)(now - pressTick);

                PressMs = ticks / 10;         /* 100µs/格 → /10 = ms */
                PressCount++;
                NewPress = 1;

                GPIO_SetBits(GPIOA, GPIO_Pin_1);
            }
        }

        EXTI_ClearITPendingBit(EXTI_Line1);   /* 必须清 */
    }
}

/* ============================================================================
 * 七、主函数
 * ==========================================================================*/
int main(void)
{
    OLED_Init();
    OLED_ShowString(1, 1, "Time:");
    OLED_ShowString(2, 1, "Cnt:");

    LED_Init();
    TIM3_Init();
    Key_Init();

    while (1)
    {
        if (NewPress)
        {
            NewPress = 0;

            OLED_ShowNum(1, 7, PressMs, 5);
            OLED_ShowNum(2, 6, PressCount, 4);

            if (PressMs < 300)
            {
                /* 短按逻辑 */
            }
            else
            {
                /* 长按逻辑 */
            }
        }
    }
}
