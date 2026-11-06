/**
  ******************************************************************************
  * @file    main.c
  * @brief   06-encoder（完整版）：TB6612 驱动 + 编码器测速
  *
  * 硬件：
  *     电机红/黑线   → TB6612 A01 / A02
  *     编码器蓝线    → 3.3V（编码器电源）
  *     编码器白线    → GND
  *     编码器黄线    → PA6（TIM3_CH1，A 相）
  *     编码器绿线    → PA7（TIM3_CH2，B 相）
  *     TB6612 VM     → 12V（12V 电机）或 5V（小电机）
  *     TB6612 VCC    → 3.3V
  *     TB6612 GND    → 公共地
  *     TB6612 STBY   → PB14
  *     TB6612 PWMA   → PA1（TIM2_CH2，20kHz）
  *     TB6612 AIN1   → PB12
  *     TB6612 AIN2   → PB13
  *
  * 两种模式（改 CALIB_MODE 切换）：
  *     CALIB_MODE = 1：标定模式——不驱动电机，用手转输出轴 10 圈，
  *                    记下 OLED 上的计数，除以 10 就是"每圈计数"
  *     CALIB_MODE = 0：测速模式——驱动电机，显示 RPM
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================================
 * 一、参数（★ 标定后要改这里）
 * ==========================================================================*/

/** @brief 模式：0=测速  1=手转标定  2=全速反推标定  3=调试（看 LOOP/CNT） */
#define CALIB_MODE        0

/**
  * @brief 输出轴转一圈的编码器计数
  * @note  本电机参数：11 线、4 倍频、减速比 1:30
  *        11 × 4 × 30 = 1320
  *        其他校验方法：
  *          手转法：CALIB_MODE=1，手转 10 圈，读数 ÷ 10
  *          全速反推：CALIB_MODE=2，12V 满 PWM 跑稳后读 Delta，
  *                    COUNTS_PER_REV = Delta × 600 / 333
  */
#define COUNTS_PER_REV    1320

/** @brief PWM 周期：72MHz/36 = 2MHz，2MHz/100 = 20kHz */
#define MOTOR_PERIOD      100

/* 采样周期：100ms → RPM = Δ × 600 / 每圈计数 */
#define SAMPLE_MS         100

/* 方向 */
#define MOTOR_STOP        0
#define MOTOR_FORWARD     1
#define MOTOR_BACKWARD    2

/* ============================================================================
 * 二、电机驱动
 * ==========================================================================*/
static void Motor_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* PA1 = PWM */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PB12/PB13 = 方向，PB14 = STBY */
    gpio.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOB, GPIO_Pin_14);                    /* STBY 拉高 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);    /* 初始停止 */

    /* TIM2：20kHz PWM */
    tb.TIM_Prescaler         = 36 - 1;
    tb.TIM_Period            = MOTOR_PERIOD - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;
    TIM_OC2Init(TIM2, &oc);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

static void Motor_SetSpeed(uint16_t duty)
{
    if (duty > MOTOR_PERIOD) duty = MOTOR_PERIOD;
    TIM_SetCompare2(TIM2, duty);
}

static void Motor_SetDir(uint8_t dir)
{
    if (dir == MOTOR_FORWARD)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    }
    else if (dir == MOTOR_BACKWARD)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
    }
}

/* ============================================================================
 * 三、编码器（TIM3，PA6/PA7，4 倍频）
 * ==========================================================================*/
static void Encoder_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef       ic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* PA6/PA7 上拉输入 */
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 编码器模式不需要分频，ARR 满量程 */
    tb.TIM_Prescaler         = 0;
    tb.TIM_Period            = 65535;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &tb);

    /* 编码器模式 3：4 倍频 */
    TIM_EncoderInterfaceConfig(TIM3,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    /* 输入滤波：电机编码器信号比较快，用中等滤波 */
    TIM_ICStructInit(&ic);
    ic.TIM_Channel     = TIM_Channel_1;
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x06;
    TIM_ICInit(TIM3, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &ic);

    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/** @brief 读取增量（带方向），并清零基准 */
static uint16_t s_last_cnt = 0;

static int16_t Encoder_GetDelta(void)
{
    uint16_t now = TIM_GetCounter(TIM3);
    int16_t  d   = (int16_t)(now - s_last_cnt);

    s_last_cnt = now;
    return d;
}

/* ============================================================================
 * 四、main
 * ==========================================================================*/
int main(void)
{
    int16_t  delta;

#if CALIB_MODE != 3
    int32_t  total = 0;
#endif

#if !CALIB_MODE
    int32_t  rpm;
    uint16_t pwm   = 100;     /* 测速模式下的占空比 0~100（3.7V 先用 100） */
#endif

    OLED_Init();

#if CALIB_MODE == 1
    OLED_ShowString(1, 1, "CALIB1");
    OLED_ShowString(2, 1, "Cnt:");
    OLED_ShowString(3, 1, "Rev:");
    OLED_ShowString(4, 1, "turn 10x");
#elif CALIB_MODE == 2
    OLED_ShowString(1, 1, "CALIB2");
    OLED_ShowString(2, 1, "D:");
    OLED_ShowString(3, 1, "T:");
    OLED_ShowString(4, 1, "fullspeed");
#elif CALIB_MODE == 3
    OLED_ShowString(1, 1, "LOOP:");
    OLED_ShowString(2, 1, "CNT:");
    OLED_ShowString(3, 1, "D:");
    OLED_ShowString(4, 1, "DBG");
#else
    OLED_ShowString(1, 1, "RPM:");
    OLED_ShowString(2, 1, "Dir:");
    OLED_ShowString(3, 1, "Cnt:");
    OLED_ShowString(4, 1, "PWM:");
#endif

    Motor_Init();
    Encoder_Init();
    Motor_SetSpeed(0);
    Motor_SetDir(MOTOR_STOP);

#if CALIB_MODE == 1
    /* ---------- 手转标定：手转 10 圈，读累计计数 ---------- */
    while (1)
    {
        Delay_ms(SAMPLE_MS);
        delta = Encoder_GetDelta();
        total += delta;

        OLED_ShowNum(2, 5, (uint32_t)(total < 0 ? -total : total), 6);
        OLED_ShowNum(3, 5, (uint32_t)((total < 0 ? -total : total) / 4), 5);
    }
#elif CALIB_MODE == 2
    /* ---------- 全速反推标定 ----------
       12V 满 PWM 跑稳后，读 100ms 的 Delta：
           COUNTS_PER_REV = Delta × 600 / 333
       注意：必须 12V 供电，5V 时转速不是 333，不能这样算 */
    Motor_SetDir(MOTOR_FORWARD);
    Motor_SetSpeed(100);

    while (1)
    {
        Delay_ms(SAMPLE_MS);
        delta = Encoder_GetDelta();
        total += delta;

        OLED_ShowNum(2, 5, (uint32_t)(delta < 0 ? -delta : delta), 4);
        OLED_ShowNum(3, 5, (uint32_t)(total < 0 ? -total : total), 6);
    }
#elif CALIB_MODE == 3
    /* ---------- 调试模式 ----------
       看三件事：
         LOOP：循环次数，涨 = 程序在跑，不涨 = 复位/卡死
         CNT ：TIM3 原始计数，电机转时应该变化
         D   ：100ms 增量 */
    {
        uint32_t loop_cnt = 0;

        Motor_SetDir(MOTOR_FORWARD);
        Motor_SetSpeed(100);

        while (1)
        {
            Delay_ms(SAMPLE_MS);
            delta = Encoder_GetDelta();
            loop_cnt++;

            OLED_ShowNum(1, 6, loop_cnt, 5);
            OLED_ShowNum(2, 5, TIM_GetCounter(TIM3), 5);
            OLED_ShowNum(3, 3, (uint32_t)(delta < 0 ? -delta : delta), 4);
        }
    }
#else
    Motor_SetDir(MOTOR_FORWARD);
    Motor_SetSpeed(pwm);

    while (1)
    {
        Delay_ms(SAMPLE_MS);

        delta = Encoder_GetDelta();
        total += delta;

        /* RPM = Δ计数 / 每圈计数 × (60000 / 采样毫秒) */
        rpm = (int32_t)delta * (60000 / SAMPLE_MS) / COUNTS_PER_REV;

        OLED_ShowNum(1, 5, (uint32_t)(rpm < 0 ? -rpm : rpm), 5);
        OLED_ShowString(2, 5, rpm > 0 ? "FWD" : (rpm < 0 ? "REV" : "---"));
        OLED_ShowNum(3, 5, (uint32_t)(delta < 0 ? -delta : delta), 3);
        OLED_ShowNum(4, 5, pwm, 3);
    }
#endif
}
