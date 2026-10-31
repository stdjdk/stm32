/**
  ******************************************************************************
  * @file    main.c
  * @brief   06 前置（下）：TB6612 开环电机测试
  *
  * 硬件：
  *     TB6612 VM   → 5V（手机充电器/充电宝/电池盒，★ 不能用 ST-Link 的 5V）
  *     TB6612 VCC  → 3.3V
  *     TB6612 GND  → 公共地（必须和 STM32 GND 连一起）
  *     TB6612 STBY → PB14（高电平才工作）
  *     TB6612 PWMA → PA1（TIM2_CH2，20kHz）
  *     TB6612 AIN1 → PB12
  *     TB6612 AIN2 → PB13
  *     TB6612 A01  → 电机红线
  *     TB6612 A02  → 电机黑线
  *     OLED        → PB8/PB9
  *
  * 现象：
  *     正转：0% → 100% → 保持 1 秒 → 100% → 0%
  *     停 0.8 秒
  *     反转：同样一遍
  *
  * 注意：
  *     本 demo 是"开环"——只给 PWM，不知道实际转速
  *     等带编码器的电机到货，再加上"反馈"，才是闭环
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================================
 * 一、常量定义
 * ==========================================================================*/

/** @brief PWM 周期：72MHz/36 = 2MHz，2MHz/100 = 20kHz（避开人耳） */
#define MOTOR_PERIOD   100

/* 方向 */
#define MOTOR_STOP      0
#define MOTOR_FORWARD   1
#define MOTOR_BACKWARD  2

/* ============================================================================
 * 二、电机驱动初始化
 * ==========================================================================*/
static void Motor_Init(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* ---------- ① PA1 = PWM 输出（复用推挽） ----------
       波形由 TIM2_CH2 产生，所以必须 AF_PP */
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ② PB12/PB13 = 方向，PB14 = STBY ----------
       这三个是普通推挽输出，由 CPU 控制高低 */
    gpio.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &gpio);

    /* STBY 拉高：使能芯片（悬空 = 待机，电机不动） */
    GPIO_SetBits(GPIOB, GPIO_Pin_14);

    /* 初始：方向脚清零（滑行停止） */
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);

    /* ---------- ③ TIM2：20kHz PWM ---------- */
    tb.TIM_Prescaler         = 36 - 1;              /* 72MHz → 2MHz */
    tb.TIM_Period            = MOTOR_PERIOD - 1;    /* 2MHz / 100 = 20kHz */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;                         /* 初始不转 */
    TIM_OC2Init(TIM2, &oc);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

/* ============================================================================
 * 三、电机控制函数
 * ==========================================================================*/

/**
  * @brief  设置速度
  * @param  duty 0~100（100 = 全速）
  */
static void Motor_SetSpeed(uint16_t duty)
{
    if (duty > MOTOR_PERIOD)
    {
        duty = MOTOR_PERIOD;
    }
    TIM_SetCompare2(TIM2, duty);      /* 通道 2 对应 PA1 */
}

/**
  * @brief  设置方向
  * @param  dir MOTOR_FORWARD / MOTOR_BACKWARD / MOTOR_STOP
  */
static void Motor_SetDir(uint8_t dir)
{
    if (dir == MOTOR_FORWARD)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);      /* AIN1 = 1 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);    /* AIN2 = 0 */
    }
    else if (dir == MOTOR_BACKWARD)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);    /* AIN1 = 0 */
        GPIO_SetBits(GPIOB, GPIO_Pin_13);      /* AIN2 = 1 */
    }
    else
    {
        /* 两个都为 0：滑行停止 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
    }
}

/** @brief 停止 */
static void Motor_Stop(void)
{
    Motor_SetSpeed(0);
    Motor_SetDir(MOTOR_STOP);
}

/* ============================================================================
 * 四、main：正转升速 → 保持 → 降速 → 停 → 反转
 * ==========================================================================*/
int main(void)
{
    uint16_t s;

    OLED_Init();
    OLED_ShowString(1, 1, "Spd:");
    OLED_ShowString(2, 1, "Dir:");
    OLED_ShowString(3, 1, "TB6612");
    OLED_ShowString(4, 1, "open-loop");

    Motor_Init();
    Motor_Stop();

    while (1)
    {
        /* ========== 正转：0% → 100% ========== */
        Motor_SetDir(MOTOR_FORWARD);
        OLED_ShowString(2, 5, "FWD");

        for (s = 0; s <= MOTOR_PERIOD; s += 5)
        {
            Motor_SetSpeed(s);
            OLED_ShowNum(1, 5, s, 3);
            Delay_ms(150);
        }

        Delay_ms(1000);                         /* 全速保持 1 秒 */

        /* ========== 正转：100% → 0% ========== */
        for (s = MOTOR_PERIOD; s > 0; s -= 5)
        {
            Motor_SetSpeed(s);
            OLED_ShowNum(1, 5, s, 3);
            Delay_ms(150);
        }

        Motor_Stop();
        Delay_ms(800);                          /* 停 0.8 秒 */

        /* ========== 反转：0% → 100% ========== */
        Motor_SetDir(MOTOR_BACKWARD);
        OLED_ShowString(2, 5, "REV");

        for (s = 0; s <= MOTOR_PERIOD; s += 5)
        {
            Motor_SetSpeed(s);
            OLED_ShowNum(1, 5, s, 3);
            Delay_ms(150);
        }

        Delay_ms(1000);

        /* ========== 反转：100% → 0% ========== */
        for (s = MOTOR_PERIOD; s > 0; s -= 5)
        {
            Motor_SetSpeed(s);
            OLED_ShowNum(1, 5, s, 3);
            Delay_ms(150);
        }

        Motor_Stop();
        Delay_ms(800);
    }
}
