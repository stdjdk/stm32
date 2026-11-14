/**
  ******************************************************************************
  * @file    main.c
  * @brief   13-wdg：IWDG 独立看门狗演示（逐行注释）
  *
  * 硬件：
  *     LED   → PA1（低电平点亮）
  *     按键  → PB1（按下接地，上拉输入）
  *     串口  → PA9(TX) / PA10(RX)，115200
  *
  * 现象：
  *     ① 正常：每秒打印一行动作 + LED 翻转
  *     ② 按下 PB1：程序进入死循环（不再喂狗）
  *     ③ 约 4 秒后：芯片自动复位，串口打印"看门狗复位"
  *
  * 注意：
  *     Keil 勾选 Use MicroLIB（printf 重定向需要）
  ******************************************************************************
  */

#include "stm32f10x.h"   /* 标准外设库：RCC/IWDG/GPIO/USART 都在这里 */
#include "OLED.h"        /* OLED（本 demo 可不用，保留方便扩展） */
#include "Delay.h"       /* Delay_ms */
#include <stdio.h>       /* printf */

/* ============================================================================
 * 一、串口初始化（用来观察复位现象）
 * ==========================================================================*/
static void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;   /* GPIO 参数结构体 */
    USART_InitTypeDef usart;  /* USART 参数结构体 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);  /* USART1 和 GPIOA 都在 APB2 */

    gpio.GPIO_Pin   = GPIO_Pin_9;               /* PA9 = TX */
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;          /* 复用推挽：波形由 USART 产生 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_10;               /* PA10 = RX */
    gpio.GPIO_Mode = GPIO_Mode_IPU;             /* 上拉输入 */
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;                        /* 115200 */
    usart.USART_WordLength          = USART_WordLength_8b;         /* 8 位 */
    usart.USART_StopBits            = USART_StopBits_1;            /* 1 停止位 */
    usart.USART_Parity              = USART_Parity_No;             /* 无校验 */
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);
}

/** @brief printf 重定向到串口（需要 Use MicroLIB） */
int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  /* 等发送区空 */
    USART_SendData(USART1, (uint8_t)ch);                           /* 写 DR 发出去 */
    return ch;
}

/* ============================================================================
 * 二、LED 与按键
 * ==========================================================================*/
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};                 /* 全部清零，避免未初始化字段 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_1;                /* PA1 */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;          /* 推挽输出 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_1);             /* 默认灭（低电平亮） */
}

static void KEY_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin  = GPIO_Pin_1;                 /* PB1 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;              /* 上拉输入：没按时为高 */
    GPIO_Init(GPIOB, &gpio);
}

/* ============================================================================
 * 三、IWDG 初始化（4 秒超时）
 * ==========================================================================*/
static void IWDG_Init_4s(void)
{
    /* ① 打开 LSI（约 40kHz）：IWDG 用它当独立时钟，主时钟挂了它还在跑 */
    RCC_LSICmd(ENABLE);

    /* ② 等 LSI 稳定：不稳定就配置，分频会错 */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    /* ③ 解锁 IWDG 的写保护：不解锁，后面三句全部无效 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* ④ 预分频 64：LSI 40kHz / 64 = 625Hz，即每 1.6ms 减 1 */
    IWDG_SetPrescaler(IWDG_Prescaler_64);

    /* ⑤ 重装值 625：625 × 1.6ms ≈ 1 秒？不——见下行注释
          实际公式 T = (4 × 2^PR) × RLR / 40000
          PR=64 → 4×64 = 256 → 256 × 625 / 40000 = 4.0 秒 */
    IWDG_SetReload(625);

    /* ⑥ 先喂一次，把计数器装满 */
    IWDG_ReloadCounter();

    /* ⑦ 启动：此后无法关闭，只能靠复位 */
    IWDG_Enable();
}

/* ============================================================================
 * 四、main
 * ==========================================================================*/
int main(void)
{
    uint32_t tick = 0;   /* 心跳计数，也用来控制 LED */

    LED_Init();                       /* 初始化 LED */
    KEY_Init();                       /* 初始化按键 */
    USART1_Init(115200);              /* 初始化串口，方便观察 */

    /* ---------- 第 1 步：判断这次复位是什么原因 ----------
       看门狗复位会置位 IWDG 标志；必须清掉，否则以后每次读都为真 */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        printf("\r\n[RESET] IWDG reset\r\n");     /* 说明是程序卡死被救回来了 */
        RCC_ClearFlag();                           /* 清所有复位标志 */
    }
    else
    {
        printf("\r\n[RESET] power-on / NRST\r\n");   /* 正常上电 / 按了复位键 */
    }

    printf("IWDG demo start (timeout 4s)\r\n");
    printf("Press PB1 to simulate hang\r\n");

    /* ---------- 第 2 步：启动看门狗 ---------- */
    IWDG_Init_4s();

    /* ---------- 第 3 步：主循环 ---------- */
    while (1)
    {
        IWDG_ReloadCounter();          /* 喂狗：必须在 4 秒内至少喂一次 */

        printf("heartbeat %lu\r\n", tick);   /* 打印心跳，证明程序活着 */
        tick++;

        /* LED 翻转：亮灭交替，肉眼看程序是否在跑 */
        if (tick & 1) GPIO_ResetBits(GPIOA, GPIO_Pin_1);   /* 奇数拍：亮 */
        else          GPIO_SetBits(GPIOA, GPIO_Pin_1);     /* 偶数拍：灭 */

        /* 按下 PB1 → 故意进入死循环，模拟"程序跑飞" */
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == RESET)
        {
            printf("HANG! stop feeding dog...\r\n");
            while (1);                 /* 死循环里不再喂狗 → 4 秒后 IWDG 复位 */
        }

        Delay_ms(500);                 /* 每 500ms 一次，喂狗间隔远小于 4s */
    }
}
