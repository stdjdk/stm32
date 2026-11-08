/**
  ******************************************************************************
  * @file    main.c
  * @brief   09-uart：USART1 收发（printf 重定向 + 回显）
  *
  * 硬件：
  *     PA9  (TX) → CH340 的 RXD
  *     PA10 (RX) ← CH340 的 TXD
  *     GND       — CH340 的 GND   （不要接 VCC）
  *     PA1  → LED（低电平点亮）
  *
  * 现象：
  *     ① 每 500ms 发一行 "Hello from STM32! tick=N"
  *     ② 电脑发什么字符，板子原样发回（回显）
  *     ③ LED 每 500ms 翻转
  *
  * 必须：
  *     Keil → Options for Target → Target → ☑ Use MicroLIB
  *     （否则 printf 走半主机，脱离调试器就卡死）
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "Delay.h"
#include <stdio.h>

/* ============================================================================
 * 一、USART1 初始化
 * ==========================================================================*/
static void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    /* ---------- ① 开时钟：USART1 和 GPIOA 都在 APB2 ---------- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* ---------- ② PA9 = TX，复用推挽 ----------
       波形由 USART 外设产生，所以必须 AF_PP */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ③ PA10 = RX，上拉输入 ----------
       上拉：没接信号时保持高电平（空闲态） */
    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    /* ---------- ④ USART 参数：8N1，收发都开 ---------- */
    usart.USART_BaudRate            = baud;                      /* 115200 */
    usart.USART_WordLength          = USART_WordLength_8b;       /* 8 数据位 */
    usart.USART_StopBits            = USART_StopBits_1;          /* 1 停止位 */
    usart.USART_Parity              = USART_Parity_No;           /* 无校验 */
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

    /* ---------- ⑤ 使能 ---------- */
    USART_Cmd(USART1, ENABLE);
}

/* ============================================================================
 * 二、printf 重定向
 * ==========================================================================*/

/**
  * @brief  把 printf 的字符输出重定向到 USART1
  * @note   printf 内部逐个字符调用 fputc；
  *         我们重写它，让字符走串口。
  *         需要 Keil 勾选 Use MicroLIB。
  */
int fputc(int ch, FILE *f)
{
    /* 等发送寄存器空，再写数据（否则覆盖未发完的字节） */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/* ============================================================================
 * 三、收发小工具
 * ==========================================================================*/

/** @brief 发一个字节 */
static void USART_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}

/**
  * @brief  轮询接收：有数据就原样发回（回显）
  * @note   读 DR 会自动清 RXNE
  */
static void USART_Poll_Echo(void)
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);
        USART_SendByte(ch);          /* 原样发回 */
    }
}

/* ============================================================================
 * 四、LED
 * ==========================================================================*/
static void LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_1);     /* 默认灭 */
}

/* ============================================================================
 * 五、main
 * ==========================================================================*/
int main(void)
{
    uint32_t tick = 0;
    uint8_t  led  = 0;
    uint8_t  i;

    LED_Init();
    USART1_Init(115200);                 /* ★ 和电脑端波特率一致 */

    printf("\r\nSTM32 UART ready, baud=115200\r\n");
    printf("Type something, it will echo back.\r\n");
    printf("--------------------------------\r\n");

    while (1)
    {
        /* ---- 发一行数据 ---- */
        printf("Hello from STM32! tick=%lu\r\n", tick);
        tick++;

        /* ---- LED 翻转 ---- */
        if (led) { GPIO_SetBits(GPIOA, GPIO_Pin_1);   led = 0; }
        else     { GPIO_ResetBits(GPIOA, GPIO_Pin_1); led = 1; }

        /* ---- 等 500ms，同时每 10ms 检查一次接收 ----
           这样电脑发来的字符能较快回显，不用中断 */
        for (i = 0; i < 50; i++)
        {
            USART_Poll_Echo();
            Delay_ms(10);
        }
    }
}
