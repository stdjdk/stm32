/**
  ******************************************************************************
  * @file    main.c
  * @brief   12-rtc：RTC 走时 + OLED 显示（逐行注释）
  *
  * 硬件：不需要外接模块
  *     · 板子有 32.768kHz 晶振 → 用 LSE，走时准
  *     · 没有 → 自动退到 LSI（内部 RC，会漂）
  *     · 想要掉电保持 → VBAT 接纽扣电池
  *
  * 现象：
  *     RTC:
  *     D: 0            ← 从起点过了多少天
  *     T: 00:00:05     ← 时:分:秒，每秒 +1
  ******************************************************************************
  */

#include "stm32f10x.h"   /* 标准外设库：RCC/RTC/BKP/PWR 的函数和寄存器定义都在里面 */
#include "OLED.h"        /* OLED 显示 */
#include "Delay.h"       /* Delay_ms：控制刷新节奏 */

/* ============================================================================
 * 一、宏定义
 * ==========================================================================*/

/* 备份寄存器里的"魔法值"：用来判断这次上电前，RTC 是否已经初始化过。
   任意一个约定好的数都行，只要和"未初始化"时的值（通常 0x0000）不同即可 */
#define RTC_MAGIC   0xA5A5

/* ============================================================================
 * 二、RTC 初始化
 * ==========================================================================*/
static void RTC_Init_Config(void)
{
    /* ---------- ① 开 PWR 和 BKP 的时钟 ----------
       PWR：电源控制，用来解锁备份域
       BKP：备份寄存器，用来存"已初始化"标记
       两个都挂在 APB1 上 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* ---------- ② 解锁备份域 ----------
       备份域默认"写保护"，不解锁的话写 RTC/BKP 全部无效（而且不报错） */
    PWR_BackupAccessCmd(ENABLE);

    /* ---------- ③ 判断是不是第一次上电 ----------
       读 BKP_DR1：等于魔法值 = 以前初始化过，时间还在走，不能重置
       不等于 = 第一次（或电池没电丢了），需要完整初始化 */
    if (BKP_ReadBackupRegister(BKP_DR1) != RTC_MAGIC)
    {
        uint32_t timeout;   /* 等 LSE 起振的超时计数，防止没晶振时死等 */

        /* ---------- ④ 先尝试 LSE（外部 32.768kHz 晶振，精度最高） ---------- */
        RCC_LSEConfig(RCC_LSE_ON);      /* 让 LSE 振荡器开始工作 */

        timeout = 0xFFFFF;              /* 给一个足够大的超时值 */
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
        {
            /* LSERDY=0 表示还没稳定；一直等，直到超时 */
            if (--timeout == 0) break;  /* 减到 0 说明等了太久 → 大概率没焊晶振 */
        }

        if (timeout == 0)
        {
            /* ---------- ⑤ LSE 起不来 → 退到 LSI（内部 40kHz RC） ---------- */
            RCC_LSICmd(ENABLE);                                 /* 启动内部 LSI */
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); /* 等 LSI 稳定（内部振荡器很快） */

            RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);             /* 选 LSI 当 RTC 时钟 */
            RCC_RTCCLKCmd(ENABLE);                              /* 把 RTC 时钟送到 RTC 模块 */
            RTC_WaitForSynchro();                               /* 等 RTC 寄存器跨时钟域同步 */
            RTC_WaitForLastTask();                              /* 等上一次写操作落盘 */

            RTC_SetPrescaler(40000 - 1);                        /* LSI≈40kHz，40000 分频 → 1Hz，即每秒 +1 */
            RTC_WaitForLastTask();                              /* 等预分频写进去 */
        }
        else
        {
            /* ---------- ⑥ LSE 正常 ---------- */
            RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);             /* 选 LSE 当 RTC 时钟 */
            RCC_RTCCLKCmd(ENABLE);                              /* 使能 RTC 时钟 */
            RTC_WaitForSynchro();                               /* 同步 */
            RTC_WaitForLastTask();                              /* 等上次写完成 */

            RTC_SetPrescaler(32768 - 1);                        /* 32768Hz / 32768 = 1Hz */
            RTC_WaitForLastTask();                              /* 等预分频生效 */
        }

        /* ---------- ⑦ 设初始时间 ----------
           这里从 0 开始；想要真实时间就填 Unix 时间戳（电脑上算好） */
        RTC_SetCounter(0);
        RTC_WaitForLastTask();          /* 等计数器写进去 */

        /* ---------- ⑧ 打标记 ----------
           写进 BKP_DR1，下次上电读到它就跳过初始化，时间继续走 */
        BKP_WriteBackupRegister(BKP_DR1, RTC_MAGIC);
    }
    else
    {
        /* ---------- 已经初始化过：只同步，绝对不能重置计数器 ---------- */
        RTC_WaitForSynchro();           /* 让 CPU 看到 RTC 的最新值 */
        RTC_WaitForLastTask();          /* 等上次操作完成 */
    }
}

/* ============================================================================
 * 三、main
 * ==========================================================================*/
int main(void)
{
    uint32_t total;        /* RTC 总秒数 */
    uint32_t days;         /* 从起点过了多少天 */
    uint32_t sec_of_day;   /* 当天是第几秒（0~86399） */
    uint8_t  hh, mm, ss;   /* 时、分、秒 */

    OLED_Init();                    /* 初始化 OLED（PB8/PB9 软件 I2C） */
    OLED_ShowString(1, 1, "RTC:");  /* 第 1 行标题 */
    OLED_ShowString(2, 1, "D:");    /* 第 2 行：天数 */
    OLED_ShowString(3, 1, "T:");    /* 第 3 行：时:分:秒 */

    RTC_Init_Config();              /* 初始化 RTC（首次才真正配置，之后跳过） */

    while (1)
    {
        total = RTC_GetCounter();          /* 读 32 位秒计数器，这就是"现在几秒" */

        days       = total / 86400;        /* 86400 秒 = 1 天 → 换算成天数 */
        sec_of_day = total % 86400;        /* 余数 = 当天已经过的秒数 */

        hh = sec_of_day / 3600;            /* 3600 秒 = 1 小时 */
        mm = (sec_of_day % 3600) / 60;     /* 剩下的再除以 60 = 分钟 */
        ss = sec_of_day % 60;              /* 余数就是秒 */

        OLED_ShowNum(2, 3, days, 3);       /* 第 2 行第 3 列显示天数（3 位） */
        OLED_ShowNum(3, 3, hh, 2);         /* 显示时（2 位） */
        OLED_ShowString(3, 5, ":");        /* 冒号分隔 */
        OLED_ShowNum(3, 6, mm, 2);         /* 显示分 */
        OLED_ShowString(3, 8, ":");        /* 冒号分隔 */
        OLED_ShowNum(3, 9, ss, 2);         /* 显示秒 */

        Delay_ms(200);                     /* 200ms 刷一次：既看着连续，又不闪 */
    }
}
