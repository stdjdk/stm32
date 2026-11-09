/**
  ******************************************************************************
  * @file    main.c
  * @brief   10-i2c：硬件 I2C1 读 MPU6050（逐行注释版）
  *
  * 硬件接线：
  *     MPU6050 VCC → 3.3V
  *     MPU6050 GND → GND
  *     MPU6050 SCL → PB6   （I2C1_SCL）
  *     MPU6050 SDA → PB7   （I2C1_SDA）
  *     MPU6050 AD0 → GND   （决定地址 0x68）
  *     MPU6050 INT → 不接
  *     OLED        → PB8/PB9（软件 I2C，与 MPU6050 不冲突）
  *
  * 用到的 MPU6050 寄存器：
  *     0x75  WHO_AM_I     读出来应为 0x68，用来验证通信
  *     0x6B  PWR_MGMT_1   写 0x00 解除睡眠
  *     0x1C  ACCEL_CONFIG 写 0x00 → ±2g，16384 LSB/g
  *     0x1B  GYRO_CONFIG  写 0x00 → ±250°/s，131 LSB/(°/s)
  *     0x3B  ACCEL_XOUT_H 从这里开始连续 14 字节：
  *           AX(2) AY(2) AZ(2) TEMP(2) GX(2) GY(2) GZ(2)，高字节在前
  *
  * I2C 帧格式（读一个寄存器）：
  *     START | 0xD0(写) | 寄存器号 | 重复START | 0xD1(读) | 数据 | NACK | STOP
  ******************************************************************************
  */

#include "stm32f10x.h"   /* 标准外设库总头：所有外设寄存器定义和 SPL 函数声明 */
#include "OLED.h"        /* OLED 驱动：OLED_Init / OLED_ShowString / OLED_ShowSignedNum */
#include "Delay.h"       /* 延时：Delay_ms() */

/* ============================ 一、地址定义 ============================ */

#define MPU_ADDR    0x68                    /* MPU6050 的 7 位地址：AD0 接 GND 时是 0x68 */
#define MPU_ADDR_W  (MPU_ADDR << 1)         /* 写方向 8 位地址：0x68<<1 | 0 = 0xD0 */
#define MPU_ADDR_R  ((MPU_ADDR << 1) | 1)   /* 读方向 8 位地址：0x68<<1 | 1 = 0xD1 */

/* ========================== 二、I2C1 初始化 ========================== */
static void I2C1_Init(void)
{
    GPIO_InitTypeDef gpio;   /* 定义 GPIO 初始化结构体：用来填引脚参数 */
    I2C_InitTypeDef  i2c;    /* 定义 I2C 初始化结构体：用来填 I2C 参数 */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  /* 开 GPIOB 时钟（APB2 总线）；不开则 PB6/PB7 配不进去 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);   /* 开 I2C1 时钟（APB1 总线）；不开则 I2C1 完全不工作 */

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;   /* 选中 PB6(SCL) 和 PB7(SDA) 两个引脚 */
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;           /* 复用开漏：引脚交给 I2C1 外设控制，且只能拉低（I2C 线与要求） */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;          /* 引脚翻转速度 50MHz；对 100kHz 的 I2C 绰绰有余 */
    GPIO_Init(GPIOB, &gpio);                     /* 把上面三个参数写进 GPIOB 的 CRL/CRH 寄存器 */

    i2c.I2C_Mode                = I2C_Mode_I2C;                  /* 选择 I2C 模式（另一种是 SMBus） */
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;               /* 快速模式的 SCL 高低比 2:1；100kHz 下该参数不起作用 */
    i2c.I2C_OwnAddress1         = 0x00;                          /* 本机地址：当主机时无意义，填 0 即可 */
    i2c.I2C_Ack                 = I2C_Ack_Enable;                /* 收到一个字节后硬件自动回 ACK（主机读从机时必须回） */
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* 使用 7 位地址格式（绝大多数器件） */
    i2c.I2C_ClockSpeed          = 100000;                        /* SCL 频率 100kHz = 标准模式；也可以试 400000 */
    I2C_Init(I2C1, &i2c);                                        /* 把上面 6 个参数写进 I2C1 的 CR2/CCR/TRISE/OAR 寄存器 */

    I2C_Cmd(I2C1, ENABLE);   /* 使能 I2C1（CR1 的 PE 位置 1）；此后 I2C1 才会驱动 PB6/PB7 */
}

/* ====================== 三、带超时的等待工具 ====================== */

/**
  * @brief  等一个 I2C 事件，最多等 10 万次
  * @param  event：事件宏，如 I2C_EVENT_MASTER_MODE_SELECT
  * @retval 1 = 等到了；0 = 超时（设备没应答 / 总线坏）
  */
static uint8_t I2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = 100000;          /* 超时计数器：防止死循环，值越大等得越久 */

    while (!I2C_CheckEvent(I2C1, event))/* I2C_CheckEvent：查 SR1+SR2 组合是否等于该事件 */
    {
        if (--timeout == 0)             /* 每循环一次减 1，减到 0 说明等太久了 */
        {
            return 0;                   /* 返回失败，调用方会走错误分支 */
        }
    }
    return 1;                           /* 等到事件，返回成功 */
}

/**
  * @brief  等总线空闲（BUSY 位为 0）
  * @retval 1 = 空闲；0 = 超时
  */
static uint8_t I2C_WaitIdle(void)
{
    uint32_t timeout = 100000;                          /* 同样加超时，防止总线被拉死时卡住 */

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))      /* BUSY=1 表示总线上还有事务没结束 */
    {
        if (--timeout == 0)                             /* 减到 0 表示等了太久 */
        {
            return 0;                                   /* 返回失败 */
        }
    }
    return 1;                                           /* 总线空闲，可以发起新事务 */
}

/* ======================= 四、写一个寄存器 ======================= */
static uint8_t MPU_WriteReg(uint8_t reg, uint8_t data)
{
    if (!I2C_WaitIdle()) return 0;      /* 先确认总线空闲；不空闲直接失败返回 */

    /* ---------- 第 1 步：发起起始信号 ---------- */
    I2C_GenerateSTART(I2C1, ENABLE);                                   /* 置 START 位：SCL 高时 SDA 拉低 = 起始信号 */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;       /* 等"起始已发出"事件（SB 置位） */

    /* ---------- 第 2 步：发从机地址 + 写方向 ---------- */
    I2C_Send7bitAddress(I2C1, MPU_ADDR_W, I2C_Direction_Transmitter);  /* 发 0xD0：地址 0x68 + 写位 0 */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))    /* 等"地址发出且收到 ACK"事件（ADDR 置位） */
        goto fail;                                                     /* 没收到 ACK 说明从机不在或地址错 */

    /* ---------- 第 3 步：发寄存器号 ---------- */
    I2C_SendData(I2C1, reg);                                           /* 把寄存器号写进 DR，硬件自动移位发出去 */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;  /* 等"这个字节发完"事件（BTF 置位） */

    /* ---------- 第 4 步：发数据 ---------- */
    I2C_SendData(I2C1, data);                                          /* 把要写的数据写进 DR 发出去 */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;  /* 再等这个字节发完 */

    /* ---------- 第 5 步：停止信号 ---------- */
    I2C_GenerateSTOP(I2C1, ENABLE);     /* 置 STOP 位：SCL 高时 SDA 拉高 = 停止信号，释放总线 */
    return 1;                           /* 写成功 */

fail:                                   /* 中途任何一步超时都会跳到这里 */
    I2C_GenerateSTOP(I2C1, ENABLE);     /* 兜底：发停止信号，释放总线，防止总线卡死 */
    return 0;                           /* 返回失败 */
}

/* ======================= 五、连续读多个寄存器 ======================= */
static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;                          /* 循环变量：第几个字节 */

    if (!I2C_WaitIdle()) return 0;      /* 总线不空闲，直接失败 */

    /* ---------- 第 1 段：起始 + 地址(写) + 寄存器号 ---------- */
    I2C_GenerateSTART(I2C1, ENABLE);                                   /* 起始信号 */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;       /* 等起始完成 */

    I2C_Send7bitAddress(I2C1, MPU_ADDR_W, I2C_Direction_Transmitter);  /* 发 0xD0（写）：先告诉它"我要读哪个寄存器" */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto fail;  /* 等地址 ACK */

    I2C_SendData(I2C1, reg);                                           /* 发寄存器号，例如 0x3B */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto fail;  /* 等该字节发完 */

    /* ---------- 第 2 段：重复起始 + 地址(读) ---------- */
    I2C_GenerateSTART(I2C1, ENABLE);                                   /* 再一次起始 = 重复起始（不先发 STOP，总线不释放） */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) goto fail;       /* 等起始完成 */

    I2C_Send7bitAddress(I2C1, MPU_ADDR_R, I2C_Direction_Receiver);     /* 发 0xD1（读）：把方向切成"从机发、主机收" */
    if (!I2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto fail;     /* 等地址 ACK（ADDR 置位） */

    /* ---------- 第 3 段：连续接收 len 个字节 ---------- */
    for (i = 0; i < len; i++)                                          /* 一个一个收，共 len 个 */
    {
        if (i == len - 1)                                              /* 收到最后一个字节之前 */
        {
            I2C_AcknowledgeConfig(I2C1, DISABLE);                      /* 关 ACK：最后一个字节回 NACK，告诉从机"别发了" */
            I2C_GenerateSTOP(I2C1, ENABLE);                            /* 提前发 STOP：收完立刻释放总线 */
        }

        if (!I2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) goto fail; /* 等"收到一个字节"事件（RXNE 置位） */
        buf[i] = I2C_ReceiveData(I2C1);                                /* 从 DR 读出这个字节，存进数组（读 DR 会清 RXNE） */
    }

    I2C_AcknowledgeConfig(I2C1, ENABLE);   /* 恢复 ACK，否则后续通信都会回 NACK */
    return 1;                              /* 读成功 */

fail:                                      /* 任何一步超时 */
    I2C_AcknowledgeConfig(I2C1, ENABLE);   /* 恢复 ACK，避免影响下一次通信 */
    I2C_GenerateSTOP(I2C1, ENABLE);        /* 发 STOP 释放总线 */
    return 0;                              /* 返回失败 */
}

/* =================== 六、读 WHO_AM_I 验证通信 =================== */
static uint8_t MPU_WhoAmI(void)
{
    uint8_t id = 0;                            /* 存放读回来的 ID，先清 0 */
    if (!MPU_ReadRegs(0x75, &id, 1))           /* 读 0x75 号寄存器，长度 1 字节 */
        return 0;                              /* 读失败返回 0（正常 ID 是 0x68，所以 0 表示失败） */
    return id;                                 /* 返回读到的 ID */
}

/* ===================== 七、MPU6050 初始化 ===================== */
static uint8_t MPU_Init(void)
{
    if (!MPU_WriteReg(0x6B, 0x00)) return 0;   /* PWR_MGMT_1 写 0：解除睡眠，芯片开始采样 */
    if (!MPU_WriteReg(0x1C, 0x00)) return 0;   /* ACCEL_CONFIG 写 0：±2g 量程，16384 LSB = 1g */
    if (!MPU_WriteReg(0x1B, 0x00)) return 0;   /* GYRO_CONFIG 写 0：±250°/s 量程，131 LSB = 1°/s */
    return 1;                                  /* 三步都成功 */
}

/* ================== 八、一次读 14 字节并拼成数字 ================== */
static uint8_t MPU_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];                                        /* 14 字节原始数据缓冲 */

    if (!MPU_ReadRegs(0x3B, buf, 14)) return 0;             /* 从 0x3B 开始连续读 14 字节 */

    *ax   = (int16_t)((buf[0]  << 8) | buf[1]);             /* AX：高字节 buf[0] 左移 8 位，再或上低字节 buf[1] */
    *ay   = (int16_t)((buf[2]  << 8) | buf[3]);             /* AY：同上，高字节 buf[2] + 低字节 buf[3] */
    *az   = (int16_t)((buf[4]  << 8) | buf[5]);             /* AZ：同上 */
    *temp = (int16_t)((buf[6]  << 8) | buf[7]);             /* 内部温度：换算 T = raw/340 + 36.53 ℃ */
    *gx   = (int16_t)((buf[8]  << 8) | buf[9]);             /* 陀螺 X：/131 得 °/s */
    *gy   = (int16_t)((buf[10] << 8) | buf[11]);            /* 陀螺 Y */
    *gz   = (int16_t)((buf[12] << 8) | buf[13]);            /* 陀螺 Z */

    /* 必须强转 int16_t：加速度有负数，不转的话 0xFFFF 会显示成 65535 而不是 -1 */
    return 1;                                               /* 读取成功 */
}

/* ============================ 九、主函数 ============================ */
int main(void)
{
    int16_t ax, ay, az, temp, gx, gy, gz;   /* 存放三轴加速度、温度、三轴角速度 */
    uint8_t id;                             /* 存放 WHO_AM_I 读到的值 */
    int32_t x100, y100, z100;               /* 加速度换算成"0.01g"单位后的值（1.00g = 100） */
    uint8_t ok;                             /* 保存每次读取是否成功 */

    OLED_Init();                            /* 初始化 OLED（PB8/PB9 软件 I2C） */
    OLED_ShowString(1, 1, "ID:");           /* 第 1 行第 1 列显示 "ID:" */
    OLED_ShowString(2, 1, "X:");            /* 第 2 行显示 "X:"（X 轴加速度） */
    OLED_ShowString(3, 1, "Y:");            /* 第 3 行显示 "Y:" */
    OLED_ShowString(4, 1, "Z:");            /* 第 4 行显示 "Z:" */

    I2C1_Init();                            /* 初始化 I2C1（PB6/PB7，100kHz） */

    /* ---------- 第 1 步：读 WHO_AM_I，确认芯片在不在 ---------- */
    id = MPU_WhoAmI();                      /* 读 0x75，正常应为 0x68 */
    OLED_ShowHexNum(1, 4, id, 2);           /* 把 ID 以 2 位十六进制显示在 "ID:" 后面 */

    if (id != 0x68)                         /* 不是 0x68 说明通信失败 */
    {
        OLED_ShowString(1, 8, "NO MPU");    /* 第 1 行右侧提示 "NO MPU" */
        while (1);                          /* 停在这里，不再往下跑，方便你排查接线 */
    }

    /* ---------- 第 2 步：初始化传感器 ---------- */
    if (!MPU_Init())                        /* 写 0x6B/0x1C/0x1B 三个配置寄存器 */
    {
        OLED_ShowString(1, 8, "INIT ER");   /* 写失败则提示 "INIT ER" */
        while (1);                          /* 停住 */
    }

    /* ---------- 第 3 步：循环读取并显示 ---------- */
    while (1)
    {
        ok = MPU_ReadAll(&ax, &ay, &az, &temp, &gx, &gy, &gz);  /* 读 14 字节并拆分到各变量 */

        if (ok)                                                 /* 读成功才更新显示 */
        {
            /* 16384 LSB = 1g，乘 100 再除 → 单位 0.01g，方便用整数显示 */
            x100 = (int32_t)ax * 100 / 16384;                   /* X 轴：例如 16384 → 100 表示 1.00g */
            y100 = (int32_t)ay * 100 / 16384;                   /* Y 轴：同上 */
            z100 = (int32_t)az * 100 / 16384;                   /* Z 轴：平放时约 +100（1.00g） */

            OLED_ShowSignedNum(2, 3, x100, 3);                  /* 第 2 行第 3 列显示带符号的 X（3 位宽度） */
            OLED_ShowSignedNum(3, 3, y100, 3);                  /* 第 3 行显示 Y */
            OLED_ShowSignedNum(4, 3, z100, 3);                  /* 第 4 行显示 Z */
        }
        else                                                    /* 读失败（比如线松了） */
        {
            OLED_ShowString(1, 8, "READ ER");                   /* 提示 "READ ER" */
        }

        Delay_ms(100);                                          /* 每 100ms 刷新一次，共 10Hz；太快 OLED 刷新不过来 */
    }
}
