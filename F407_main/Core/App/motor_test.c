/**
 * @file    motor_test.c
 * @brief   电机驱动验证测试
 *
 * 直接通过 USART1 发送 Emm 固件命令，不依赖运动学模块。
 * 所有命令帧均按手册 5.3.2 / 5.3.12 / 5.3.15 构造。
 *
 * 帧格式 (Emm 固件, 默认校验 0x6B):
 *  使能: Addr F3 AB 01 00 6B
 *  停止: Addr FE 98 00 6B
 *  位置: Addr FD dir spd_h spd_l acc p3 p2 p1 p0 mode sync 6B
 *    mode=02 → 相对当前位置运动
 */
#include "motor_test.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

/* 电机 1 地址 */
#define TEST_MOTOR_ID   0x01U
#define TEST_CHECKSUM   0x6BU

/* 1.8° 步进 + 16 细分 = 3200 脉冲/圈 */
#define PULSES_ONE_REV  3200U

/* 测试速度 (RPM): 低速便于观察 */
#define TEST_SPEED_RPM  200U

/* 加速档位: 0 = 直接以设定速度启动 */
#define TEST_ACC        0U

/* 等待电机运动完成的延时 (ms): 1圈 at 200RPM → 0.3s, 留余量 */
#define TEST_WAIT_MS    3000U
#define TEST_REPEAT     3U

/* ---- 内部函数 ---- */

static void uart1_send(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 100U);
}

/** 发送使能命令 */
static void test_enable(void)
{
    uint8_t cmd[] = {TEST_MOTOR_ID, 0xF3U, 0xABU, 0x01U, 0x00U, TEST_CHECKSUM};
    uart1_send(cmd, sizeof(cmd));
    HAL_Delay(50);
}

/** 发送停止命令 */
static void test_stop(void)
{
    uint8_t cmd[] = {TEST_MOTOR_ID, 0xFEU, 0x98U, 0x00U, TEST_CHECKSUM};
    uart1_send(cmd, sizeof(cmd));
}

/**
 * 发送位置命令 (mode=02, 相对当前位置)
 * @param cw       1=正转(CW), 0=反转(CCW)
 * @param pulses   脉冲数
 */
static void test_move_relative(uint8_t cw, uint32_t pulses)
{
    uint8_t dir  = cw ? 0x00U : 0x01U;
    uint16_t spd = TEST_SPEED_RPM;
    uint8_t cmd[13];
    uint8_t i = 0;
    cmd[i++] = TEST_MOTOR_ID;
    cmd[i++] = 0xFDU;
    cmd[i++] = dir;
    cmd[i++] = (uint8_t)(spd >> 8);
    cmd[i++] = (uint8_t)(spd);
    cmd[i++] = TEST_ACC;
    cmd[i++] = (uint8_t)(pulses >> 24);
    cmd[i++] = (uint8_t)(pulses >> 16);
    cmd[i++] = (uint8_t)(pulses >> 8);
    cmd[i++] = (uint8_t)(pulses);
    cmd[i++] = 0x02U;           /* mode=2: 相对当前位置 */
    cmd[i++] = 0x00U;           /* sync=0: 立即执行 */
    cmd[i++] = TEST_CHECKSUM;
    uart1_send(cmd, 13U);
}

/* ---- 对外接口 ---- */

void MotorTest_Run(void)
{
    /* 步骤 1: 使能电机 */
    test_enable();

    for (uint8_t r = 0; r < TEST_REPEAT; r++) {
        /* 步骤 2: 正转 1 圈 (CW) */
      //  test_move_relative(1U, PULSES_ONE_REV);
      //  HAL_Delay(TEST_WAIT_MS);

        /* 步骤 3: 反转 1 圈 (CCW) 回原位 */
      test_move_relative(0U, PULSES_ONE_REV);
       HAL_Delay(TEST_WAIT_MS);
    }

    /* 步骤 4: 停止 */
    test_stop();
}
