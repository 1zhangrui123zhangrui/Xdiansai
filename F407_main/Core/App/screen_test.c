/**
 * @file    screen_test.c
 * @brief   串口屏通信测试实现
 *
 * 测试分两阶段:
 *  Phase 1 (发送测试): 每隔 1s 向屏幕发送一组测试数据，观察屏幕是否刷新
 *  Phase 2 (接收测试): 进入监听模式，按下屏幕任意按键后，
 *                      将收到的原始字节通过 USART3 打印出 16 进制
 *
 * USART3 调试输出格式: "RX: AA BB CC ...\r\n"
 */
#include "screen_test.h"
#include "screen.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* 发送测试轮数 */
#define SEND_TEST_ROUNDS    5U
/* 每轮间隔 (ms) */
#define SEND_TEST_INTERVAL  1500U
/* Phase 2 坐标刷新周期 */
#define COORD_UPDATE_MS      100U
/* 按键结果在屏幕上保留时长 */
#define CMD_HOLD_MS          800U
/* 页面切换后等待屏幕稳定，再写测试结果 */
#define PAGE_SETTLE_DELAY_MS  120U

/* ---- 调试串口输出 (USART3) ---- */
static void dbg_print(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

static void dbg_hex(const uint8_t *buf, uint16_t len)
{
    char tmp[8];
    dbg_print("RX:");
    for (uint16_t i = 0; i < len; i++) {
        snprintf(tmp, sizeof(tmp), " %02X", buf[i]);
        dbg_print(tmp);
    }
    dbg_print("\r\n");
}

static uint8_t find_cmd_code(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (buf[i] >= 0xA0U && buf[i] <= 0xA5U) {
            return buf[i];
        }
    }
    return 0U;
}

static int8_t find_page_code(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (buf[i] >= 0xB0U && buf[i] <= 0xB5U) {
            return (int8_t)(buf[i] - 0xB0U);
        }
    }
    return -1;
}

static void format_fixed1(float value, char *out, size_t out_size)
{
    uint8_t negative = 0U;
    if (value < 0.0f) {
        negative = 1U;
        value = -value;
    }

    uint32_t scaled = (uint32_t)(value * 10.0f + 0.5f);
    uint32_t whole  = scaled / 10U;
    uint32_t frac   = scaled % 10U;

    if (scaled == 0U) {
        negative = 0U;
    }

    snprintf(out,
             out_size,
             negative ? "-%lu.%01lu" : "%lu.%01lu",
             (unsigned long)whole,
             (unsigned long)frac);
}

static void next_test_coord(float *x, float *y)
{
    static const float k_points[][2] = {
        {-10.0f, -6.0f},
        {-7.5f,  -4.5f},
        {-5.0f,  -3.0f},
        {-2.5f,  -1.5f},
        { 0.0f,   0.0f},
        { 2.5f,   1.5f},
        { 5.0f,   3.0f},
        { 7.5f,   4.5f},
        {10.0f,   6.0f},
        { 5.0f,   0.0f},
        { 0.0f,  -4.0f},
        {-5.0f,   0.0f},
    };
    static uint8_t s_idx = 0;

    *x = k_points[s_idx][0];
    *y = k_points[s_idx][1];
    s_idx = (uint8_t)((s_idx + 1U) % (sizeof(k_points) / sizeof(k_points[0])));
}

/* ---- 对外接口 ---- */

void ScreenTest_Run(void)
{
    /* 直接初始化串口屏，不通过 Screen_Init (避免中断接收与轮询冲突) */
    extern UART_HandleTypeDef huart2;
    /* huart2 已由 MX_USART2_UART_Init 配置好，这里只需绑定到 screen */
    Screen_Init(&huart2);
    /* 关闭中断接收，改用轮询 */
    HAL_UART_AbortReceive(&huart2);

    dbg_print("=== Screen Test Start ===\r\n");

    /* ---------- Phase 1: 发送测试 ----------
     * 向屏幕发送测试坐标，观察屏幕文本框是否刷新 */
    dbg_print("[Phase 1] Sending display test data...\r\n");

    for (uint8_t r = 0; r < SEND_TEST_ROUNDS; r++) {
        float x = (float)r * 5.0f - 10.0f;   /* -10, -5, 0, 5, 10 */
        float y = (float)r * 3.0f - 6.0f;

        Screen_SetCoord(x, y);

        if (r == 2U) {
            Screen_RecordFire(x, y);
        }

        char xbuf[16];
        char ybuf[16];
        char msg[64];
        format_fixed1(x, xbuf, sizeof(xbuf));
        format_fixed1(y, ybuf, sizeof(ybuf));
        snprintf(msg, sizeof(msg), "[TX] coord x=%s y=%s\r\n", xbuf, ybuf);
        dbg_print(msg);

        HAL_Delay(SEND_TEST_INTERVAL);
    }

    dbg_print("[Phase 1] Done. Check screen display.\r\n\r\n");

    /* ---------- Phase 2: 接收测试 ----------
     * 长时间轮询 USART2，收到按键后等待页面跳转完成，再把结果写到当前页 tX/tY
     * 例如按下区域巡逻后，page 2 稳定后显示:
     *   tX = CMD:A2
     *   tY = PAGE:2
     */
    dbg_print("[Phase 2] Listening continuously, result shown on current page tX/tY.\r\n");

    uint32_t last_coord_ms = 0;
    uint32_t cmd_hold_until = 0;
    float cur_x = 0.0f;
    float cur_y = 0.0f;

    while (1) {
        uint32_t now = HAL_GetTick();

        /* 默认持续刷新测试坐标，验证 MCU -> Screen 实时发送 */
        if (now - last_coord_ms >= COORD_UPDATE_MS) {
            last_coord_ms = now;
            next_test_coord(&cur_x, &cur_y);
            if (now >= cmd_hold_until) {
                Screen_SetCoord(cur_x, cur_y);
            }
        }

        uint8_t buf[16];
        uint16_t cnt = 0;

        /* 短超时轮询，避免阻塞坐标刷新 */
        uint8_t b;
        if (HAL_UART_Receive(&huart2, &b, 1, 10) != HAL_OK) {
            continue;
        }
        buf[cnt++] = b;

        /* 继续读同一包剩余字节 */
        while (cnt < sizeof(buf)) {
            if (HAL_UART_Receive(&huart2, &b, 1, 10) == HAL_OK) {
                buf[cnt++] = b;
            } else {
                break;
            }
        }

        if (cnt == 0U) {
            continue;
        }

        dbg_hex(buf, cnt);

        uint8_t cmd = find_cmd_code(buf, cnt);
        int8_t page = find_page_code(buf, cnt);

        /* 按键通常伴随 page 跳转，等新页面创建完控件后再写文本 */
        HAL_Delay((page >= 0) ? PAGE_SETTLE_DELAY_MS : 20U);
        cmd_hold_until = HAL_GetTick() + CMD_HOLD_MS;

        if (cmd != 0U) {
            char tx[24];
            snprintf(tx, sizeof(tx), "CMD:%02X", cmd);
            Screen_SetText("tX", tx);
        }

        if (page >= 0) {
            char ty[24];
            snprintf(ty, sizeof(ty), "PAGE:%d", (int)page);
            Screen_SetText("tY", ty);
        } else if (cmd != 0U) {
            Screen_SetText("tY", "RX OK");
        }
    }
}
