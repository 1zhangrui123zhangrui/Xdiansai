/**
 * @file    screen.c
 * @brief   串口屏驱动实现 (来源: 串口屏.md 最终版方案)
 */
#include "screen.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *s_huart    = NULL;
static uint8_t             s_rx_byte  = 0;
static uint8_t             s_cur_page = 0;

static float   s_fire_x[2] = {0};
static float   s_fire_y[2] = {0};
static uint8_t s_fire_cnt  = 0;

#define MAX_CMD_CB  8
typedef struct { uint8_t cmd; ScreenCmdCallback_t cb; } CmdEntry_t;
static CmdEntry_t          s_cmd_tbl[MAX_CMD_CB];
static uint8_t             s_cmd_cnt     = 0;
static ScreenSeqCallback_t s_seq_cb      = NULL;

typedef enum { RX_HEAD1, RX_HEAD2, RX_CMD, RX_SEQ } RxState_t;
static RxState_t s_rx_state = RX_HEAD1;
static uint8_t   s_seq_buf[5];
static uint8_t   s_seq_idx  = 0;

static const uint8_t k_end[3] = {0xFF, 0xFF, 0xFF};

/* ---------- 内部 ---------- */

static void raw_send(const char *str)
{
    HAL_UART_Transmit(s_huart, (uint8_t *)str, (uint16_t)strlen(str), 100);
    HAL_UART_Transmit(s_huart, (uint8_t *)k_end, 3, 100);
}

static void dispatch(uint8_t cmd)
{
    for (uint8_t i = 0; i < s_cmd_cnt; i++) {
        if (s_cmd_tbl[i].cmd == cmd && s_cmd_tbl[i].cb) {
            s_cmd_tbl[i].cb();
            return;
        }
    }
}

static void flush_fire(void)
{
    char buf[40];
    for (uint8_t i = 0; i < s_fire_cnt; i++) {
        snprintf(buf, sizeof(buf), "tF%dX.txt=\"%.2f\"", i + 1, (double)s_fire_x[i]);
        raw_send(buf);
        snprintf(buf, sizeof(buf), "tF%dY.txt=\"%.2f\"", i + 1, (double)s_fire_y[i]);
        raw_send(buf);
    }
}

/* ---------- 对外接口 ---------- */

void Screen_Init(UART_HandleTypeDef *huart)
{
    s_huart     = huart;
    s_cmd_cnt   = 0;
    s_seq_cb    = NULL;
    s_rx_state  = RX_HEAD1;
    s_cur_page  = 0;
    s_fire_cnt  = 0;
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void Screen_RegisterCallback(uint8_t cmd, ScreenCmdCallback_t cb)
{
    for (uint8_t i = 0; i < s_cmd_cnt; i++) {
        if (s_cmd_tbl[i].cmd == cmd) { s_cmd_tbl[i].cb = cb; return; }
    }
    if (s_cmd_cnt < MAX_CMD_CB) {
        s_cmd_tbl[s_cmd_cnt].cmd = cmd;
        s_cmd_tbl[s_cmd_cnt].cb  = cb;
        s_cmd_cnt++;
    }
}

void Screen_RegisterSequenceCallback(ScreenSeqCallback_t cb)
{
    s_seq_cb = cb;
}

uint8_t Screen_GetCurrentPage(void)
{
    return s_cur_page;
}

void Screen_SetCoord(float x, float y)
{
    char buf[40];
    snprintf(buf, sizeof(buf), "tX.txt=\"%.2f\"", (double)x);
    raw_send(buf);
    snprintf(buf, sizeof(buf), "tY.txt=\"%.2f\"", (double)y);
    raw_send(buf);
}

void Screen_RecordFire(float x, float y)
{
    if (s_fire_cnt >= 2) return;
    s_fire_x[s_fire_cnt] = x;
    s_fire_y[s_fire_cnt] = y;
    s_fire_cnt++;
    if (s_cur_page == 5) flush_fire();
}

void Screen_SetText(const char *objname, const char *text)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", objname, text);
    raw_send(buf);
}

void Screen_OnByteReceived(uint8_t byte)
{
    switch (s_rx_state) {
    case RX_HEAD1:
        if (byte == 0x55U) s_rx_state = RX_HEAD2;
        break;

    case RX_HEAD2:
        s_rx_state = (byte == 0x55U) ? RX_CMD : RX_HEAD1;
        break;

    case RX_CMD:
        if (byte >= 0xB0U && byte <= 0xBFU) {
            s_cur_page = byte - 0xB0U;
            if (s_cur_page == 5) flush_fire();
            s_rx_state = RX_HEAD1;
        } else if (byte == CMD_SEQ_PATROL) {
            s_seq_idx  = 0;
            s_rx_state = RX_SEQ;
        } else {
            dispatch(byte);
            s_rx_state = RX_HEAD1;
        }
        break;

    case RX_SEQ:
        if (byte == 0xFEU) {
            if (s_seq_idx == 5 && s_seq_cb) {
                uint8_t seq[5];
                uint8_t valid = 1;
                for (uint8_t i = 0; i < 5; i++) {
                    if (s_seq_buf[i] >= '1' && s_seq_buf[i] <= '5') {
                        seq[i] = s_seq_buf[i] - '0';
                    } else {
                        valid = 0;
                        break;
                    }
                }
                if (valid) s_seq_cb(seq);
            }
            s_rx_state = RX_HEAD1;
        } else if (s_seq_idx < 5) {
            s_seq_buf[s_seq_idx++] = byte;
        } else {
            s_rx_state = RX_HEAD1;
        }
        break;
    }
}

/* HAL 中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        Screen_OnByteReceived(s_rx_byte);
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    }
}
