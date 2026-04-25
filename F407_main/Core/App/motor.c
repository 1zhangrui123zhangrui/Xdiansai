/**
 * @file    motor.c
 * @brief   ZDT X42S 电机驱动实现 (Emm 固件)
 *
 * Emm 固件关键命令 (默认校验字节 0x6B):
 *  使能:  Addr F3 AB 01/00 00 6B  (6 字节)
 *  停止:  Addr FE 98 00 6B        (5 字节)
 *  清零:  Addr 0A 6D 6B           (4 字节)
 *  位置:  Addr FD dir spd_hi spd_lo acc p3 p2 p1 p0 mode sync 6B (13 字节)
 *  同步:  00 FF 66 6B              (4 字节)
 *  多机:  00 AA len_hi len_lo [cmds...] 6B
 *
 * 脉冲换算: 1.8°步进, 16细分 → 3200 脉冲/圈 (360°)
 *          pulses = |angle_deg| / 360.0 × 3200
 */
#include "motor.h"
#include "platform_config.h"
#include <math.h>
#include <string.h>

#define PULSES_PER_REV      3200U   /* 脉冲/圈 (1.8°步进, 16细分) */
#define MOTOR_CMD_MULTI     0xAAU

static UART_HandleTypeDef *s_huart = NULL;

/* ---------- 内部辅助 ---------- */

static void send(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, len, 50U);
}

/**
 * 构造单条 Emm FD 位置命令 → buf, 返回字节数 (13)
 */
static uint16_t build_fd(uint8_t *buf,
                          uint8_t  id,
                          float    angle_deg,
                          float    speed_rpm,
                          uint8_t  acc,
                          uint8_t  mode,
                          uint8_t  sync)
{
    uint8_t  dir     = (angle_deg >= 0.0f) ? 0x00U : 0x01U;
    uint32_t pulses  = (uint32_t)(fabsf(angle_deg) / 360.0f * (float)PULSES_PER_REV);
    uint16_t spd     = (uint16_t)fabsf(speed_rpm);
    if (spd > 3000U) spd = 3000U;

    uint8_t i = 0;
    buf[i++] = id;
    buf[i++] = 0xFDU;
    buf[i++] = dir;
    buf[i++] = (uint8_t)(spd >> 8);
    buf[i++] = (uint8_t)(spd);
    buf[i++] = acc;
    buf[i++] = (uint8_t)(pulses >> 24);
    buf[i++] = (uint8_t)(pulses >> 16);
    buf[i++] = (uint8_t)(pulses >> 8);
    buf[i++] = (uint8_t)(pulses);
    buf[i++] = mode;
    buf[i++] = sync;
    buf[i++] = MOTOR_CHECKSUM;
    return i;  /* 13 */
}

/* ---------- 对外接口 ---------- */

void Motor_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
}

void Motor_Enable(uint8_t id)
{
    uint8_t buf[] = {id, 0xF3U, 0xABU, 0x01U, 0x00U, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_Disable(uint8_t id)
{
    uint8_t buf[] = {id, 0xF3U, 0xABU, 0x00U, 0x00U, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_EnableAll(void)
{
    for (uint8_t id = MOTOR_ID_1; id <= MOTOR_ID_4; id++) {
        Motor_Enable(id);
        HAL_Delay(10);
    }
}

void Motor_Stop(uint8_t id)
{
    uint8_t buf[] = {id, 0xFEU, 0x98U, 0x00U, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_StopAll(void)
{
    uint8_t buf[] = {MOTOR_ID_BROADCAST, 0xFEU, 0x98U, 0x00U, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_ZeroPosition(uint8_t id)
{
    /* 将当前位置角度清零: Addr 0A 6D 6B */
    uint8_t buf[] = {id, 0x0AU, 0x6DU, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_ZeroAllPositions(void)
{
    for (uint8_t id = MOTOR_ID_1; id <= MOTOR_ID_4; id++) {
        Motor_ZeroPosition(id);
        HAL_Delay(20);
    }
}

void Motor_MoveAbsolute(uint8_t id,
                        float   angle_deg,
                        float   speed_rpm,
                        uint8_t acc,
                        uint8_t sync)
{
    uint8_t buf[13];
    uint16_t len = build_fd(buf, id, angle_deg, speed_rpm, acc,
                            0x01U,  /* mode=1: 绝对零点 */
                            sync);
    send(buf, len);
}

void Motor_TriggerSync(void)
{
    uint8_t buf[] = {MOTOR_ID_BROADCAST, 0xFFU, 0x66U, MOTOR_CHECKSUM};
    send(buf, sizeof(buf));
}

void Motor_MoveAllSync(const float angle_deg[4],
                       float speed_rpm,
                       uint8_t acc)
{
    for (uint8_t i = 0; i < 4; i++) {
        Motor_MoveAbsolute((uint8_t)(MOTOR_ID_1 + i),
                           angle_deg[i], speed_rpm, acc,
                           0x01U);  /* sync=1: 缓存 */
        HAL_Delay(2);
    }
    Motor_TriggerSync();
}

/**
 * 多机命令帧原子发送
 * 总帧 = 00 AA len_hi len_lo [4×13字节命令] 6B = 4+52+1 = 57 字节
 * 总长 total_len = 4+52+1 = 57
 */
void Motor_MultiPositionCmd(const float angle_deg[4],
                            float speed_rpm,
                            uint8_t acc)
{
    uint8_t cmds[13 * 4];
    uint16_t cmd_total = 0;

    for (uint8_t i = 0; i < 4; i++) {
        cmd_total += build_fd(cmds + cmd_total,
                              (uint8_t)(MOTOR_ID_1 + i),
                              angle_deg[i], speed_rpm, acc,
                              0x01U,   /* mode=1: 绝对零点 */
                              0x00U);  /* sync=0: 立即执行 */
    }

    /* len 字段 = 命令内容字节数 (不含 00 AA / len / 6B) */
    uint8_t frame[4 + 13 * 4 + 1];
    frame[0] = MOTOR_ID_BROADCAST;
    frame[1] = MOTOR_CMD_MULTI;
    frame[2] = (uint8_t)(cmd_total >> 8);
    frame[3] = (uint8_t)(cmd_total);
    memcpy(frame + 4, cmds, cmd_total);
    frame[4 + cmd_total] = MOTOR_CHECKSUM;

    send(frame, (uint16_t)(4U + cmd_total + 1U));
}
