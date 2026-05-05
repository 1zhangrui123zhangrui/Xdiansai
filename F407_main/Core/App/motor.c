/**
 * @file    motor.c
 * @brief   ZDT X42S 电机驱动实现 (X 固件)
 *
 * X 固件关键命令 (默认校验字节 0x6B):
 *  使能:  Addr F3 AB 01/00 00 6B  (6 字节)
 *  停止:  Addr FE 98 00 6B        (5 字节)
 *  清零:  Addr 0A 6D 6B           (4 字节)
 *  位置:  Addr FD dir acc dec speed angle mode sync 6B (16 字节)
 *  同步:  00 FF 66 6B              (4 字节)
 *  多机:  00 AA len_hi len_lo [cmds...] 6B
 */
#include "motor.h"
#include "platform_config.h"
#include <math.h>
#include <string.h>

#define MOTOR_CMD_MULTI         0xAAU
#define MOTOR_X_POS_CMD_LEN     16U
#define MOTOR_X_SPEED_MAX_01RPM 30000U
#define MOTOR_CMD_READ_POSITION 0x36U
#define MOTOR_CMD_READ_ERROR    0x37U
#define MOTOR_CMD_READ_STATUS   0x3AU
#define MOTOR_STATUS_REACHED    0x02U
#define MOTOR_RX_TIMEOUT_MS     20U

static UART_HandleTypeDef *s_huart = NULL;

/* ---------- 内部辅助 ---------- */

static void send(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(s_huart, (uint8_t *)buf, len, 50U);
}

static void clear_rx_fifo(void)
{
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_RXNE)) {
        (void)s_huart->Instance->DR;
    }
}

static uint8_t query(uint8_t id, uint8_t cmd, uint8_t *rx, uint16_t rx_len)
{
    uint8_t tx[3] = {id, cmd, MOTOR_CHECKSUM};

    clear_rx_fifo();
    if (HAL_UART_Transmit(s_huart, tx, sizeof(tx), 20U) != HAL_OK) {
        return 0;
    }
    if (HAL_UART_Receive(s_huart, rx, rx_len, MOTOR_RX_TIMEOUT_MS) != HAL_OK) {
        return 0;
    }
    if (rx[0] != id || rx[1] != cmd || rx[rx_len - 1U] != MOTOR_CHECKSUM) {
        return 0;
    }
    return 1;
}

static int32_t read_i32_be(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) |
                     ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) |
                     (uint32_t)p[3]);
}

static uint16_t clamp_u16(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

static float motor_dir_sign(uint8_t id)
{
    switch (id) {
    case MOTOR_ID_1: return MOTOR1_DIR_SIGN;
    case MOTOR_ID_2: return MOTOR2_DIR_SIGN;
    case MOTOR_ID_3: return MOTOR3_DIR_SIGN;
    case MOTOR_ID_4: return MOTOR4_DIR_SIGN;
    default:         return 1.0f;
    }
}

/**
 * 构造单条 X 固件 FD 梯形曲线位置命令 → buf, 返回字节数 (16)
 */
static uint16_t build_fd_x(uint8_t *buf,
                           uint8_t  id,
                           float    angle_deg,
                           float    speed_rpm,
                           uint16_t accel_rpms,
                           uint16_t decel_rpms,
                           uint8_t  mode,
                           uint8_t  sync)
{
    angle_deg *= motor_dir_sign(id);

    uint8_t  dir = (angle_deg >= 0.0f) ? 0x00U : 0x01U;
    uint32_t angle_0p1deg = (uint32_t)(fabsf(angle_deg) * 10.0f + 0.5f);
    uint32_t speed_0p1rpm = (uint32_t)(fabsf(speed_rpm) * 10.0f + 0.5f);
    if (speed_0p1rpm > MOTOR_X_SPEED_MAX_01RPM) speed_0p1rpm = MOTOR_X_SPEED_MAX_01RPM;

    uint8_t i = 0;
    buf[i++] = id;
    buf[i++] = 0xFDU;
    buf[i++] = dir;
    buf[i++] = (uint8_t)(accel_rpms >> 8);
    buf[i++] = (uint8_t)(accel_rpms);
    buf[i++] = (uint8_t)(decel_rpms >> 8);
    buf[i++] = (uint8_t)(decel_rpms);
    buf[i++] = (uint8_t)(speed_0p1rpm >> 8);
    buf[i++] = (uint8_t)(speed_0p1rpm);
    buf[i++] = (uint8_t)(angle_0p1deg >> 24);
    buf[i++] = (uint8_t)(angle_0p1deg >> 16);
    buf[i++] = (uint8_t)(angle_0p1deg >> 8);
    buf[i++] = (uint8_t)(angle_0p1deg);
    buf[i++] = mode;
    buf[i++] = sync;
    buf[i++] = MOTOR_CHECKSUM;
    return i;  /* 16 */
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

void Motor_DisableAll(void)
{
    for (uint8_t id = MOTOR_ID_1; id <= MOTOR_ID_4; id++) {
        Motor_Disable(id);
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

uint8_t Motor_ReadStatus(uint8_t id, uint8_t *status_flags)
{
    uint8_t rx[4];

    if (status_flags == NULL) {
        return 0;
    }
    if (!query(id, MOTOR_CMD_READ_STATUS, rx, sizeof(rx))) {
        return 0;
    }

    *status_flags = rx[2];
    return 1;
}

uint8_t Motor_IsReached(uint8_t id)
{
    uint8_t status = 0;

    if (!Motor_ReadStatus(id, &status)) {
        return 0;
    }
    return ((status & MOTOR_STATUS_REACHED) != 0U) ? 1U : 0U;
}

uint8_t Motor_AllReached(void)
{
    for (uint8_t id = MOTOR_ID_1; id <= MOTOR_ID_4; id++) {
        if (!Motor_IsReached(id)) {
            return 0;
        }
    }
    return 1;
}

uint8_t Motor_ReadPositionDeg(uint8_t id, float *angle_deg)
{
    uint8_t rx[8];
    int32_t raw;
    float sign;

    if (angle_deg == NULL) {
        return 0;
    }
    if (!query(id, MOTOR_CMD_READ_POSITION, rx, sizeof(rx))) {
        return 0;
    }

    sign = (rx[2] == 0x01U) ? -1.0f : 1.0f;
    raw = read_i32_be(&rx[3]);
    *angle_deg = sign * ((float)raw / 10.0f) * motor_dir_sign(id);
    return 1;
}

uint8_t Motor_ReadAllPositions(float angle_deg[4])
{
    if (angle_deg == NULL) {
        return 0;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (!Motor_ReadPositionDeg((uint8_t)(MOTOR_ID_1 + i), &angle_deg[i])) {
            return 0;
        }
    }
    return 1;
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
                        uint16_t accel_rpms,
                        uint16_t decel_rpms,
                        uint8_t sync)
{
    uint8_t buf[MOTOR_X_POS_CMD_LEN];
    uint16_t len = build_fd_x(buf, id, angle_deg, speed_rpm,
                              clamp_u16(accel_rpms),
                              clamp_u16(decel_rpms),
                              0x01U,  /* mode=1: 绝对零点 */
                              sync);
    send(buf, len);
}

void Motor_MoveRelative(uint8_t id,
                        float   delta_deg,
                        float   speed_rpm,
                        uint16_t accel_rpms,
                        uint16_t decel_rpms,
                        uint8_t sync)
{
    uint8_t buf[MOTOR_X_POS_CMD_LEN];
    uint16_t len = build_fd_x(buf, id, delta_deg, speed_rpm,
                              clamp_u16(accel_rpms),
                              clamp_u16(decel_rpms),
                              0x02U,  /* mode=2: 相对当前位置 */
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
                       uint16_t accel_rpms,
                       uint16_t decel_rpms)
{
    float speeds[4] = {speed_rpm, speed_rpm, speed_rpm, speed_rpm};
    Motor_MoveAllSyncSpeeds(angle_deg, speeds, accel_rpms, decel_rpms);
}

void Motor_MoveAllSyncSpeeds(const float angle_deg[4],
                             const float speed_rpm[4],
                             uint16_t accel_rpms,
                             uint16_t decel_rpms)
{
    for (uint8_t i = 0; i < 4; i++) {
        Motor_MoveAbsolute((uint8_t)(MOTOR_ID_1 + i),
                           angle_deg[i], speed_rpm[i],
                           accel_rpms, decel_rpms,
                           0x01U);  /* sync=1: 缓存 */
        HAL_Delay(2);
    }
    Motor_TriggerSync();
}

/**
 * 多机命令帧原子发送
 * 总帧 = 00 AA len_hi len_lo [4×16字节命令] 6B = 4+64+1 = 69 字节
 */
void Motor_MultiPositionCmd(const float angle_deg[4],
                            float speed_rpm,
                            uint16_t accel_rpms,
                            uint16_t decel_rpms)
{
    float speeds[4] = {speed_rpm, speed_rpm, speed_rpm, speed_rpm};
    Motor_MultiPositionCmdSpeeds(angle_deg, speeds, accel_rpms, decel_rpms);
}

void Motor_MultiPositionCmdSpeeds(const float angle_deg[4],
                                  const float speed_rpm[4],
                                  uint16_t accel_rpms,
                                  uint16_t decel_rpms)
{
    uint8_t cmds[MOTOR_X_POS_CMD_LEN * 4];
    uint16_t cmd_total = 0;

    for (uint8_t i = 0; i < 4; i++) {
        cmd_total += build_fd_x(cmds + cmd_total,
                                (uint8_t)(MOTOR_ID_1 + i),
                                angle_deg[i], speed_rpm[i],
                                clamp_u16(accel_rpms),
                                clamp_u16(decel_rpms),
                                0x01U,   /* mode=1: 绝对零点 */
                                0x00U);  /* sync=0: 立即执行 */
    }

    /* len 字段 = 命令内容字节数 (不含 00 AA / len / 6B) */
    uint8_t frame[4 + MOTOR_X_POS_CMD_LEN * 4 + 1];
    frame[0] = MOTOR_ID_BROADCAST;
    frame[1] = MOTOR_CMD_MULTI;
    frame[2] = (uint8_t)(cmd_total >> 8);
    frame[3] = (uint8_t)(cmd_total);
    memcpy(frame + 4, cmds, cmd_total);
    frame[4 + cmd_total] = MOTOR_CHECKSUM;

    send(frame, (uint16_t)(4U + cmd_total + 1U));
}

void Motor_MultiPositionDeltaCmdSpeeds(const float delta_deg[4],
                                       const float speed_rpm[4],
                                       uint16_t accel_rpms,
                                       uint16_t decel_rpms)
{
    uint8_t cmds[MOTOR_X_POS_CMD_LEN * 4];
    uint16_t cmd_total = 0;

    for (uint8_t i = 0; i < 4; i++) {
        cmd_total += build_fd_x(cmds + cmd_total,
                                (uint8_t)(MOTOR_ID_1 + i),
                                delta_deg[i], speed_rpm[i],
                                clamp_u16(accel_rpms),
                                clamp_u16(decel_rpms),
                                0x02U,   /* mode=2: 相对当前位置 */
                                0x00U);
    }

    uint8_t frame[4 + MOTOR_X_POS_CMD_LEN * 4 + 1];
    frame[0] = MOTOR_ID_BROADCAST;
    frame[1] = MOTOR_CMD_MULTI;
    frame[2] = (uint8_t)(cmd_total >> 8);
    frame[3] = (uint8_t)(cmd_total);
    memcpy(frame + 4, cmds, cmd_total);
    frame[4 + cmd_total] = MOTOR_CHECKSUM;

    send(frame, (uint16_t)(4U + cmd_total + 1U));
}

void Motor_MultiPositionDeltaCmd(const float delta_deg[4],
                                 float speed_rpm,
                                 uint16_t accel_rpms,
                                 uint16_t decel_rpms)
{
    float speeds[4] = {speed_rpm, speed_rpm, speed_rpm, speed_rpm};
    Motor_MultiPositionDeltaCmdSpeeds(delta_deg, speeds, accel_rpms, decel_rpms);
}
