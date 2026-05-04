/**
 * @file    task.c
 * @brief   任务状态机实现
 *
 * 运动执行流程:
 *  1. 计算目标平台中心坐标 (cx = laser_x - LASER_OFFSET_X, cy = laser_y)
 *  2. 调用 Kinematics_CamToAngles 得到四电机绝对角度
 *  3. 启动前先使能电机, 再调用 X 固件梯形曲线位置命令
 *  4. 等待 MOTOR_MOVE_TIMEOUT_MS 后判定到位 (开环)
 */
#include "task.h"
#include "platform_config.h"
#include "motor.h"
#include "kinematics.h"
#include "screen.h"
#include "nrf_app.h"
#include "buzzer.h"
#include <string.h>
#include <math.h>

/* ============================================================
 * 橙色圆激光目标坐标表 (cm, index 1~5 对应圆 1~5)
 * ============================================================ */
static const float k_circles[5][2] = {
    {CIRCLE1_X, CIRCLE1_Y},   /* [0]: 圆1 左下 */
    {CIRCLE2_X, CIRCLE2_Y},   /* [1]: 圆2 右下 */
    {CIRCLE3_X, CIRCLE3_Y},   /* [2]: 圆3 右上 */
    {CIRCLE4_X, CIRCLE4_Y},   /* [3]: 圆4 左上 */
    {CIRCLE5_X, CIRCLE5_Y},   /* [4]: 圆5 中心 */
};

/* ============================================================
 * 蛇形巡逻路点生成
 * 从圆1 (左下) 开始, 10cm 间隔遍历, 结束于对角圆 (右上/左上)
 * 覆盖范围: x ∈ [-20,20], y ∈ [-20,20], 步长 10cm → 5×5=25 点
 * ============================================================ */
#define SNAKE_COLS  5
#define SNAKE_ROWS  5
static float s_snake[SNAKE_ROWS * SNAKE_COLS][2];
static uint8_t s_snake_cnt = 0;

static void generate_snake(void)
{
    s_snake_cnt = 0;
    float x_vals[SNAKE_COLS] = {
        AUTO_PATROL_X_MIN_CM,
        AUTO_PATROL_X_MIN_CM + AUTO_PATROL_STEP_CM,
        AUTO_PATROL_X_MIN_CM + 2 * AUTO_PATROL_STEP_CM,
        AUTO_PATROL_X_MIN_CM + 3 * AUTO_PATROL_STEP_CM,
        AUTO_PATROL_X_MAX_CM,
    };
    float y_vals[SNAKE_ROWS] = {
        AUTO_PATROL_Y_MIN_CM,
        AUTO_PATROL_Y_MIN_CM + AUTO_PATROL_STEP_CM,
        AUTO_PATROL_Y_MIN_CM + 2 * AUTO_PATROL_STEP_CM,
        AUTO_PATROL_Y_MIN_CM + 3 * AUTO_PATROL_STEP_CM,
        AUTO_PATROL_Y_MAX_CM,
    };
    for (uint8_t r = 0; r < SNAKE_ROWS; r++) {
        for (uint8_t c = 0; c < SNAKE_COLS; c++) {
            uint8_t col = (r % 2 == 0) ? c : (SNAKE_COLS - 1 - c);
            s_snake[s_snake_cnt][0] = x_vals[col];
            s_snake[s_snake_cnt][1] = y_vals[r];
            s_snake_cnt++;
        }
    }
}

/* ============================================================
 * 内部状态
 * ============================================================ */
static TaskState_t s_state      = TASK_IDLE;
static uint8_t     s_wp_idx     = 0;       /* 当前路点索引 */
static uint8_t     s_wp_total   = 0;       /* 总路点数 */
static uint8_t     s_seq[5];               /* 顺序巡逻序列 */
static uint32_t    s_move_start = 0;       /* 本段运动开始时间 */
static float       s_cam_x      = 0.0f;   /* 当前摄像头坐标 */
static float       s_cam_y      = 0.0f;
static float       s_laser_x    = 0.0f;   /* 当前激光坐标 */
static float       s_laser_y    = 0.0f;
static float       s_motor_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t     s_moving     = 0;       /* 1=等待电机运动完成 */
static uint8_t     s_fire_halt  = 0;       /* 检测到火源后暂停计数 */
static uint8_t     s_motor_enabled = 0;    /* 上电清零后默认松轴 */
static uint8_t     s_angle_known = 0;      /* 手拉后未知, 回中心后重新可信 */
static volatile uint8_t s_estop_pending = 0;

static void enable_for_motion(void)
{
    if (s_motor_enabled) return;
    Motor_EnableAll();
    HAL_Delay(50);
    s_motor_enabled = 1;
}

static void calc_sync_speeds(const float target_angles[4], float speeds[4])
{
    float max_delta = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float d = fabsf(target_angles[i] - s_motor_angle[i]);
        if (d > max_delta) max_delta = d;
    }

    if (!s_angle_known || max_delta < 0.1f ||
        MOTOR_ACCEL_RPMS == 0U || MOTOR_DECEL_RPMS == 0U) {
        for (uint8_t i = 0; i < 4; i++) speeds[i] = (float)MOTOR_SPEED_RPM;
        return;
    }

    float accel = (float)MOTOR_ACCEL_RPMS * 6.0f;  /* RPM/S -> deg/s^2 */
    float decel = (float)MOTOR_DECEL_RPMS * 6.0f;
    float vmax  = (float)MOTOR_SPEED_RPM * 6.0f;   /* RPM -> deg/s */
    float k = (1.0f / accel) + (1.0f / decel);
    float accel_decel_dist = 0.5f * vmax * vmax * k;
    float total_time;

    if (max_delta <= accel_decel_dist) {
        total_time = sqrtf(2.0f * max_delta * k);
    } else {
        total_time = (max_delta / vmax) + (0.5f * vmax * k);
    }

    for (uint8_t i = 0; i < 4; i++) {
        float d = fabsf(target_angles[i] - s_motor_angle[i]);
        if (d < 0.1f) {
            speeds[i] = 1.0f;
            continue;
        }

        float disc = total_time * total_time - 2.0f * d * k;
        if (disc < 0.0f) disc = 0.0f;

        float v = (total_time - sqrtf(disc)) / k;
        float rpm = v / 6.0f;
        if (rpm < 1.0f) rpm = 1.0f;
        if (rpm > (float)MOTOR_SPEED_RPM) rpm = (float)MOTOR_SPEED_RPM;
        speeds[i] = rpm;
    }
}

static void remember_target_angles(const float target_angles[4])
{
    for (uint8_t i = 0; i < 4; i++) {
        s_motor_angle[i] = target_angles[i];
    }
    s_angle_known = 1;
}

/* ============================================================
 * 辅助: 发送平台移动到激光目标 (lx, ly)
 * ============================================================ */
static void move_to_laser(float lx, float ly)
{
    enable_for_motion();

    float cx, cy;
    Kinematics_LaserToCam(lx, ly, &cx, &cy);

    float angles[4];
    Kinematics_CamToAngles(cx, cy, angles);

    float speeds[4];
    calc_sync_speeds(angles, speeds);

    Motor_MultiPositionCmdSpeeds(angles, speeds,
                                 MOTOR_ACCEL_RPMS,
                                 MOTOR_DECEL_RPMS);

    s_cam_x   = cx;
    s_cam_y   = cy;
    s_laser_x = lx;
    s_laser_y = ly;
    s_moving  = 1;
    s_move_start = HAL_GetTick();
    remember_target_angles(angles);
}

/** 等待电机运动完成 (超时判定到位) */
static uint8_t wait_done(void)
{
    return (HAL_GetTick() - s_move_start >= MOTOR_MOVE_TIMEOUT_MS);
}

/* ============================================================
 * Task_Init
 * ============================================================ */
void Task_Init(void)
{
    generate_snake();
    s_state         = TASK_IDLE;
    s_moving        = 0;
    s_motor_enabled = 0;
    s_laser_x       = 0.0f;
    s_laser_y       = 0.0f;
    Kinematics_LaserToCam(0.0f, 0.0f, &s_cam_x, &s_cam_y);
    s_angle_known   = 0;
    for (uint8_t i = 0; i < 4; i++) s_motor_angle[i] = 0.0f;
}

/* ============================================================
 * 对外触发接口
 * ============================================================ */
void Task_StartHome(void)
{
    /* 注意: 此函数从 USART2 中断上下文调用, 不能调用含 HAL_Delay 的函数。
     * 真正的使能和同步回 0 在 Task_Tick 主循环上下文执行。 */
    s_state = TASK_HOME;
    s_moving = 0;
}

void Task_StartAreaPatrol(void)
{
    s_state    = TASK_AREA_PATROL;
    s_wp_idx   = 0;
    s_wp_total = 4;
    s_moving   = 0;
}

void Task_StartSeqPatrol(const uint8_t seq[5])
{
    memcpy(s_seq, seq, 5);
    s_state    = TASK_SEQ_PATROL;
    s_wp_idx   = 0;
    s_wp_total = 5;
    s_moving   = 0;
}

void Task_StartAutoPatrol(void)
{
    s_state    = TASK_AUTO_PATROL;
    s_wp_idx   = 0;
    s_wp_total = s_snake_cnt;
    s_fire_halt = 0;
    s_moving   = 0;
}

void Task_StartCalibrate(void)
{
    s_state = TASK_CALIBRATE;
    s_moving = 0;
}

void Task_EStop(void)
{
    s_estop_pending = 1;
    s_state         = TASK_E_STOP;
    s_moving        = 0;
}

TaskState_t Task_GetState(void) { return s_state; }
float Task_GetLaserX(void) { return s_laser_x; }
float Task_GetLaserY(void) { return s_laser_y; }

/* ============================================================
 * Task_Tick (主循环调用)
 * ============================================================ */
void Task_Tick(void)
{
    /* 处理异步蜂鸣 */
    Buzzer_Tick();

    if (s_estop_pending) {
        Motor_StopAll();
        Motor_DisableAll();
        s_motor_enabled = 0;
        s_angle_known = 0;
        s_estop_pending = 0;
        s_state         = TASK_E_STOP;
        s_moving        = 0;
        return;
    }

    /* NRF 轮询 */
    if (NrfApp_Poll()) {
        /* 收到火源后蜂鸣；自动巡逻时同时停止并记录火源坐标 */
        if (NrfApp_IsFire()) {
            float fire_x;
            float fire_y;

            if (s_state == TASK_AUTO_PATROL) {
                Motor_StopAll();
                s_fire_halt = 1;
            }
            Buzzer_BeepAsync(BUZZER_FIRE_BEEPS);
            for (uint8_t i = 0; i < NrfApp_GetFireCount(); i++) {
                if (NrfApp_GetFireCm(i, &fire_x, &fire_y)) {
                    Screen_RecordFire(fire_x, fire_y);
                }
            }
        }
    }

    switch (s_state) {

    /* ---- IDLE / E_STOP ---- */
    case TASK_IDLE:
    case TASK_E_STOP:
        break;

    /* ---- CALIBRATE ---- */
    case TASK_CALIBRATE:
        /* 只允许在激光点确实位于中心圆时执行。执行后保持松轴。 */
        Motor_ZeroAllPositions();
        Kinematics_Init();
        Motor_DisableAll();
        s_motor_enabled = 0;
        s_angle_known = 1;
        for (uint8_t i = 0; i < 4; i++) s_motor_angle[i] = 0.0f;
        s_laser_x = 0.0f;
        s_laser_y = 0.0f;
        Kinematics_LaserToCam(0.0f, 0.0f, &s_cam_x, &s_cam_y);
        s_state   = TASK_IDLE;
        break;

    /* ---- HOME ---- */
    case TASK_HOME:
        if (!s_moving) {
            float zero_angles[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            float speeds[4];
            calc_sync_speeds(zero_angles, speeds);
            enable_for_motion();
            Motor_MoveAllSyncSpeeds(zero_angles, speeds,
                                     MOTOR_ACCEL_RPMS,
                                     MOTOR_DECEL_RPMS);
            Kinematics_LaserToCam(0.0f, 0.0f, &s_cam_x, &s_cam_y);
            s_laser_x = CIRCLE5_X;
            s_laser_y = CIRCLE5_Y;
            s_moving  = 1;
            s_move_start = HAL_GetTick();
            remember_target_angles(zero_angles);
        } else if (wait_done()) {
            s_moving = 0;
            s_state  = TASK_IDLE;
        }
        break;

    /* ---- AREA_PATROL ---- */
    case TASK_AREA_PATROL:
        if (!s_moving) {
            if (s_wp_idx >= 4) {
                s_state = TASK_IDLE;
                break;
            }
            move_to_laser(k_circles[s_wp_idx][0],
                          k_circles[s_wp_idx][1]);
        } else if (wait_done()) {
            s_moving = 0;
            HAL_Delay(PATROL_DWELL_MS);
            s_wp_idx++;
        }
        break;

    /* ---- SEQ_PATROL ---- */
    case TASK_SEQ_PATROL:
        if (!s_moving) {
            if (s_wp_idx >= 5) {
                s_state = TASK_IDLE;
                break;
            }
            uint8_t ci = s_seq[s_wp_idx] - 1;  /* 1-based → 0-based */
            if (ci >= 5) ci = 0;
            move_to_laser(k_circles[ci][0], k_circles[ci][1]);
        } else if (wait_done()) {
            s_moving = 0;
            HAL_Delay(PATROL_DWELL_MS);
            s_wp_idx++;
        }
        break;

    /* ---- AUTO_PATROL ---- */
    case TASK_AUTO_PATROL:
        if (s_fire_halt) {
            /* 检测到火源后, 等待蜂鸣完成再继续 */
            s_fire_halt = 0;
            s_moving    = 0;
            /* 继续下一路点 */
            s_wp_idx++;
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
            }
            break;
        }
        if (!s_moving) {
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
                break;
            }
            move_to_laser(s_snake[s_wp_idx][0],
                          s_snake[s_wp_idx][1]);
        } else if (wait_done()) {
            s_moving = 0;
            HAL_Delay(PATROL_DWELL_MS);
            s_wp_idx++;
        }
        break;
    }
}
