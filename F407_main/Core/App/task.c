/**
 * @file    task.c
 * @brief   任务状态机实现
 *
 * 运动执行流程:
 *  1. 根据目标激光坐标计算平台中心坐标
 *  2. 调用 Kinematics_CamToAngles 得到四电机绝对角度
 *  3. 启动前先使能电机, 再调用 X 固件梯形曲线位置命令
 *  4. 停稳后采集视觉坐标, 若误差过大则小步补偿
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

#define FIRE_DEDUP_DIST_CM 3.0f
#define FIRE_RECORD_MAX    2U

typedef enum {
    CL_PHASE_SETTLE,
    CL_PHASE_SAMPLE,
} ClosedLoopPhase_t;

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
static uint8_t     s_fire_halt  = 0;       /* 检测到火源后暂停, 等蜂鸣结束再继续 */
static uint8_t     s_fire_latched = 0;     /* 火源持续可见时只触发一次蜂鸣 */
static uint8_t     s_fire_record_cnt = 0;  /* 已记录的不同火源数量 */
static float       s_fire_record_x[FIRE_RECORD_MAX];
static float       s_fire_record_y[FIRE_RECORD_MAX];
static uint8_t     s_cl_active = 0;
static ClosedLoopPhase_t s_cl_phase = CL_PHASE_SETTLE;
static float       s_cl_target_x = 0.0f;
static float       s_cl_target_y = 0.0f;
static uint8_t     s_cl_corrections = 0;
static uint32_t    s_cl_phase_start = 0;
static uint8_t     s_cl_sample_count = 0;
static float       s_cl_sample_x[CLOSED_LOOP_SAMPLE_COUNT];
static float       s_cl_sample_y[CLOSED_LOOP_SAMPLE_COUNT];
static uint8_t     s_nrf_new_frame = 0;
static uint8_t     s_motor_enabled = 0;    /* 上电清零后默认松轴 */
static uint8_t     s_angle_known = 0;      /* 手拉后未知, 回中心后重新可信 */
static volatile uint8_t s_estop_pending = 0;

static void move_to_laser(float lx, float ly);
static uint8_t wait_done(void);

static void reset_closed_loop(void)
{
    s_cl_active = 0;
    s_cl_sample_count = 0;
    s_cl_corrections = 0;
}

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

static uint8_t record_fire_if_new(float x, float y)
{
    for (uint8_t i = 0; i < s_fire_record_cnt; i++) {
        float dx = x - s_fire_record_x[i];
        float dy = y - s_fire_record_y[i];
        if (sqrtf(dx * dx + dy * dy) <= FIRE_DEDUP_DIST_CM) {
            return 0;
        }
    }

    if (s_fire_record_cnt >= FIRE_RECORD_MAX) {
        return 0;
    }

    s_fire_record_x[s_fire_record_cnt] = x;
    s_fire_record_y[s_fire_record_cnt] = y;
    s_fire_record_cnt++;
    Screen_RecordFire(x, y);
    return 1;
}

static float median_of_samples(float *v, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; j++) {
            if (v[j] < v[i]) {
                float t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
        }
    }
    return v[count / 2U];
}

static void closed_loop_begin_settle(void)
{
    s_cl_phase = CL_PHASE_SETTLE;
    s_cl_phase_start = HAL_GetTick();
    s_cl_sample_count = 0;
}

static void closed_loop_start(float target_x, float target_y)
{
    s_cl_active = 1;
    s_cl_target_x = target_x;
    s_cl_target_y = target_y;
    s_cl_corrections = 0;
    closed_loop_begin_settle();
}

static uint8_t closed_loop_tick(void)
{
#if CLOSED_LOOP_ENABLE == 0U
    s_cl_active = 0;
    return 1;
#else
    if (!s_cl_active) {
        return 1;
    }

    if (s_moving) {
        if (wait_done()) {
            s_moving = 0;
            closed_loop_begin_settle();
        }
        return 0;
    }

    uint32_t now = HAL_GetTick();
    if (s_cl_phase == CL_PHASE_SETTLE) {
        if (now - s_cl_phase_start < CLOSED_LOOP_SETTLE_MS) {
            return 0;
        }
        s_cl_phase = CL_PHASE_SAMPLE;
        s_cl_phase_start = now;
        s_cl_sample_count = 0;
    }

    if (s_nrf_new_frame &&
        NrfApp_HasValidPosition() &&
        !NrfApp_IsEstimatedPosition()) {
        if (s_cl_sample_count < CLOSED_LOOP_SAMPLE_COUNT) {
            s_cl_sample_x[s_cl_sample_count] = NrfApp_GetPlatformXCm();
            s_cl_sample_y[s_cl_sample_count] = NrfApp_GetPlatformYCm();
            s_cl_sample_count++;
        }
    }

    if (s_cl_sample_count < CLOSED_LOOP_SAMPLE_COUNT) {
        if (now - s_cl_phase_start >= CLOSED_LOOP_SAMPLE_TIMEOUT_MS) {
            s_cl_active = 0;
            return 1;
        }
        return 0;
    }

    float vx[CLOSED_LOOP_SAMPLE_COUNT];
    float vy[CLOSED_LOOP_SAMPLE_COUNT];
    for (uint8_t i = 0; i < CLOSED_LOOP_SAMPLE_COUNT; i++) {
        vx[i] = s_cl_sample_x[i];
        vy[i] = s_cl_sample_y[i];
    }

    float actual_x = median_of_samples(vx, CLOSED_LOOP_SAMPLE_COUNT);
    float actual_y = median_of_samples(vy, CLOSED_LOOP_SAMPLE_COUNT);
    float err_x = s_cl_target_x - actual_x;
    float err_y = s_cl_target_y - actual_y;
    float err = sqrtf(err_x * err_x + err_y * err_y);

    if (err <= POSITION_TOL_CM ||
        s_cl_corrections >= CLOSED_LOOP_MAX_CORRECTIONS) {
        s_cl_active = 0;
        return 1;
    }

    float step_x = err_x * CLOSED_LOOP_GAIN;
    float step_y = err_y * CLOSED_LOOP_GAIN;
    float step = sqrtf(step_x * step_x + step_y * step_y);
    if (step > CLOSED_LOOP_MAX_STEP_CM && step > 0.001f) {
        float k = CLOSED_LOOP_MAX_STEP_CM / step;
        step_x *= k;
        step_y *= k;
    }

    s_cl_corrections++;
    move_to_laser(s_laser_x + step_x, s_laser_y + step_y);
    return 0;
#endif
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
    reset_closed_loop();
    s_state         = TASK_IDLE;
    s_moving        = 0;
    s_motor_enabled = 0;
    s_laser_x       = 0.0f;
    s_laser_y       = 0.0f;
    Kinematics_LaserToCam(0.0f, 0.0f, &s_cam_x, &s_cam_y);
    s_angle_known   = 0;
    s_fire_record_cnt = 0;
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
    reset_closed_loop();
}

void Task_StartAreaPatrol(void)
{
    s_state    = TASK_AREA_PATROL;
    s_wp_idx   = 0;
    s_wp_total = 4;
    s_moving   = 0;
    reset_closed_loop();
}

void Task_StartSeqPatrol(const uint8_t seq[5])
{
    memcpy(s_seq, seq, 5);
    s_state    = TASK_SEQ_PATROL;
    s_wp_idx   = 0;
    s_wp_total = 5;
    s_moving   = 0;
    reset_closed_loop();
}

void Task_StartAutoPatrol(void)
{
    s_state    = TASK_AUTO_PATROL;
    s_wp_idx   = 0;
    s_wp_total = s_snake_cnt;
    s_fire_halt = 0;
    s_moving   = 0;
    reset_closed_loop();
}

void Task_StartCalibrate(void)
{
    s_state = TASK_CALIBRATE;
    s_moving = 0;
    reset_closed_loop();
}

void Task_EStop(void)
{
    s_estop_pending = 1;
    s_state         = TASK_E_STOP;
    s_moving        = 0;
    reset_closed_loop();
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
        reset_closed_loop();
        return;
    }

    /* NRF 轮询 */
    s_nrf_new_frame = NrfApp_Poll();
    if (s_nrf_new_frame) {
        uint8_t fire_seen = NrfApp_IsFire();
        if (!fire_seen) {
            s_fire_latched = 0;
        }

        /* 收到火源后蜂鸣；记录检测瞬间的激光点坐标 */
        if (fire_seen && s_fire_latched == 0U && s_fire_halt == 0U) {
            s_fire_latched = 1;
            if (s_state == TASK_AUTO_PATROL) {
                Motor_StopAll();
                s_moving = 0;
                reset_closed_loop();
                s_fire_halt = 1;
            }
            Buzzer_BeepAsync(BUZZER_FIRE_BEEPS);
            if (NrfApp_HasValidPosition()) {
                record_fire_if_new(NrfApp_GetPlatformXCm(), NrfApp_GetPlatformYCm());
            } else {
                record_fire_if_new(s_laser_x, s_laser_y);
            }
        }
    }

    if (s_fire_halt) {
        if (Buzzer_IsBusy()) {
            return;
        }

        s_fire_halt = 0;
        if (s_state == TASK_AUTO_PATROL) {
            s_wp_idx++;
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
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
        reset_closed_loop();
        s_state   = TASK_IDLE;
        break;

    /* ---- HOME ---- */
    case TASK_HOME:
        if (s_cl_active) {
            if (closed_loop_tick()) {
                s_state = TASK_IDLE;
            }
        } else if (!s_moving) {
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
            closed_loop_start(CIRCLE5_X, CIRCLE5_Y);
        }
        break;

    /* ---- AREA_PATROL ---- */
    case TASK_AREA_PATROL:
        if (s_cl_active) {
            if (closed_loop_tick()) {
                HAL_Delay(PATROL_DWELL_MS);
                s_wp_idx++;
            }
        } else if (!s_moving) {
            if (s_wp_idx >= 4) {
                s_state = TASK_IDLE;
                break;
            }
            move_to_laser(k_circles[s_wp_idx][0],
                          k_circles[s_wp_idx][1]);
        } else if (wait_done()) {
            s_moving = 0;
            closed_loop_start(k_circles[s_wp_idx][0],
                              k_circles[s_wp_idx][1]);
        }
        break;

    /* ---- SEQ_PATROL ---- */
    case TASK_SEQ_PATROL:
        if (s_cl_active) {
            if (closed_loop_tick()) {
                HAL_Delay(PATROL_DWELL_MS);
                s_wp_idx++;
            }
        } else if (!s_moving) {
            if (s_wp_idx >= 5) {
                s_state = TASK_IDLE;
                break;
            }
            uint8_t ci = s_seq[s_wp_idx] - 1;  /* 1-based → 0-based */
            if (ci >= 5) ci = 0;
            move_to_laser(k_circles[ci][0], k_circles[ci][1]);
        } else if (wait_done()) {
            uint8_t ci = s_seq[s_wp_idx] - 1;
            if (ci >= 5) ci = 0;
            s_moving = 0;
            closed_loop_start(k_circles[ci][0], k_circles[ci][1]);
        }
        break;

    /* ---- AUTO_PATROL ---- */
    case TASK_AUTO_PATROL:
        if (s_cl_active) {
            if (closed_loop_tick()) {
                HAL_Delay(PATROL_DWELL_MS);
                s_wp_idx++;
            }
        } else if (!s_moving) {
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
                break;
            }
            move_to_laser(s_snake[s_wp_idx][0],
                          s_snake[s_wp_idx][1]);
        } else if (wait_done()) {
            s_moving = 0;
            closed_loop_start(s_snake[s_wp_idx][0],
                              s_snake[s_wp_idx][1]);
        }
        break;
    }
}
