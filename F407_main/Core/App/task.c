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
#define MOVE_MIN_WAIT_MS       600U
#define MOVE_SETTLE_MARGIN_MS  500U
#define MOVE_UNKNOWN_WAIT_MS   8000U

typedef enum {
    CL_PHASE_SETTLE,
    CL_PHASE_SAMPLE,
} ClosedLoopPhase_t;

/* ============================================================
 * 橙色圆激光目标坐标表 (cm, index 1~5 对应圆 1~5)
 * ============================================================ */
static const float k_circles[5][2] = {
    {CIRCLE1_X, CIRCLE1_Y},   /* [0]: 新圆1 左上 */
    {CIRCLE2_X, CIRCLE2_Y},   /* [1]: 新圆2 右上 */
    {CIRCLE3_X, CIRCLE3_Y},   /* [2]: 新圆3 中心 */
    {CIRCLE4_X, CIRCLE4_Y},   /* [3]: 新圆4 左下 */
    {CIRCLE5_X, CIRCLE5_Y},   /* [4]: 新圆5 右下 */
};

/* 区域巡逻路线: 圆1 -> 圆2 -> 圆5 -> 圆4 -> 圆1 */
static const uint8_t k_area_patrol_route[5] = {0, 1, 4, 3, 0};

/* 区域巡逻姿态补偿表: 行对应圆1~圆5目标位置。
 * 正值表示该电机额外收线, 用于抬高长绳侧低下去的角。
 * 规则:
 *   y>0 时下侧 ID1/ID2 更长, y<0 时上侧 ID3/ID4 更长。
 *   x>0 时左侧 ID1/ID4 更长, x<0 时右侧 ID2/ID3 更长。
 * 角点处两个方向叠加, 最远的对角电机补偿为 2 倍。
 */
static const float k_area_tilt_comp[5][4] = {
    {AREA_TILT_COMP_DEG,       2.0f * AREA_TILT_COMP_DEG, AREA_TILT_COMP_DEG,       0.0f},
    {2.0f * AREA_TILT_COMP_DEG, AREA_TILT_COMP_DEG,       0.0f,                     AREA_TILT_COMP_DEG},
    {0.0f,                     0.0f,                     0.0f,                     0.0f},
    {0.0f,                     AREA_TILT_COMP_DEG,       2.0f * AREA_TILT_COMP_DEG, AREA_TILT_COMP_DEG},
    {AREA_TILT_COMP_DEG,       0.0f,                     AREA_TILT_COMP_DEG,       2.0f * AREA_TILT_COMP_DEG},
};

/* 任务2实测角点角度表: 每行对应圆1~圆5, 列为 ZDT ID1~ID4 绝对角度(°) */
static const float k_circle_angles[5][4] = {
    { -236.8f,  961.5f, -189.5f, -951.2f },  /* 圆1 */
    { -934.0f,  362.2f,  924.6f,  227.9f },  /* 圆2 */
    {    0.0f,    0.0f,    0.0f,    0.0f },  /* 圆3: 中心 */
    {  869.4f,  364.7f, -936.0f,  250.5f },  /* 圆4 */
    { -400.5f, -830.0f, -365.8f,  875.0f },  /* 圆5 */
};

typedef struct {
    float x;
    float y;
    float angles[4];
    int8_t circle_idx;  /* 0~4=圆1~圆5, -1=非圆点边界点 */
} AutoPatrolPoint_t;

/* 自动巡逻实测边界端点: 5 条横向扫描线, 行内用角度插值运动 */
static const AutoPatrolPoint_t k_auto_patrol_points[] = {
    {-20.0f,  20.0f, {-236.8f,  961.5f, -189.5f, -951.2f}, 0},   /* 圆1 */
    { 20.0f,  20.0f, {-934.0f,  362.2f,  924.6f,  227.9f}, 1},   /* 圆2 */
    { 20.0f,  10.0f, {-712.1f,   -0.7f,  578.7f,  356.8f}, -1},
    {-20.0f,  10.0f, {  83.6f,  743.0f, -325.6f, -651.4f}, -1},
    {-20.0f,   0.0f, { 357.2f,  543.0f, -499.9f, -380.0f}, -1},
    { 20.0f,   0.0f, {-559.7f, -331.7f,  292.3f,  530.9f}, -1},
    { 20.0f, -10.0f, {
        -463.4f - AUTO_STRESS_RELIEF_DEG * MOTOR1_DIR_SIGN,
        -703.3f - AUTO_STRESS_RELIEF_DEG * MOTOR2_DIR_SIGN,
         -35.5f - AUTO_STRESS_RELIEF_DEG * MOTOR3_DIR_SIGN,
         706.6f - AUTO_STRESS_RELIEF_DEG * MOTOR4_DIR_SIGN}, -1},
    {-20.0f, -10.0f, {
         660.7f - AUTO_STRESS_RELIEF_DEG * MOTOR1_DIR_SIGN,
         421.9f - AUTO_STRESS_RELIEF_DEG * MOTOR2_DIR_SIGN,
        -699.0f - AUTO_STRESS_RELIEF_DEG * MOTOR3_DIR_SIGN,
         -17.8f - AUTO_STRESS_RELIEF_DEG * MOTOR4_DIR_SIGN}, -1},
    {-20.0f, -20.0f, {
         869.4f - AUTO_STRESS_RELIEF_DEG * MOTOR1_DIR_SIGN,
         364.7f - AUTO_STRESS_RELIEF_DEG * MOTOR2_DIR_SIGN,
        -936.0f - AUTO_STRESS_RELIEF_DEG * MOTOR3_DIR_SIGN,
         250.5f - AUTO_STRESS_RELIEF_DEG * MOTOR4_DIR_SIGN}, 3},   /* 圆4 */
    { 20.0f, -20.0f, {-400.5f, -830.0f, -365.8f,  875.0f}, 4},   /* 圆5 */
};

#define AUTO_PATROL_POINT_COUNT ((uint8_t)(sizeof(k_auto_patrol_points) / sizeof(k_auto_patrol_points[0])))

/* ============================================================
 * 蛇形巡逻路点生成
 * 从左下角开始, 10cm 间隔遍历, 结束于左上角
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
static uint32_t    s_move_wait_ms = MOVE_UNKNOWN_WAIT_MS;
static float       s_cam_x      = 0.0f;   /* 当前摄像头坐标 */
static float       s_cam_y      = 0.0f;
static float       s_laser_x    = 0.0f;   /* 当前激光坐标 */
static float       s_laser_y    = 0.0f;
static float       s_motor_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t     s_moving     = 0;       /* 1=等待电机运动完成 */
static uint8_t     s_path_active = 0;      /* 1=正在执行分段路径 */
static uint16_t    s_path_total = 0;
static uint16_t    s_path_next = 0;
static float       s_path_start_x = 0.0f;
static float       s_path_start_y = 0.0f;
static float       s_path_target_x = 0.0f;
static float       s_path_target_y = 0.0f;
static uint8_t     s_angle_path_active = 0;
static uint16_t    s_angle_path_total = 0;
static uint16_t    s_angle_path_next = 0;
static uint8_t     s_angle_path_pending_active = 0;
static uint8_t     s_angle_path_pending_circle = 0;
static uint8_t     s_angle_path_pending_has_comp = 0;
static float       s_angle_path_pending_comp[4];
static float       s_angle_path_start[4];
static float       s_angle_path_target[4];
static uint8_t     s_angle_path_slow_release[4];
static uint8_t     s_angle_path_light_slow_release[4];
static uint8_t     s_angle_path_light_fast_takeup[4];
static uint8_t     s_angle_path_fast_takeup[4];
static uint8_t     s_angle_path_slow_takeup[4];
static float       s_angle_path_start_x = 0.0f;
static float       s_angle_path_start_y = 0.0f;
static float       s_angle_path_target_x = 0.0f;
static float       s_angle_path_target_y = 0.0f;
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
static volatile uint8_t s_enable_pending = 0;
static volatile uint8_t s_disable_pending = 0;

static void move_to_laser(float lx, float ly);
static void move_to_circle_angles(uint8_t circle_idx, const float comp_deg[4], int8_t from_circle_idx);
static void move_to_auto_point(uint8_t point_idx);
static uint8_t wait_done(void);
static void sync_motor_angles_from_driver(void);
static uint32_t estimate_motion_wait_ms(const float target_angles[4],
                                        const float speeds_rpm[4],
                                        uint32_t min_wait_ms,
                                        uint32_t margin_ms);

static void reset_closed_loop(void)
{
    s_cl_active = 0;
    s_cl_sample_count = 0;
    s_cl_corrections = 0;
}

static void reset_motion_path(void)
{
    s_path_active = 0;
    s_path_total = 0;
    s_path_next = 0;
    s_angle_path_active = 0;
    s_angle_path_total = 0;
    s_angle_path_next = 0;
    s_angle_path_pending_active = 0;
    s_angle_path_pending_circle = 0;
    s_angle_path_pending_has_comp = 0;
    memset(s_angle_path_slow_release, 0, sizeof(s_angle_path_slow_release));
    memset(s_angle_path_light_slow_release, 0, sizeof(s_angle_path_light_slow_release));
    memset(s_angle_path_light_fast_takeup, 0, sizeof(s_angle_path_light_fast_takeup));
    memset(s_angle_path_fast_takeup, 0, sizeof(s_angle_path_fast_takeup));
    memset(s_angle_path_slow_takeup, 0, sizeof(s_angle_path_slow_takeup));
}

static float motor_dir_sign_by_index(uint8_t i)
{
    switch (i) {
    case 0: return MOTOR1_DIR_SIGN;
    case 1: return MOTOR2_DIR_SIGN;
    case 2: return MOTOR3_DIR_SIGN;
    case 3: return MOTOR4_DIR_SIGN;
    default: return 1.0f;
    }
}

static uint8_t edge_long_side_motors(uint8_t from, uint8_t to, uint8_t long_side[4])
{
    memset(long_side, 0, 4);

    if ((from == 0U && to == 1U) || (from == 1U && to == 0U)) {
        long_side[0] = 1U; long_side[1] = 1U;  /* 上边: 下侧 ID1/ID2 */
    } else if ((from == 1U && to == 4U) || (from == 4U && to == 1U)) {
        long_side[0] = 1U; long_side[3] = 1U;  /* 右边: 左侧 ID1/ID4 */
    } else if ((from == 4U && to == 3U) || (from == 3U && to == 4U)) {
        long_side[2] = 1U; long_side[3] = 1U;  /* 下边: 上侧 ID3/ID4 */
    } else if ((from == 3U && to == 0U) || (from == 0U && to == 3U)) {
        long_side[1] = 1U; long_side[2] = 1U;  /* 左边: 右侧 ID2/ID3 */
    } else {
        return 0U;
    }

    return 1U;
}

static int8_t directed_fast_takeup_motor(uint8_t from, uint8_t to)
{
    if (from == 0U && (to == 1U || to == 3U)) {
        return 1;  /* 圆1->圆2/圆4: 起点圆1对角 ID2 */
    }
    if (from == 1U && (to == 4U || to == 0U)) {
        return 0;  /* 圆2->圆5/圆1: 起点圆2对角 ID1 */
    }
    if (from == 4U && (to == 3U || to == 1U)) {
        return 3;  /* 圆5->圆4/圆2: 起点圆5对角 ID4 */
    }
    if (from == 3U && (to == 0U || to == 4U)) {
        return 2;  /* 圆4->圆1/圆5: 起点圆4对角 ID3 */
    }
    return -1;
}

static uint8_t side_curve_motors(uint8_t from, uint8_t to, uint8_t side[4])
{
    memset(side, 0, 4);

    uint8_t corner = 0xFFU;
    if (from == 2U && to != 2U) {
        corner = to;       /* 圆3中心 -> 四个角点 */
    } else if (to == 2U && from != 2U) {
        corner = from;     /* 四个角点 -> 圆3中心 */
    } else {
        return 0U;
    }

    switch (corner) {
    case 0U:  /* 圆1左上: 侧边 ID1/ID3 */
    case 4U:  /* 圆5右下: 侧边 ID1/ID3 */
        side[0] = 1U;
        side[2] = 1U;
        return 1U;
    case 1U:  /* 圆2右上: 侧边 ID2/ID4 */
    case 3U:  /* 圆4左下: 侧边 ID2/ID4 */
        side[1] = 1U;
        side[3] = 1U;
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t should_split_via_center(uint8_t from, uint8_t to)
{
    return ((from == 0U && to == 4U) ||
            (from == 4U && to == 0U) ||
            (from == 1U && to == 3U) ||
            (from == 3U && to == 1U)) ? 1U : 0U;
}

static void setup_edge_slow_release(int8_t from_circle_idx, uint8_t to_circle_idx)
{
    memset(s_angle_path_slow_release, 0, sizeof(s_angle_path_slow_release));
    memset(s_angle_path_light_slow_release, 0, sizeof(s_angle_path_light_slow_release));
    memset(s_angle_path_light_fast_takeup, 0, sizeof(s_angle_path_light_fast_takeup));
    memset(s_angle_path_fast_takeup, 0, sizeof(s_angle_path_fast_takeup));
    memset(s_angle_path_slow_takeup, 0, sizeof(s_angle_path_slow_takeup));

    if (from_circle_idx < 0) {
        return;
    }

#if AREA_EDGE_SLOW_RELEASE_ENABLE
    uint8_t long_side[4];
    if (edge_long_side_motors((uint8_t)from_circle_idx, to_circle_idx, long_side)) {
        for (uint8_t i = 0; i < 4U; i++) {
            float delta = s_angle_path_target[i] - s_angle_path_start[i];
            float takeup_delta = delta * motor_dir_sign_by_index(i);
            if (long_side[i] && takeup_delta < -0.1f) {
                s_angle_path_slow_release[i] = 1U;
            }
        }
    }
#else
    (void)from_circle_idx;
    (void)to_circle_idx;
#endif

#if AREA_EDGE_FAST_TAKEUP_ENABLE
    int8_t fast_motor = directed_fast_takeup_motor((uint8_t)from_circle_idx, to_circle_idx);
    if (fast_motor >= 0) {
        float delta = s_angle_path_target[fast_motor] - s_angle_path_start[fast_motor];
        float takeup_delta = delta * motor_dir_sign_by_index((uint8_t)fast_motor);
        if (takeup_delta > 0.1f) {
            s_angle_path_fast_takeup[fast_motor] = 1U;
        }
    }
#else
    (void)from_circle_idx;
    (void)to_circle_idx;
#endif

#if AREA_SIDE_CURVE_ENABLE
    uint8_t side[4];
    if (side_curve_motors((uint8_t)from_circle_idx, to_circle_idx, side)) {
        for (uint8_t i = 0; i < 4U; i++) {
            float delta = s_angle_path_target[i] - s_angle_path_start[i];
            float takeup_delta = delta * motor_dir_sign_by_index(i);
            if (side[i] && takeup_delta < -0.1f) {
                s_angle_path_slow_release[i] = 1U;
            } else if (side[i] && takeup_delta > 0.1f) {
                s_angle_path_fast_takeup[i] = 1U;
            }
        }
    }
#endif
}

static void mark_slow_takeup_if_needed(uint8_t motor_idx)
{
    float delta = s_angle_path_target[motor_idx] - s_angle_path_start[motor_idx];
    float takeup_delta = delta * motor_dir_sign_by_index(motor_idx);
    if (takeup_delta > 0.1f) {
        s_angle_path_slow_takeup[motor_idx] = 1U;
    }
}

static void mark_slow_release_if_needed(uint8_t motor_idx)
{
    float delta = s_angle_path_target[motor_idx] - s_angle_path_start[motor_idx];
    float takeup_delta = delta * motor_dir_sign_by_index(motor_idx);
    if (takeup_delta < -0.1f) {
        s_angle_path_slow_release[motor_idx] = 1U;
    }
}

static void mark_fast_takeup_if_needed(uint8_t motor_idx)
{
    float delta = s_angle_path_target[motor_idx] - s_angle_path_start[motor_idx];
    float takeup_delta = delta * motor_dir_sign_by_index(motor_idx);
    if (takeup_delta > 0.1f) {
        s_angle_path_fast_takeup[motor_idx] = 1U;
    }
}

static void mark_light_slow_release_if_needed(uint8_t motor_idx)
{
    float delta = s_angle_path_target[motor_idx] - s_angle_path_start[motor_idx];
    float takeup_delta = delta * motor_dir_sign_by_index(motor_idx);
    if (takeup_delta < -0.1f) {
        s_angle_path_light_slow_release[motor_idx] = 1U;
    }
}

static void mark_light_fast_takeup_if_needed(uint8_t motor_idx)
{
    float delta = s_angle_path_target[motor_idx] - s_angle_path_start[motor_idx];
    float takeup_delta = delta * motor_dir_sign_by_index(motor_idx);
    if (takeup_delta > 0.1f) {
        s_angle_path_light_fast_takeup[motor_idx] = 1U;
    }
}

static void apply_auto_segment_release(uint8_t point_idx)
{
    if (point_idx == 5U) {  /* (-20,0)->(20,0): ID1/ID4 稍微松线 */
        s_angle_path_target[0] -= AUTO_SEGMENT_RELEASE_DEG * MOTOR1_DIR_SIGN;
        s_angle_path_target[3] -= AUTO_SEGMENT_RELEASE_DEG * MOTOR4_DIR_SIGN;
    }
}

static void setup_auto_risk_curves(uint8_t point_idx)
{
    if (point_idx == 0U) {
        return;
    }

    uint8_t prev = (uint8_t)(point_idx - 1U);

    /* 经过 (20,-10) 附近时, ID2/ID3 收线稍慢一点 */
    if (prev == 6U || point_idx == 6U) {
        mark_slow_takeup_if_needed(1U);
        mark_slow_takeup_if_needed(2U);
    }

    /* (-20,-10)->(-20,-20) 时, ID1/ID4 收线稍慢一点 */
    if (prev == 7U && point_idx == 8U) {
        mark_slow_takeup_if_needed(0U);
        mark_slow_takeup_if_needed(3U);
    }

    /* (20,10)->(-20,10): ID1/ID4 收线快一点, ID2/ID3 放线稍慢一点 */
    if (prev == 2U && point_idx == 3U) {
        mark_light_fast_takeup_if_needed(0U);
        mark_fast_takeup_if_needed(3U);
        mark_light_slow_release_if_needed(1U);
        mark_light_slow_release_if_needed(2U);
    }

    /* (20,-10)->(-20,-10): ID4 收线快一点, ID3 放线慢一点 */
    if (prev == 6U && point_idx == 7U) {
        mark_fast_takeup_if_needed(3U);
        mark_slow_release_if_needed(2U);
    }

    /* (-20,0)->(20,0): ID2/ID3 收线稍快一点点, ID1/ID4 放线稍慢一点点 */
    if (prev == 4U && point_idx == 5U) {
        mark_light_fast_takeup_if_needed(1U);
        mark_light_fast_takeup_if_needed(2U);
        mark_light_slow_release_if_needed(0U);
        mark_light_slow_release_if_needed(3U);
    }
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

static void sync_motor_angles_from_driver(void)
{
    float angles[4];

    if (Motor_ReadAllPositions(angles)) {
        for (uint8_t i = 0; i < 4; i++) {
            s_motor_angle[i] = angles[i];
        }
        s_angle_known = 1;
    }
}

static uint32_t clamp_move_wait_ms(uint32_t wait_ms, uint32_t min_wait_ms)
{
    if (wait_ms < min_wait_ms) {
        wait_ms = min_wait_ms;
    }
    return wait_ms;
}

static uint32_t estimate_motion_wait_ms(const float target_angles[4],
                                        const float speeds_rpm[4],
                                        uint32_t min_wait_ms,
                                        uint32_t margin_ms)
{
    float max_time_s = 0.0f;

    if (!s_angle_known) {
        return clamp_move_wait_ms(MOVE_UNKNOWN_WAIT_MS, min_wait_ms);
    }

    for (uint8_t i = 0; i < 4; i++) {
        float d = fabsf(target_angles[i] - s_motor_angle[i]);
        float vmax = fabsf(speeds_rpm[i]) * 6.0f;
        float time_s = 0.0f;

        if (d < 0.1f || vmax < 0.001f) {
            continue;
        }

        if (MOTOR_ACCEL_RPMS == 0U || MOTOR_DECEL_RPMS == 0U) {
            time_s = d / vmax;
        } else {
            float accel = (float)MOTOR_ACCEL_RPMS * 6.0f;
            float decel = (float)MOTOR_DECEL_RPMS * 6.0f;
            float t_acc = vmax / accel;
            float t_dec = vmax / decel;
            float d_acc = 0.5f * vmax * t_acc;
            float d_dec = 0.5f * vmax * t_dec;

            if (d <= (d_acc + d_dec)) {
                float v_peak = sqrtf((2.0f * d * accel * decel) / (accel + decel));
                time_s = (v_peak / accel) + (v_peak / decel);
            } else {
                time_s = t_acc + ((d - d_acc - d_dec) / vmax) + t_dec;
            }
        }

        if (time_s > max_time_s) {
            max_time_s = time_s;
        }
    }

    return clamp_move_wait_ms((uint32_t)(max_time_s * 1000.0f + 0.5f) +
                              margin_ms,
                              min_wait_ms);
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

static void command_laser_point(float lx, float ly,
                                uint32_t min_wait_ms,
                                uint32_t margin_ms)
{
    float cx, cy;
    Kinematics_LaserToCam(lx, ly, &cx, &cy);

    float angles[4];
    Kinematics_CamToAngles(cx, cy, angles);

    float speeds[4];
    calc_sync_speeds(angles, speeds);

    Motor_MultiPositionCmdSpeeds(angles, speeds,
                                 MOTOR_ACCEL_RPMS,
                                 MOTOR_DECEL_RPMS);

    s_move_wait_ms = estimate_motion_wait_ms(angles, speeds,
                                             min_wait_ms,
                                             margin_ms);
    s_cam_x   = cx;
    s_cam_y   = cy;
    s_laser_x = lx;
    s_laser_y = ly;
    s_moving  = 1;
    s_move_start = HAL_GetTick();
    remember_target_angles(angles);
}

static void command_angle_point(const float target_angles[4],
                                float laser_x,
                                float laser_y)
{
    float speeds[4];

    calc_sync_speeds(target_angles, speeds);
    Motor_MultiPositionCmdSpeeds(target_angles, speeds,
                                 MOTOR_ACCEL_RPMS,
                                 MOTOR_DECEL_RPMS);

    s_move_wait_ms = estimate_motion_wait_ms(target_angles, speeds,
                                             MOTION_SEGMENT_MIN_WAIT_MS,
                                             MOTION_SEGMENT_MARGIN_MS);
    s_laser_x = laser_x;
    s_laser_y = laser_y;
    Kinematics_LaserToCam(laser_x, laser_y, &s_cam_x, &s_cam_y);
    s_moving = 1;
    s_move_start = HAL_GetTick();
    remember_target_angles(target_angles);
}

static void command_angle_path_segment(uint16_t seg_idx)
{
    float t = 1.0f;
    float target[4];

    if (s_angle_path_total > 0U) {
        t = (float)seg_idx / (float)s_angle_path_total;
    }

    for (uint8_t i = 0; i < 4; i++) {
        float motor_t = t;
        if (s_angle_path_slow_release[i]) {
            motor_t = powf(t, AREA_EDGE_SLOW_RELEASE_POWER);
        } else if (s_angle_path_light_slow_release[i]) {
            motor_t = powf(t, AUTO_LIGHT_CURVE_POWER);
        } else if (s_angle_path_slow_takeup[i]) {
            motor_t = powf(t, AUTO_RISK_SLOW_TAKEUP_POWER);
        } else if (s_angle_path_fast_takeup[i]) {
            motor_t = 1.0f - powf(1.0f - t, AREA_EDGE_FAST_TAKEUP_POWER);
        } else if (s_angle_path_light_fast_takeup[i]) {
            motor_t = 1.0f - powf(1.0f - t, AUTO_LIGHT_CURVE_POWER);
        }
        target[i] = s_angle_path_start[i] +
                    (s_angle_path_target[i] - s_angle_path_start[i]) * motor_t;
    }

    float lx = s_angle_path_start_x +
               (s_angle_path_target_x - s_angle_path_start_x) * t;
    float ly = s_angle_path_start_y +
               (s_angle_path_target_y - s_angle_path_start_y) * t;

    command_angle_point(target, lx, ly);
    s_angle_path_next = (uint16_t)(seg_idx + 1U);
}

static void command_path_segment(uint16_t seg_idx)
{
    float t = 1.0f;
    if (s_path_total > 0U) {
        t = (float)seg_idx / (float)s_path_total;
    }

    float lx = s_path_start_x + (s_path_target_x - s_path_start_x) * t;
    float ly = s_path_start_y + (s_path_target_y - s_path_start_y) * t;

    command_laser_point(lx, ly,
                        MOTION_SEGMENT_MIN_WAIT_MS,
                        MOTION_SEGMENT_MARGIN_MS);
    s_path_next = (uint16_t)(seg_idx + 1U);
}

/* ============================================================
 * 辅助: 发送平台移动到激光目标 (lx, ly)
 * ============================================================ */
static void move_to_laser(float lx, float ly)
{
    enable_for_motion();
    sync_motor_angles_from_driver();

    float dx = lx - s_laser_x;
    float dy = ly - s_laser_y;
    float dist = sqrtf(dx * dx + dy * dy);

    s_path_start_x = s_laser_x;
    s_path_start_y = s_laser_y;
    s_path_target_x = lx;
    s_path_target_y = ly;
    s_path_total = 1U;
    if (MOTION_SEGMENT_STEP_CM > 0.01f && dist > MOTION_SEGMENT_STEP_CM) {
        s_path_total = (uint16_t)ceilf(dist / MOTION_SEGMENT_STEP_CM);
    }
    s_path_active = 1U;
    command_path_segment(1U);
}

static void move_to_circle_angles(uint8_t circle_idx, const float comp_deg[4], int8_t from_circle_idx)
{
    enable_for_motion();
    sync_motor_angles_from_driver();

    if (circle_idx >= 5U) {
        circle_idx = 0U;
    }

    if (from_circle_idx >= 0 &&
        should_split_via_center((uint8_t)from_circle_idx, circle_idx)) {
        s_angle_path_pending_active = 1U;
        s_angle_path_pending_circle = circle_idx;
        s_angle_path_pending_has_comp = (comp_deg != NULL) ? 1U : 0U;
        if (comp_deg != NULL) {
            memcpy(s_angle_path_pending_comp, comp_deg, sizeof(s_angle_path_pending_comp));
        }
        circle_idx = 2U;   /* 先从起点角点到圆3中心 */
        comp_deg = NULL;
    } else {
        s_angle_path_pending_active = 0U;
        s_angle_path_pending_has_comp = 0U;
    }

    for (uint8_t i = 0; i < 4; i++) {
        s_angle_path_start[i] = s_motor_angle[i];
        float comp = (comp_deg != NULL) ? comp_deg[i] : 0.0f;
        s_angle_path_target[i] = k_circle_angles[circle_idx][i] + comp;
    }
    s_angle_path_start_x = s_laser_x;
    s_angle_path_start_y = s_laser_y;
    s_angle_path_target_x = k_circles[circle_idx][0];
    s_angle_path_target_y = k_circles[circle_idx][1];
    setup_edge_slow_release(from_circle_idx, circle_idx);

    float max_delta = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float d = fabsf(s_angle_path_target[i] - s_angle_path_start[i]);
        if (d > max_delta) {
            max_delta = d;
        }
    }

    s_angle_path_total = 1U;
    if (ANGLE_SEGMENT_MAX_DEG > 0.1f && max_delta > ANGLE_SEGMENT_MAX_DEG) {
        s_angle_path_total = (uint16_t)ceilf(max_delta / ANGLE_SEGMENT_MAX_DEG);
    }
    s_angle_path_active = 1U;
    command_angle_path_segment(1U);
}

static void move_to_auto_point(uint8_t point_idx)
{
    enable_for_motion();
    sync_motor_angles_from_driver();

    if (point_idx >= AUTO_PATROL_POINT_COUNT) {
        point_idx = 0U;
    }

    const AutoPatrolPoint_t *p = &k_auto_patrol_points[point_idx];
    for (uint8_t i = 0; i < 4; i++) {
        s_angle_path_start[i] = s_motor_angle[i];
        s_angle_path_target[i] = p->angles[i];
    }
    apply_auto_segment_release(point_idx);
    s_angle_path_start_x = s_laser_x;
    s_angle_path_start_y = s_laser_y;
    s_angle_path_target_x = p->x;
    s_angle_path_target_y = p->y;

    memset(s_angle_path_slow_release, 0, sizeof(s_angle_path_slow_release));
    memset(s_angle_path_light_slow_release, 0, sizeof(s_angle_path_light_slow_release));
    memset(s_angle_path_light_fast_takeup, 0, sizeof(s_angle_path_light_fast_takeup));
    memset(s_angle_path_fast_takeup, 0, sizeof(s_angle_path_fast_takeup));
    memset(s_angle_path_slow_takeup, 0, sizeof(s_angle_path_slow_takeup));
    if (point_idx > 0U) {
        int8_t from_ci = k_auto_patrol_points[point_idx - 1U].circle_idx;
        if (from_ci >= 0 && p->circle_idx >= 0) {
            setup_edge_slow_release(from_ci, (uint8_t)p->circle_idx);
        }
    } else if (p->circle_idx >= 0) {
        setup_edge_slow_release(2, (uint8_t)p->circle_idx);
    }
    setup_auto_risk_curves(point_idx);

    float max_delta = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float d = fabsf(s_angle_path_target[i] - s_angle_path_start[i]);
        if (d > max_delta) {
            max_delta = d;
        }
    }

    s_angle_path_total = 1U;
    if (ANGLE_SEGMENT_MAX_DEG > 0.1f && max_delta > ANGLE_SEGMENT_MAX_DEG) {
        s_angle_path_total = (uint16_t)ceilf(max_delta / ANGLE_SEGMENT_MAX_DEG);
    }
    s_angle_path_active = 1U;
    command_angle_path_segment(1U);
}

/** 等待电机运动完成：不依赖 ZDT 到位返回, 按估算时间推进 */
static uint8_t wait_done(void)
{
    if (HAL_GetTick() - s_move_start < s_move_wait_ms) {
        return 0;
    }

    if (s_angle_path_active && s_angle_path_next <= s_angle_path_total) {
        command_angle_path_segment(s_angle_path_next);
        return 0;
    }

    if (s_path_active && s_path_next <= s_path_total) {
        command_path_segment(s_path_next);
        return 0;
    }

    if (s_angle_path_pending_active) {
        const float *pending_comp = s_angle_path_pending_has_comp ? s_angle_path_pending_comp : NULL;
        s_angle_path_pending_active = 0U;
        move_to_circle_angles(s_angle_path_pending_circle, pending_comp, 2);
        return 0;
    }

    reset_motion_path();
    return 1;
}

/* ============================================================
 * Task_Init
 * ============================================================ */
void Task_Init(void)
{
    generate_snake();
    reset_closed_loop();
    reset_motion_path();
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
    reset_motion_path();
    reset_closed_loop();
}

void Task_StartAreaPatrol(void)
{
    s_state    = TASK_AREA_PATROL;
    s_wp_idx   = 0;
    s_wp_total = (uint8_t)(sizeof(k_area_patrol_route) / sizeof(k_area_patrol_route[0]));
    s_moving   = 0;
    reset_motion_path();
    reset_closed_loop();
}

void Task_StartSeqPatrol(const uint8_t seq[5])
{
    memcpy(s_seq, seq, 5);
    s_state    = TASK_SEQ_PATROL;
    s_wp_idx   = 0;
    s_wp_total = 5;
    s_moving   = 0;
    reset_motion_path();
    reset_closed_loop();
}

void Task_StartAutoPatrol(void)
{
    s_state    = TASK_AUTO_PATROL;
    s_wp_idx   = 0;
    s_wp_total = AUTO_PATROL_POINT_COUNT;
    s_fire_halt = 0;
    s_moving   = 0;
    reset_motion_path();
    reset_closed_loop();
}

void Task_StartCalibrate(void)
{
    s_state = TASK_CALIBRATE;
    s_moving = 0;
    reset_motion_path();
    reset_closed_loop();
}

void Task_EnableMotors(void)
{
    s_enable_pending = 1;
}

void Task_DisableMotors(void)
{
    s_disable_pending = 1;
}

void Task_EStop(void)
{
    s_estop_pending = 1;
    s_state         = TASK_E_STOP;
    s_moving        = 0;
    reset_motion_path();
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
        HAL_Delay(50);
        Motor_DisableAll();
        HAL_Delay(20);
        Motor_DisableAll();
        s_motor_enabled = 0;
        s_angle_known = 0;
        s_estop_pending = 0;
        s_state         = TASK_E_STOP;
        s_moving        = 0;
        reset_motion_path();
        reset_closed_loop();
        return;
    }

    if (s_disable_pending) {
        Motor_StopAll();
        HAL_Delay(50);
        Motor_DisableAll();
        HAL_Delay(20);
        Motor_DisableAll();
        s_motor_enabled = 0;
        s_angle_known = 0;
        s_disable_pending = 0;
        s_state = TASK_IDLE;
        s_moving = 0;
        reset_motion_path();
        reset_closed_loop();
        Buzzer_BeepAsync(2);
        return;
    }

    if (s_enable_pending) {
        Motor_EnableAll();
        s_motor_enabled = 1;
        s_enable_pending = 0;
        sync_motor_angles_from_driver();
        Buzzer_BeepAsync(1);
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
                reset_motion_path();
                reset_closed_loop();
                s_fire_halt = 1;
                sync_motor_angles_from_driver();
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
            enable_for_motion();
            sync_motor_angles_from_driver();
            calc_sync_speeds(zero_angles, speeds);
            Motor_MoveAllSyncSpeeds(zero_angles, speeds,
                                     MOTOR_ACCEL_RPMS,
                                     MOTOR_DECEL_RPMS);
            Kinematics_LaserToCam(0.0f, 0.0f, &s_cam_x, &s_cam_y);
            reset_motion_path();
            s_move_wait_ms = estimate_motion_wait_ms(zero_angles, speeds,
                                                     MOVE_MIN_WAIT_MS,
                                                     MOVE_SETTLE_MARGIN_MS);
            s_laser_x = CIRCLE3_X;
            s_laser_y = CIRCLE3_Y;
            s_moving  = 1;
            s_move_start = HAL_GetTick();
            remember_target_angles(zero_angles);
        } else if (wait_done()) {
            s_moving = 0;
            closed_loop_start(CIRCLE3_X, CIRCLE3_Y);
        }
        break;

    /* ---- AREA_PATROL ---- */
    case TASK_AREA_PATROL:
        if (!s_moving) {
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
                break;
            }
            uint8_t ci = k_area_patrol_route[s_wp_idx];
            const float *comp = (s_wp_idx == 0U) ? NULL : k_area_tilt_comp[ci];
            int8_t from_ci = (s_wp_idx == 0U) ? 2 : (int8_t)k_area_patrol_route[s_wp_idx - 1U];
            move_to_circle_angles(ci, comp, from_ci);
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
            int8_t from_ci = 2;
            if (s_wp_idx > 0U) {
                uint8_t prev = s_seq[s_wp_idx - 1U] - 1U;
                if (prev < 5U) {
                    from_ci = (int8_t)prev;
                }
            }
            move_to_circle_angles(ci, NULL, from_ci);
        } else if (wait_done()) {
            s_moving = 0;
            HAL_Delay(PATROL_DWELL_MS);
            s_wp_idx++;
        }
        break;

    /* ---- AUTO_PATROL ---- */
    case TASK_AUTO_PATROL:
        if (!s_moving) {
            if (s_wp_idx >= s_wp_total) {
                s_state = TASK_IDLE;
                break;
            }
            move_to_auto_point(s_wp_idx);
        } else if (wait_done()) {
            s_moving = 0;
            HAL_Delay(PATROL_DWELL_MS);
            s_wp_idx++;
        }
        break;
    }
}
