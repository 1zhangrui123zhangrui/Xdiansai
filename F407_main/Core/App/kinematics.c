/**
 * @file    kinematics.c
 * @brief   绳驱平台运动学实现
 *
 * 由于滑轮顶部与平台几乎水平 (ROPE_H_CM≈0), 绳长退化为纯 2D 平面距离:
 *   L_i = sqrt( (px - EA_i_x)^2 + (py - EA_i_y)^2 )
 *
 * 有效锚点 EA_i = 滑轮投影坐标 - 平台角点相对中心偏移
 */
#include "kinematics.h"
#include "platform_config.h"
#include <math.h>

typedef struct { float x; float y; } Vec2;

/* 有效锚点: EA_i = Motor_i_proj - Platform_corner_i_offset */
static const Vec2 k_ea[4] = {
    /* Motor1 左下: 平台角点 (-half, -half) */
    { MOTOR1_PROJ_X - (-PLATFORM_HALF_CM), MOTOR1_PROJ_Y - (-PLATFORM_HALF_CM) },
    /* Motor2 右下: 平台角点 (+half, -half) */
    { MOTOR2_PROJ_X - ( PLATFORM_HALF_CM), MOTOR2_PROJ_Y - (-PLATFORM_HALF_CM) },
    /* Motor3 右上: 平台角点 (+half, +half) */
    { MOTOR3_PROJ_X - ( PLATFORM_HALF_CM), MOTOR3_PROJ_Y - ( PLATFORM_HALF_CM) },
    /* Motor4 左上: 平台角点 (-half, +half) */
    { MOTOR4_PROJ_X - (-PLATFORM_HALF_CM), MOTOR4_PROJ_Y - ( PLATFORM_HALF_CM) },
};

static float k_l0[4];          /* 标定原点 (0,0) 处各电机绳长 (cm) */
static float k_spool_circ_cm;  /* 绕线轮周长 (cm) */

/** 计算摄像头中心在 (cx, cy) 时电机 i 的绳长 (cm) */
static float rope_len(uint8_t i, float cx, float cy)
{
    float dx = cx - k_ea[i].x;
    float dy = cy - k_ea[i].y;
    float h  = ROPE_H_CM;   /* H=0 时 h*h=0, 等价于纯 2D */
    return sqrtf(dx * dx + dy * dy + h * h);
}

void Kinematics_Init(void)
{
    k_spool_circ_cm = 2.0f * (float)M_PI * SPOOL_RADIUS_CM;
    /* 上电时激光在(0,0), 摄像头在(-LASER_OFFSET_X_CM, 0),
     * 电机零点就在此物理位置, 基准绳长必须用摄像头实际起始坐标计算。 */
    for (uint8_t i = 0; i < 4; i++) {
        k_l0[i] = rope_len(i, -LASER_OFFSET_X_CM, 0.0f);
    }
}

void Kinematics_LaserToCam(float laser_x, float laser_y,
                            float *cam_x, float *cam_y)
{
    *cam_x = laser_x - LASER_OFFSET_X_CM;
    *cam_y = laser_y;
}

void Kinematics_CamToAngles(float cam_x, float cam_y,
                             float angles_deg[4])
{
    for (uint8_t i = 0; i < 4; i++) {
        float lt = rope_len(i, cam_x, cam_y);
        /* ΔL = L_zero - L_target: 正→绳子变短→收线→CW→正角度 */
        float delta_cm  = k_l0[i] - lt;
        /* angle(°) = ΔL / 周长 × 360° */
        angles_deg[i] = delta_cm / k_spool_circ_cm * 360.0f;
    }
}
