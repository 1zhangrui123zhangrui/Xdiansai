/**
 * @file    kinematics.c
 * @brief   绳驱平台运动学实现
 *
 * 绳长模型:
 *   corner_i = platform_center + platform_corner_offset_i
 *   L_i = sqrt( (corner_i_x - motor_i_x)^2
 *             + (corner_i_y - motor_i_y)^2 + H^2 )
 */
#include "kinematics.h"
#include "platform_config.h"
#include <math.h>

typedef struct { float x; float y; } Vec2;

static const Vec2 k_motor_proj[4] = {
    { MOTOR1_PROJ_X, MOTOR1_PROJ_Y },
    { MOTOR2_PROJ_X, MOTOR2_PROJ_Y },
    { MOTOR3_PROJ_X, MOTOR3_PROJ_Y },
    { MOTOR4_PROJ_X, MOTOR4_PROJ_Y },
};

static const Vec2 k_corner_offset[4] = {
    { -PLATFORM_HALF_CM, -PLATFORM_HALF_CM },  /* Motor1 左下挂点 */
    {  PLATFORM_HALF_CM, -PLATFORM_HALF_CM },  /* Motor2 右下挂点 */
    {  PLATFORM_HALF_CM,  PLATFORM_HALF_CM },  /* Motor3 右上挂点 */
    { -PLATFORM_HALF_CM,  PLATFORM_HALF_CM },  /* Motor4 左上挂点 */
};

static float k_l0[4];          /* 标定原点 (0,0) 处各电机绳长 (cm) */
static float k_spool_circ_cm;  /* 绕线轮周长 (cm) */

static float angle_scale(uint8_t i)
{
    static const float k_scale[4] = {
        MOTOR1_ANGLE_SCALE,
        MOTOR2_ANGLE_SCALE,
        MOTOR3_ANGLE_SCALE,
        MOTOR4_ANGLE_SCALE,
    };
    return k_scale[i];
}

/** 计算平台中心在 (cx, cy) 时电机 i 的绳长 (cm) */
static float rope_len(uint8_t i, float cx, float cy)
{
    float corner_x = cx + k_corner_offset[i].x;
    float corner_y = cy + k_corner_offset[i].y;
    float dx = corner_x - k_motor_proj[i].x;
    float dy = corner_y - k_motor_proj[i].y;
    float h  = ROPE_H_CM;   /* H=0 时 h*h=0, 等价于纯 2D */
    return sqrtf(dx * dx + dy * dy + h * h);
}

void Kinematics_Init(void)
{
    float cam_x, cam_y;

    k_spool_circ_cm = 2.0f * (float)M_PI * SPOOL_RADIUS_CM;
    /* 上电时激光在(0,0), 平台中心实际在
     * (-LASER_OFFSET_X_CM, -LASER_OFFSET_Y_CM),
     * 基准绳长必须用平台中心实际起始坐标计算。 */
    Kinematics_LaserToCam(0.0f, 0.0f, &cam_x, &cam_y);
    for (uint8_t i = 0; i < 4; i++) {
        k_l0[i] = rope_len(i, cam_x, cam_y);
    }
}

void Kinematics_LaserToCam(float laser_x, float laser_y,
                            float *cam_x, float *cam_y)
{
    float target_x = laser_x;
    float target_y = laser_y;

#if KINEMATICS_CALIB_ENABLE
    target_x = KINEMATICS_CALIB_XX * laser_x + KINEMATICS_CALIB_XY * laser_y;
    target_y = KINEMATICS_CALIB_YX * laser_x + KINEMATICS_CALIB_YY * laser_y;
#endif

#if KINEMATICS_SWAP_XY
    /* 现场安装坐标修正: 代码目标坐标 (x,y) 映射到运动学坐标 (y,x)。
     * 交换后再按运动学坐标系补偿激光相对平台中心的偏移。 */
    *cam_x = target_y - LASER_OFFSET_X_CM;
    *cam_y = target_x - LASER_OFFSET_Y_CM;
#else
    *cam_x = target_x - LASER_OFFSET_X_CM;
    *cam_y = target_y - LASER_OFFSET_Y_CM;
#endif
}

void Kinematics_CamToAngles(float cam_x, float cam_y,
                             float angles_deg[4])
{
    float center_cam_x, center_cam_y;
    Kinematics_LaserToCam(0.0f, 0.0f, &center_cam_x, &center_cam_y);
    uint8_t is_center = (fabsf(cam_x - center_cam_x) < 0.2f) &&
                        (fabsf(cam_y - center_cam_y) < 0.2f);

    for (uint8_t i = 0; i < 4; i++) {
        float lt = rope_len(i, cam_x, cam_y);
        /* ΔL = L_zero - L_target: 正→绳子变短→收线→CW→正角度 */
        float delta_cm  = k_l0[i] - lt;
        /* angle(°) = ΔL / 周长 × 360° */
        angles_deg[i] = delta_cm / k_spool_circ_cm * 360.0f * angle_scale(i);
    }

    if (is_center) {
        angles_deg[1] += MOTOR2_CENTER_TAKEUP_DEG;
    }
}
