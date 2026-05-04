/**
 * @file    motor_test.c
 * @brief   平台坐标移动测试
 *
 * 调用完整的 motor + kinematics 栈，验证平台能否按坐标正确移动。
 * 坐标单位: cm，以中心圆为原点，激光坐标系。
 */
#include "motor_test.h"
#include "motor.h"
#include "kinematics.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 1=只断使能松轴; 0=执行坐标移动测试 */
#define MOTOR_TEST_DISABLE_ONLY  0U

#if !MOTOR_TEST_DISABLE_ONLY
/* ---- 参数 ---- */

/* 移动速度 (RPM): 调低便于观察 */
#define COORD_TEST_SPEED_RPM    10U

/* 梯形曲线加/减速度 (RPM/S) */
#define COORD_TEST_ACCEL_RPMS   10U
#define COORD_TEST_DECEL_RPMS   10U

/* 到达每个坐标后停留时间 (ms) */
#define COORD_TEST_DWELL_MS     5000U

/* 每段运动等待超时 (ms): 一次到位, 给足观察裕量 */
#define COORD_TEST_MOVE_MS      30000U

/* ---- 测试坐标表 (激光坐标, cm) ---- */
/* 上电中心清零后, 只测试四个角点, 最后回中心。 */
static const float k_test_points[][2] = {
    {-20.0f, -20.0f},   /* 左下 */
    { 20.0f, -20.0f},   /* 右下 */
    { 20.0f,  20.0f},   /* 右上 */
    {-20.0f,  20.0f},   /* 左上 */
    {  0.0f,   0.0f},   /* 中心 */
};
#define TEST_POINT_CNT  (sizeof(k_test_points) / sizeof(k_test_points[0]))

/* ---- 内部函数 ---- */

/** 直接发送一次激光坐标绝对位置运动命令并等待 */
static void move_once_and_wait(float lx, float ly)
{
    float cam_x, cam_y;
    Kinematics_LaserToCam(lx, ly, &cam_x, &cam_y);

    float angles[4];
    Kinematics_CamToAngles(cam_x, cam_y, angles);

    Motor_MultiPositionCmd(angles,
                           (float)COORD_TEST_SPEED_RPM,
                           COORD_TEST_ACCEL_RPMS,
                           COORD_TEST_DECEL_RPMS);

    HAL_Delay(COORD_TEST_MOVE_MS);
}
#endif

/* ---- 对外接口 ---- */

void MotorTest_Run(void)
{
#if MOTOR_TEST_DISABLE_ONLY
    /* 断使能测试: 保持松轴, 方便手动拉动平台和检查机械状态。 */
    Motor_StopAll();
    HAL_Delay(100);
    Motor_DisableAll();

    while (1) {
        HAL_Delay(1000);
    }
#else
    /* 主程序已在中心完成清零和运动学初始化, 这里直接使能并执行测试路径。 */
    Motor_EnableAll();
    HAL_Delay(100);

    for (uint8_t i = 0; i < TEST_POINT_CNT; i++) {
        move_once_and_wait(k_test_points[i][0], k_test_points[i][1]);
        HAL_Delay(COORD_TEST_DWELL_MS);
    }

    Motor_StopAll();
#endif
}
