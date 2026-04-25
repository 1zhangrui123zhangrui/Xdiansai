/**
 * @file    motor_test.c
 * @brief   平台坐标移动测试
 *
 * 调用完整的 motor + kinematics 栈，验证平台能否按坐标正确移动。
 * 坐标单位: cm，以平台中心为原点，激光坐标系。
 */
#include "motor_test.h"
#include "motor.h"
#include "kinematics.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---- 参数 ---- */

/* 移动速度 (RPM): 调低便于观察 */
#define COORD_TEST_SPEED_RPM    200U

/* 加速档位 */
#define COORD_TEST_ACC          30U

/* 到达每个坐标后停留时间 (ms) */
#define COORD_TEST_DWELL_MS     3000U

/* 每段运动等待超时 (ms): 给足裕量 */
#define COORD_TEST_MOVE_MS      6000U

/* ---- 测试坐标表 (激光坐标, cm) ---- */
/* 根据需要修改这里的坐标和数量 */
static const float k_test_points[][2] = {
    { 20.0f,   0.0f},   /* 右  */
   {  0.0f,  20.0f},   /* 上  */
    {-20.0f,   0.0f},   /* 左  */
    {  0.0f, -20.0f},   /* 下  */
    { 20.0f,  20.0f},   /* 右上 */
   {-20.0f, -20.0f},   /* 左下 */
    {  0.0f,   0.0f},   /* 回中心 */
};
#define TEST_POINT_CNT  (sizeof(k_test_points) / sizeof(k_test_points[0]))

/* ---- 内部函数 ---- */

/** 移动到激光坐标 (lx, ly) 并等待到位 */
static void move_and_wait(float lx, float ly)
{
    float cam_x, cam_y;
    Kinematics_LaserToCam(lx, ly, &cam_x, &cam_y);

    float angles[4];
    Kinematics_CamToAngles(cam_x, cam_y, angles);

    Motor_MultiPositionCmd(angles,
                           (float)COORD_TEST_SPEED_RPM,
                           COORD_TEST_ACC);

    HAL_Delay(COORD_TEST_MOVE_MS);
}

/* ---- 对外接口 ---- */

void MotorTest_Run(void)
{
    /* 1. 使能所有电机 */
    Motor_EnableAll();
    HAL_Delay(100);

    /* 2. 标定: 把平台手动放到中心后调用 */
    Motor_ZeroAllPositions();
    Kinematics_Init();
    HAL_Delay(200);

    /* 3. 依次移动到各测试坐标 */
    for (uint8_t i = 0; i < TEST_POINT_CNT; i++) {
        move_and_wait(k_test_points[i][0], k_test_points[i][1]);
        HAL_Delay(COORD_TEST_DWELL_MS);
    }

    /* 4. 停止 */
    Motor_StopAll();
}
