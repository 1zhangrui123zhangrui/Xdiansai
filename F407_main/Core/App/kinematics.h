/**
 * @file    kinematics.h
 * @brief   绳驱平台运动学: 目标坐标 → 各电机绝对角度
 *
 * 坐标系: 以中心橙色圆为原点, X 轴向右 (1→2 电机方向), Y 轴向上
 *
 * 绳长公式:
 *   L_i = sqrt( (px - EA_i_x)^2 + (py - EA_i_y)^2 + H^2 )
 * 其中 EA_i = 滑轮投影坐标 - 平台角点相对中心偏移
 *      H    = ROPE_H_CM (滑轮顶部到平台的竖直距离)
 *
 * 电机角度 (以标定原点为零):
 *   angle_i = (L_i_at_zero - L_i_at_target) / (2π × R_spool) × 360°
 *   正值 → CW (收线), 负值 → CCW (放线)
 */
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/**
 * @brief  计算平台移动到 (cam_x, cam_y) 时各电机所需的绝对角度
 *
 * @param  cam_x      摄像头 X 坐标 (cm, 以中心为原点)
 * @param  cam_y      摄像头 Y 坐标 (cm)
 * @param  angles_deg 输出: 四个电机绝对角度 (°), 正=CW, 负=CCW
 *                    indices: [0]=Motor1, [1]=Motor2, [2]=Motor3, [3]=Motor4
 */
void Kinematics_CamToAngles(float cam_x, float cam_y,
                             float angles_deg[4]);

/**
 * @brief  将激光目标坐标转换为摄像头目标坐标
 *         cam_x = laser_x - LASER_OFFSET_X_CM
 *         cam_y = laser_y
 */
void Kinematics_LaserToCam(float laser_x, float laser_y,
                            float *cam_x, float *cam_y);

/**
 * @brief  初始化: 计算并缓存标定原点处的绳长 (供后续差分)
 *         在标定完成 (所有电机角度清零) 后调用一次
 */
void Kinematics_Init(void);

#endif /* __KINEMATICS_H */
