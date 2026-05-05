/**
 * @file    kinematics.h
 * @brief   绳驱平台运动学: 目标坐标 → 各电机绝对角度
 *
 * 坐标系: 以中心橙色圆为原点, X 轴向右 (1→2 电机方向), Y 轴向上
 *
 * 绳长公式:
 *   platform_center = laser_target - laser_offset
 *   corner_i = platform_center + platform_corner_offset_i
 *   L_i = sqrt( (corner_i_x - motor_i_x)^2
 *             + (corner_i_y - motor_i_y)^2 + H^2 )
 * 其中 H = ROPE_H_CM (滑轮顶部到平台的竖直距离)
 *
 * 电机角度 (以标定原点为零):
 *   angle_i = (L_i_at_zero - L_i_at_target) / (2π × R_spool) × 360°
 *   正值 → CW (收线), 负值 → CCW (放线)
 */
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include <stdint.h>

/**
 * @brief  计算平台中心移动到 (cam_x, cam_y) 时各电机所需的绝对角度
 *
 * @param  cam_x      平台中心 X 坐标 (cm, 以中心圆为原点)
 * @param  cam_y      平台中心 Y 坐标 (cm)
 * @param  angles_deg 输出: 四个电机绝对角度 (°), 正=CW, 负=CCW
 *                    indices: [0]=Motor1, [1]=Motor2, [2]=Motor3, [3]=Motor4
 */
void Kinematics_CamToAngles(float cam_x, float cam_y,
                             float angles_deg[4]);

/**
 * @brief  将激光目标坐标转换为平台中心目标坐标
 *         默认: cam_x = laser_x - LASER_OFFSET_X_CM,
 *              cam_y = laser_y - LASER_OFFSET_Y_CM。
 *         当前实物: 激光点在平台中心 +Y 方向 3.5cm,
 *         即激光点在中心圆 (0,0) 时, 平台中心在 (0,-3.5)。
 *         若 KINEMATICS_SWAP_XY=1, 则先交换现场 X/Y 轴再补偿激光偏移
 */
void Kinematics_LaserToCam(float laser_x, float laser_y,
                            float *cam_x, float *cam_y);

/**
 * @brief  初始化: 计算并缓存标定原点处的绳长 (供后续差分)
 *         在标定完成 (所有电机角度清零) 后调用一次
 */
void Kinematics_Init(void);

#endif /* __KINEMATICS_H */
