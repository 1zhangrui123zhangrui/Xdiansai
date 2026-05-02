/**
 * @file    motor_test.h
 * @brief   平台坐标移动测试
 *
 * 使用方法:
 *   在 main.c 的 USER CODE BEGIN PD 区域取消注释:
 *       #define MOTOR_TEST_ENABLE
 *   验证完成后重新注释掉，恢复正常运行。
 *
 * 测试流程:
 *   1. main.c 上电清零并初始化运动学
 *   2. MotorTest_Run 使能电机
 *   3. 依次移动到右、上、左、下、右上、右下、左上、左下
 *   4. 每个点停留, 记录实际激光坐标用于后续标定
 */
#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

void MotorTest_Run(void);

#endif /* __MOTOR_TEST_H */
