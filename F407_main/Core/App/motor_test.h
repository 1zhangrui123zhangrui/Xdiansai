/**
 * @file    motor_test.h
 * @brief   电机功能验证测试
 *
 * 使用方法:
 *   在 main.c 的 USER CODE BEGIN PD 区域加入:
 *       #define MOTOR_TEST_ENABLE
 *   然后在 USER CODE BEGIN 2 调用 MotorTest_Run()
 *   验证完成后注释掉 MOTOR_TEST_ENABLE 恢复正常运行
 */
#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

/**
 * @brief  运行电机测试序列 (阻塞)
 *
 * 测试步骤:
 *  1. 使能电机 1
 *  2. 正转 1 圈 (CW, 3200 脉冲, mode=相对当前)
 *  3. 等待 3s
 *  4. 反转 1 圈回原位 (CCW)
 *  5. 等待 3s
 *  6. 重复 3 次
 */
void MotorTest_Run(void);

#endif /* __MOTOR_TEST_H */
