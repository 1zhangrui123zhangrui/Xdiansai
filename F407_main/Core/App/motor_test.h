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
 *   1. 标定 (清零所有电机, 初始化运动学)
 *   2. 依次移动到预设坐标列表
 *   3. 每个坐标停留 COORD_TEST_DWELL_MS 毫秒
 *   4. 最后回到 (0,0) 中心
 *
 * 修改坐标: 在 motor_test.c 的 k_test_points[] 中修改。
 */
#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

void MotorTest_Run(void);

#endif /* __MOTOR_TEST_H */
