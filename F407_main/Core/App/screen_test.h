/**
 * @file    screen_test.h
 * @brief   串口屏通信测试
 *
 * 使用方法:
 *   在 main.c 的 USER CODE BEGIN PD 区域取消注释:
 *       #define SCREEN_TEST_ENABLE
 *   验证完成后重新注释掉，恢复正常运行。
 *
 * 测试内容:
 *   1. 向屏幕发送测试坐标，验证屏幕能否正确显示
 *   2. 监听屏幕发来的字节，通过 USART3 (PA11/PA12 调试口) 打印出来
 *      → 在上位机打开 USART3 (115200) 即可看到原始数据
 */
#ifndef __SCREEN_TEST_H
#define __SCREEN_TEST_H

void ScreenTest_Run(void);

#endif /* __SCREEN_TEST_H */
