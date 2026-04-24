/**
 * @file    buzzer.h
 * @brief   蜂鸣器驱动 (需在 CubeMX IOC 中新增蜂鸣器 GPIO Output 引脚)
 */
#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/**
 * @brief  蜂鸣 n 声 (阻塞)
 * @param  times  响的次数
 */
void Buzzer_Beep(uint8_t times);

/**
 * @brief  在主循环中调用, 处理非阻塞蜂鸣序列
 *         先调用 Buzzer_BeepAsync 发起, 再在主循环中周期调用此函数
 */
void Buzzer_Tick(void);

/**
 * @brief  发起异步蜂鸣 (非阻塞)
 * @param  times  响的次数
 */
void Buzzer_BeepAsync(uint8_t times);

#endif /* __BUZZER_H */
