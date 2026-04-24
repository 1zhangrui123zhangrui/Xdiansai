/**
 * @file    motor.h
 * @brief   ZDT X42S 第二代闭环步进电机驱动 (USART1 多机通信)
 *
 * 固件版本: Emm 固件 (X42S V1.0 出厂默认)
 * 通信协议: UART 115200 8N1
 * 帧格式:   Addr | FuncCode | Data... | 0x6B
 *
 * Emm 固件 FD 位置命令 (13 字节):
 *   Addr FD dir speed_hi speed_lo acc pulse3 pulse2 pulse1 pulse0 mode sync 6B
 *   - dir:    0x00=CW(正转/收线), 0x01=CCW(反转/放线)
 *   - speed:  RPM, 0-3000, 2字节大端
 *   - acc:    加速档位 0-255 (0=直接起速, 越大加速越快)
 *   - pulses: 脉冲数, 1.8°步进+16细分=3200脉冲/圈
 *   - mode:   0=相对上次目标, 1=绝对零点, 2=相对当前位置
 *   - sync:   0=立即执行, 1=缓存等待同步触发
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---- 初始化 ---- */
void Motor_Init(UART_HandleTypeDef *huart);

/* ---- 使能/停止 ---- */
void Motor_Enable(uint8_t id);
void Motor_Disable(uint8_t id);
void Motor_EnableAll(void);
void Motor_Stop(uint8_t id);
void Motor_StopAll(void);

/* ---- 位置归零 (标定) ---- */
void Motor_ZeroPosition(uint8_t id);
void Motor_ZeroAllPositions(void);

/**
 * @brief  绝对位置运动 (Emm FD, mode=1: 相对标定原点)
 * @param  id         电机地址 1-4
 * @param  angle_deg  目标角度(°), 正=CW收线, 负=CCW放线
 * @param  speed_rpm  速度 (RPM, 0-3000)
 * @param  acc        加速档位 (0-255, 50 为适中)
 * @param  sync       0=立即执行, 1=缓存
 */
void Motor_MoveAbsolute(uint8_t id,
                        float   angle_deg,
                        float   speed_rpm,
                        uint8_t acc,
                        uint8_t sync);

/**
 * @brief  四电机缓存后同步触发
 * @param  angle_deg[4]  各电机目标绝对角度 (°)
 */
void Motor_MoveAllSync(const float angle_deg[4],
                       float speed_rpm,
                       uint8_t acc);

/** 发送多机同步触发命令 00 FF 66 6B */
void Motor_TriggerSync(void);

/**
 * @brief  多机命令帧 (00 AA): 单帧原子发送四条位置命令
 *         仅地址1电机回复 ACK, 不会造成总线冲突
 */
void Motor_MultiPositionCmd(const float angle_deg[4],
                            float speed_rpm,
                            uint8_t acc);

#endif /* __MOTOR_H */
