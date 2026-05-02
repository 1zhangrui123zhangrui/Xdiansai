/**
 * @file    motor.h
 * @brief   ZDT X42S 第二代闭环步进电机驱动 (USART1 多机通信)
 *
 * 固件版本: X 固件
 * 通信协议: UART 115200 8N1
 * 帧格式:   Addr | FuncCode | Data... | 0x6B
 *
 * X 固件 FD 梯形曲线加减速位置命令 (16 字节):
 *   Addr FD dir acc_hi acc_lo dec_hi dec_lo spd_hi spd_lo
 *        pos3 pos2 pos1 pos0 mode sync 6B
 *   - dir:   0x00=CW(正转/收线), 0x01=CCW(反转/放线)
 *   - acc:   加速加速度, RPM/S, 2字节大端
 *   - dec:   减速加速度, RPM/S, 2字节大端
 *   - speed: 最大速度, 0.1RPM, 2字节大端
 *   - pos:   位置角度, 0.1°, 4字节大端
 *   - mode:  0=相对上次目标, 1=绝对零点, 2=相对当前位置
 *   - sync:  0=立即执行, 1=缓存等待同步触发
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
void Motor_DisableAll(void);
void Motor_Stop(uint8_t id);
void Motor_StopAll(void);

/* ---- 位置归零 (标定) ---- */
void Motor_ZeroPosition(uint8_t id);
void Motor_ZeroAllPositions(void);

/**
 * @brief  绝对位置运动 (X FD 梯形曲线, mode=1: 相对标定原点)
 * @param  id         电机地址 1-4
 * @param  angle_deg  目标角度(°), 正=CW收线, 负=CCW放线
 * @param  speed_rpm  最大速度 (RPM, 0-3000)
 * @param  accel_rpms 加速加速度 (RPM/S)
 * @param  decel_rpms 减速加速度 (RPM/S)
 * @param  sync       0=立即执行, 1=缓存
 */
void Motor_MoveAbsolute(uint8_t id,
                        float   angle_deg,
                        float   speed_rpm,
                        uint16_t accel_rpms,
                        uint16_t decel_rpms,
                        uint8_t sync);

/**
 * @brief  相对当前位置运动 (X FD 梯形曲线, mode=2)
 */
void Motor_MoveRelative(uint8_t id,
                        float   delta_deg,
                        float   speed_rpm,
                        uint16_t accel_rpms,
                        uint16_t decel_rpms,
                        uint8_t sync);

/**
 * @brief  四电机缓存后同步触发
 * @param  angle_deg[4]  各电机目标绝对角度 (°)
 */
void Motor_MoveAllSync(const float angle_deg[4],
                       float speed_rpm,
                       uint16_t accel_rpms,
                       uint16_t decel_rpms);

/**
 * @brief  四电机缓存后同步触发, 每个电机使用独立速度
 */
void Motor_MoveAllSyncSpeeds(const float angle_deg[4],
                             const float speed_rpm[4],
                             uint16_t accel_rpms,
                             uint16_t decel_rpms);

/** 发送多机同步触发命令 00 FF 66 6B */
void Motor_TriggerSync(void);

/**
 * @brief  多机命令帧 (00 AA): 单帧原子发送四条位置命令
 *         仅地址1电机回复 ACK, 不会造成总线冲突
 */
void Motor_MultiPositionCmd(const float angle_deg[4],
                            float speed_rpm,
                            uint16_t accel_rpms,
                            uint16_t decel_rpms);

/**
 * @brief  多机命令帧 (00 AA): 四条位置命令可使用不同速度
 */
void Motor_MultiPositionCmdSpeeds(const float angle_deg[4],
                                  const float speed_rpm[4],
                                  uint16_t accel_rpms,
                                  uint16_t decel_rpms);

/**
 * @brief  多机相对运动命令帧 (00 AA): 四条相对位置命令可使用不同速度
 */
void Motor_MultiPositionDeltaCmdSpeeds(const float delta_deg[4],
                                       const float speed_rpm[4],
                                       uint16_t accel_rpms,
                                       uint16_t decel_rpms);

void Motor_MultiPositionDeltaCmd(const float delta_deg[4],
                                 float speed_rpm,
                                 uint16_t accel_rpms,
                                 uint16_t decel_rpms);

#endif /* __MOTOR_H */
