/**
 * @file    screen.h
 * @brief   串口屏通信驱动
 *
 * 屏→MCU: 帧头 0x55 0x55 + CMD
 * MCU→屏: 文本指令 + 0xFF 0xFF 0xFF
 */
#ifndef __SCREEN_H
#define __SCREEN_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 命令码 (屏发给 MCU) */
#define CMD_HOME            0xA0U   /* 返回/急停 */
#define CMD_START           0xA1U   /* 启动: 回中心 */
#define CMD_AREA_PATROL     0xA2U   /* 区域巡逻 */
#define CMD_SEQ_PATROL      0xA3U   /* 顺序巡逻 (后跟 5 字节 ASCII + 0xFE) */
#define CMD_AUTO_PATROL     0xA4U   /* 自动巡逻 */
#define CMD_MOTOR_ENABLE    0xA5U   /* 电机使能/抱轴 */
#define CMD_MOTOR_DISABLE   0xA6U   /* 电机失能/松轴 */

typedef void (*ScreenCmdCallback_t)(void);
typedef void (*ScreenSeqCallback_t)(uint8_t *seq);  /* seq[5]: 1-5 */

void Screen_Init(UART_HandleTypeDef *huart);
void Screen_RegisterCallback(uint8_t cmd, ScreenCmdCallback_t cb);
void Screen_RegisterSequenceCallback(ScreenSeqCallback_t cb);
uint8_t Screen_GetCurrentPage(void);

/** 每 100ms 调用, 刷新当前页面坐标控件 tX / tY */
void Screen_SetCoord(float x, float y);

/** 坐标丢失时显示 LOST */
void Screen_SetCoordLost(void);

/** 记录火源坐标并发送到屏 (最多 2 个) */
void Screen_RecordFire(float x, float y);

/** 通用文本控件赋值 */
void Screen_SetText(const char *objname, const char *text);

/** 中断回调中调用 (逐字节喂给状态机) */
void Screen_OnByteReceived(uint8_t byte);

#endif /* __SCREEN_H */
