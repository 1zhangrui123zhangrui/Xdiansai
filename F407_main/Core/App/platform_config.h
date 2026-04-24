/**
 * @file    platform_config.h
 * @brief   全局硬件与系统参数宏定义
 *
 * 所有不确定的硬件参数在此处以宏定义形式集中管理，
 * 调试时只需修改本文件。
 */
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include "stm32f4xx_hal.h"
#include "usart.h"

/* ============================================================
 * 电机 UART (USART1, PA9=TX, PA10=RX)
 * 四台 ZDT X42S 电机通过 RS-485 / TTL 连接到 USART1
 * 固件: Emm 固件 (X42S V1.0 出厂默认)
 * ============================================================ */
#define MOTOR_UART              (&huart1)
#define MOTOR_BAUD              115200U
#define MOTOR_CHECKSUM          0x6BU   /* 出厂默认校验字节 */

/* 电机 ID */
#define MOTOR_ID_1              0x01U   /* 1号: 左下 */
#define MOTOR_ID_2              0x02U   /* 2号: 右下 */
#define MOTOR_ID_3              0x03U   /* 3号: 右上 */
#define MOTOR_ID_4              0x04U   /* 4号: 左上 */
#define MOTOR_ID_BROADCAST      0x00U

/* ============================================================
 * 机械几何参数 (单位: cm)
 * ============================================================ */

/* 电机轴在纸面上的投影坐标 (以中心圆为原点) */
#define MOTOR1_PROJ_X   (-26.7f)
#define MOTOR1_PROJ_Y   (-26.7f)
#define MOTOR2_PROJ_X   ( 26.7f)
#define MOTOR2_PROJ_Y   (-26.7f)
#define MOTOR3_PROJ_X   ( 26.7f)
#define MOTOR3_PROJ_Y   ( 26.7f)
#define MOTOR4_PROJ_X   (-26.7f)
#define MOTOR4_PROJ_Y   ( 26.7f)

/* 平台尺寸 (10cm×10cm 正方形, 绳子固定在四角) */
#define PLATFORM_HALF_CM    5.0f

/* 绳子竖直分量 (cm)
 * 滑轮顶部与平台几乎水平 → H≈0 */
#define ROPE_H_CM           0.0f

/* 绕线轮半径 (cm): 实测 35mm = 3.5cm */
#define SPOOL_RADIUS_CM     3.5f

/* 激光笔相对摄像头中心的偏移 (cm, 沿 X 轴正方向) */
#define LASER_OFFSET_X_CM   3.5f

/* ============================================================
 * 橙色圆形区域的激光目标坐标 (cm, 以中心圆为原点)
 * ============================================================ */
#define CIRCLE1_X   (-20.0f)    /* 左下 (BL) */
#define CIRCLE1_Y   (-20.0f)
#define CIRCLE2_X   ( 20.0f)    /* 右下 (BR) */
#define CIRCLE2_Y   (-20.0f)
#define CIRCLE3_X   ( 20.0f)    /* 右上 (TR) */
#define CIRCLE3_Y   ( 20.0f)
#define CIRCLE4_X   (-20.0f)    /* 左上 (TL) */
#define CIRCLE4_Y   ( 20.0f)
#define CIRCLE5_X   (  0.0f)    /* 中心 */
#define CIRCLE5_Y   (  0.0f)

/* ============================================================
 * 电机运动参数 (Emm 固件)
 * ============================================================ */
#define MOTOR_SPEED_RPM         300U    /* 运动速度 (RPM, 0-3000) */
#define MOTOR_ACCEL_LEVEL       50U     /* 加速档位 (0-255, 0=直接起速, 越大越快) */
#define MOTOR_MOVE_TIMEOUT_MS   8000U   /* 单段运动超时 (ms) */

/* ============================================================
 * NRF24L01 GPIO (SPI1: PA5=SCK, PA6=MISO, PA7=MOSI)
 * PA8=CE, PC9=CSN, PC8=IRQ
 * ============================================================ */
#define NRF_CE_PORT     GPIOA
#define NRF_CE_PIN      GPIO_PIN_8
#define NRF_CSN_PORT    GPIOC
#define NRF_CSN_PIN     GPIO_PIN_9
#define NRF_IRQ_PORT    GPIOC
#define NRF_IRQ_PIN     GPIO_PIN_8

/* NRF 有效载荷长度 */
#define NRF_TX_WIDTH    4U
#define NRF_RX_WIDTH    8U

/* ============================================================
 * 蜂鸣器 GPIO
 * TODO: 在 CubeMX IOC 中新增该 GPIO Output 引脚后修改以下宏
 * ============================================================ */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_PIN     GPIO_PIN_8
#define BUZZER_ON_LEVEL     GPIO_PIN_SET
#define BUZZER_OFF_LEVEL    GPIO_PIN_RESET
#define BUZZER_BEEP_ON_MS   300U
#define BUZZER_BEEP_OFF_MS  200U
#define BUZZER_FIRE_BEEPS   3U

/* ============================================================
 * 串口屏 UART (USART2, PA2=TX, PA3=RX)
 * ============================================================ */
#define SCREEN_UART     (&huart2)

/* ============================================================
 * 任务调度参数
 * ============================================================ */
#define COORD_UPDATE_INTERVAL_MS    100U
#define PATROL_DWELL_MS             300U
#define AUTO_PATROL_STEP_CM         10.0f
#define AUTO_PATROL_X_MIN_CM        (-20.0f)
#define AUTO_PATROL_X_MAX_CM        ( 20.0f)
#define AUTO_PATROL_Y_MIN_CM        (-20.0f)
#define AUTO_PATROL_Y_MAX_CM        ( 20.0f)
#define POSITION_TOL_CM             2.0f

#endif /* __PLATFORM_CONFIG_H */
