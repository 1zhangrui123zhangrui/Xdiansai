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
 * 固件: X 固件
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

/* 绳子出线点在纸面上的投影坐标 (以中心圆为原点) */
#define MOTOR1_PROJ_X   (-29.5f)
#define MOTOR1_PROJ_Y   (-30.5f)
#define MOTOR2_PROJ_X   ( 30.5f)
#define MOTOR2_PROJ_Y   (-31.0f)
#define MOTOR3_PROJ_X   ( 30.5f)
#define MOTOR3_PROJ_Y   ( 30.5f)
#define MOTOR4_PROJ_X   (-30.0f)
#define MOTOR4_PROJ_Y   ( 30.5f)

/* 平台尺寸 (10cm×10cm 正方形, 绳子固定在四角) */
#define PLATFORM_HALF_CM    5.0f

/* 绳子竖直分量 (cm)
 * 出线点到平台挂点的垂直高度差。高度必须接近真实值, 否则大范围移动会失张力/歪斜。 */
#define ROPE_H_CM           0.0f

/* 绕线轮半径 (cm): 实测直径 3.5cm, 半径 1.75cm */
#define SPOOL_RADIUS_CM     1.75f

/* 激光点相对平台/摄像头中心的偏移 (cm)
 * 当前平台旋转后, 摄像头坐标系和电机坐标系统一。
 * 激光点在平台中心的 +Y 方向 3.5cm, 所以激光点在目标 (0,0) 时,
 * 平台/摄像头中心应位于 (0,-3.5)。 */
#define LASER_OFFSET_X_CM   0.0f
#define LASER_OFFSET_Y_CM   3.5f

/* 现场坐标轴修正: 1=目标坐标进入运动学前交换 X/Y */
#define KINEMATICS_SWAP_XY  0U

/* 开环坐标标定补偿
 * 由实测“目标坐标 -> 实际坐标”拟合得到, 用于把期望激光坐标先转换成
 * 更大的虚拟目标坐标, 再进入绳长运动学。
 *
 * 拟合数据:
 *   右(20,0)->(12.5,0), 上(0,20)->(0,11.7), 左(-20,0)->(-9.8,0),
 *   下(0,-20)->(0,-11.5), 四个角点同理。
 *
 * 实际约为:
 *   actual_x = 0.5775 * cmd_x + 0.008333 * cmd_y
 *   actual_y = -0.005833 * cmd_x + 0.525833 * cmd_y
 *
 * 因此这里使用其逆矩阵:
 *   cmd_x = 1.731325 * target_x - 0.027438 * target_y
 *   cmd_y = 0.019206 * target_x + 1.901439 * target_y
 */
#define KINEMATICS_CALIB_ENABLE     0U
#define KINEMATICS_CALIB_XX         ( 0.904673f)
#define KINEMATICS_CALIB_XY         (-0.030872f)
#define KINEMATICS_CALIB_YX         (-0.051452f)
#define KINEMATICS_CALIB_YY         ( 0.926697f)

/* 电机方向修正: 1=正常, -1=反向。当前实物中 2号/4号需要取反。 */
#define MOTOR1_DIR_SIGN     ( 1.0f)
#define MOTOR2_DIR_SIGN     ( -1.0f)
#define MOTOR3_DIR_SIGN     ( 1.0f)
#define MOTOR4_DIR_SIGN     ( -1.0f)

/* 电机绳长/卷筒等效比例微调: >1 表示该电机收放绳更多 */
#define MOTOR1_ANGLE_SCALE  ( 1.00f)
#define MOTOR2_ANGLE_SCALE  ( 1.00f)
#define MOTOR3_ANGLE_SCALE  ( 1.00f)
#define MOTOR4_ANGLE_SCALE  ( 1.00f)

/* 回中心时给 2 号一个很小的额外收线量, 用于补偿中心松绳 */
#define MOTOR2_CENTER_TAKEUP_DEG  ( 0.0f)

/* ============================================================
 * 橙色圆形区域的激光目标坐标 (cm, 以中心圆为原点)
 * ============================================================ */
#define CIRCLE1_X   (-20.0f)    /* 新圆1 = 旧圆4: 左上 (TL) */
#define CIRCLE1_Y   ( 20.0f)
#define CIRCLE2_X   ( 20.0f)    /* 新圆2 = 旧圆3: 右上 (TR) */
#define CIRCLE2_Y   ( 20.0f)
#define CIRCLE3_X   (  0.0f)    /* 新圆3 = 旧圆5: 中心 */
#define CIRCLE3_Y   (  0.0f)
#define CIRCLE4_X   (-20.0f)    /* 新圆4 = 旧圆1: 左下 (BL) */
#define CIRCLE4_Y   (-20.0f)
#define CIRCLE5_X   ( 20.0f)    /* 新圆5 = 旧圆2: 右下 (BR) */
#define CIRCLE5_Y   (-20.0f)

/* ============================================================
 * 电机运动参数 (X 固件梯形曲线加减速位置模式)
 * ============================================================ */
#define MOTOR_SPEED_RPM         40U     /* 最大速度 (RPM, 0-3000) */
#define MOTOR_ACCEL_RPMS        50U     /* 加速加速度 (RPM/S, 0-65535) */
#define MOTOR_DECEL_RPMS        50U     /* 减速加速度 (RPM/S, 0-65535) */

/* 上电是否把当前电机位置记录为 0 度
 * 1=上电自动清零: 只在手动把激光点放到中心后用于建立零点。
 * 0=上电不清零: 比赛/日常运行推荐, 避免手拉随机位置后覆盖中心零点。 */
#define BOOT_AUTO_ZERO_ENABLE   0U

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
#define NRF_TX_WIDTH    20U
#define NRF_RX_WIDTH    20U

/* ============================================================
 * 蜂鸣器 GPIO
 * TODO: 在 CubeMX IOC 中新增该 GPIO Output 引脚后修改以下宏
 * ============================================================ */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_PIN     GPIO_PIN_8
#define BUZZER_ON_LEVEL     GPIO_PIN_RESET
#define BUZZER_OFF_LEVEL    GPIO_PIN_SET
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
#define MOTION_SEGMENT_STEP_CM      1.0f
#define ANGLE_SEGMENT_MAX_DEG       30.0f
#define AREA_TILT_COMP_DEG          0.0f
#define AREA_EDGE_SLOW_RELEASE_ENABLE  1U
#define AREA_EDGE_SLOW_RELEASE_POWER   2.0f
#define AREA_EDGE_FAST_TAKEUP_ENABLE   1U
#define AREA_EDGE_FAST_TAKEUP_POWER    2.0f
#define AREA_SIDE_CURVE_ENABLE         1U
#define AUTO_STRESS_RELIEF_DEG         15.0f
#define AUTO_RISK_SLOW_TAKEUP_POWER    1.4f
#define AUTO_SEGMENT_RELEASE_DEG       8.0f
#define AUTO_LIGHT_CURVE_POWER         1.25f
#define MOTION_SEGMENT_MIN_WAIT_MS  60U
#define MOTION_SEGMENT_MARGIN_MS    25U
#define AUTO_PATROL_STEP_CM         10.0f
#define AUTO_PATROL_X_MIN_CM        (-20.0f)
#define AUTO_PATROL_X_MAX_CM        ( 20.0f)
#define AUTO_PATROL_Y_MIN_CM        (-20.0f)
#define AUTO_PATROL_Y_MAX_CM        ( 20.0f)
#define POSITION_TOL_CM             2.0f

/* ============================================================
 * 视觉闭环修正参数
 * 流程: 开环运动 -> 停稳 -> 采 3 帧视觉坐标 -> 小步补偿 -> 再确认
 * ============================================================ */
#define CLOSED_LOOP_ENABLE              1U
#define CLOSED_LOOP_SETTLE_MS           250U
#define CLOSED_LOOP_SAMPLE_TIMEOUT_MS   1000U
#define CLOSED_LOOP_SAMPLE_COUNT        3U
#define CLOSED_LOOP_MAX_CORRECTIONS     2U
#define CLOSED_LOOP_GAIN                0.70f
#define CLOSED_LOOP_MAX_STEP_CM         5.0f

#endif /* __PLATFORM_CONFIG_H */
