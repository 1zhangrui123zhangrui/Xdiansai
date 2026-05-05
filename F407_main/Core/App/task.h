/**
 * @file    task.h
 * @brief   任务状态机
 *
 * 任务列表:
 *  HOME        - 回到中心 (激光→中心圆)
 *  AREA_PATROL - 区域巡逻 (四角橙色圆依次)
 *  SEQ_PATROL  - 顺序巡逻 (用户指定 5 个圆顺序)
 *  AUTO_PATROL - 自动巡逻 (蛇形, 兼顾火源检测)
 *  CALIBRATE   - 标定 (清零电机角度)
 *  MOTOR_ENABLE  - 电机使能/抱轴
 *  MOTOR_DISABLE - 停止并失能/松轴
 *  E_STOP      - 急停
 */
#ifndef __TASK_H
#define __TASK_H

#include <stdint.h>

typedef enum {
    TASK_IDLE,
    TASK_HOME,
    TASK_AREA_PATROL,
    TASK_SEQ_PATROL,
    TASK_AUTO_PATROL,
    TASK_CALIBRATE,
    TASK_E_STOP,
} TaskState_t;

/**
 * @brief  初始化任务模块
 *         必须在 Kinematics_Init / Motor_Init 之后调用
 */
void Task_Init(void);

/** 主循环中每次都调用 */
void Task_Tick(void);

/* ---- 任务触发接口 (由串口屏回调调用) ---- */
void Task_StartHome(void);
void Task_StartAreaPatrol(void);
void Task_StartSeqPatrol(const uint8_t seq[5]);
void Task_StartAutoPatrol(void);
void Task_StartCalibrate(void);
void Task_EnableMotors(void);
void Task_DisableMotors(void);
void Task_EStop(void);

/** 当前任务状态 */
TaskState_t Task_GetState(void);

/** 当前激光位置 (cm) — 供坐标上报使用 */
float Task_GetLaserX(void);
float Task_GetLaserY(void);

#endif /* __TASK_H */
