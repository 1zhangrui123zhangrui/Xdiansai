/**
 * @file    nrf_app.h
 * @brief   NRF24L01 应用层: 接收 F103 视觉模块数据
 *
 * 数据包格式 (6 字节, F103→F407):
 *  Byte 0:   flags (bit0=检测到火源, bit1=位置数据有效)
 *  Byte 1:   序列号 (调试用)
 *  Byte 2-3: dx (int16, 0.1mm, 摄像头偏离指令位置的 X 误差)
 *  Byte 4-5: dy (int16, 0.1mm)
 *
 * 注: 火源坐标由 F407 直接用当前激光坐标记录, 无需 F103 发送.
 */
#ifndef __NRF_APP_H
#define __NRF_APP_H

#include <stdint.h>

#define NRF_FLAG_FIRE       0x01U
#define NRF_FLAG_POS_VALID  0x02U

#pragma pack(push, 1)
typedef struct {
    uint8_t  flags;
    uint8_t  seq;
    int16_t  dx_0p1mm;   /* 摄像头偏离指令中心的 X 误差 (0.1mm) */
    int16_t  dy_0p1mm;   /* Y 误差 (0.1mm) */
} NRF_CamData_t;   /* 6 bytes */
#pragma pack(pop)

/** 初始化 NRF, 设置为接收模式 */
void NrfApp_Init(void);

/**
 * @brief  主循环中轮询 NRF, 若收到数据则解包
 * @return 1 = 收到新帧, 0 = 无新数据
 */
uint8_t NrfApp_Poll(void);

/** 获取最新一帧摄像头数据 */
const NRF_CamData_t *NrfApp_GetData(void);

/** 判断最近一帧是否包含火源标志 */
uint8_t NrfApp_IsFire(void);

/** 获取摄像头位置误差 (cm) */
float NrfApp_GetDxCm(void);
float NrfApp_GetDyCm(void);

#endif /* __NRF_APP_H */
