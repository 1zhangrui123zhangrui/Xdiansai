/**
 * @file    nrf_app.h
 * @brief   NRF24L01 应用层: 接收 F103 视觉模块数据
 *
 * 数据包格式 (20 字节, F103→F407):
 *  Byte 0:    0xA5
 *  Byte 1:    flags
 *  Byte 2:    seq
 *  Byte 3-6:  platform_x/y_mm (int16 little-endian)
 *  Byte 7-14: fire1_x/y_mm, fire2_x/y_mm (int16 little-endian)
 *  Byte 15:   fire_count
 *  Byte 16-17: fire scores (0-100)
 *  Byte 18:   xor checksum over byte 0..17
 *  Byte 19:   0x5A
 */
#ifndef __NRF_APP_H
#define __NRF_APP_H

#include <stdint.h>

#define NRF_FRAME_HEAD      0xA5U
#define NRF_FRAME_TAIL      0x5AU
#define NRF_FLAG_FIRE       0x01U
#define NRF_FLAG_POS_VALID  0x02U
#define NRF_FLAG_POS_EST    0x04U
#define NRF_FLAG_FIRE1_VALID 0x08U
#define NRF_FLAG_FIRE2_VALID 0x10U

#pragma pack(push, 1)
typedef struct {
    uint8_t  head;
    uint8_t  flags;
    uint8_t  seq;
    int16_t  platform_x_mm;
    int16_t  platform_y_mm;
    int16_t  fire_x_mm[2];
    int16_t  fire_y_mm[2];
    uint8_t  fire_count;
    uint8_t  fire_score[2];
    uint8_t  checksum;
    uint8_t  tail;
} NRF_CamData_t;   /* 20 bytes */
#pragma pack(pop)

/** 初始化 NRF, 设置为接收模式 */
void NrfApp_Init(void);
uint8_t NrfApp_Check(void);

/**
 * @brief  主循环中轮询 NRF, 若收到数据则解包
 * @return 1 = 收到新帧, 0 = 无新数据
 */
uint8_t NrfApp_Poll(void);

/** 获取最新一帧摄像头数据 */
const NRF_CamData_t *NrfApp_GetData(void);

/** 判断最近一帧是否包含火源标志 */
uint8_t NrfApp_IsFire(void);

/** 获取视觉定位/火源世界坐标 (cm) */
uint8_t NrfApp_HasValidPosition(void);
float NrfApp_GetPlatformXCm(void);
float NrfApp_GetPlatformYCm(void);
uint8_t NrfApp_GetFireCount(void);
uint8_t NrfApp_GetFireCm(uint8_t index, float *x_cm, float *y_cm);

#endif /* __NRF_APP_H */
