/**
 * @file    nrf_app.c
 */
#include "nrf_app.h"
#include "24l01.h"
#include <string.h>

static NRF_CamData_t s_data = {0};
static uint8_t       s_fire_flag = 0;

static uint8_t checksum_xor(const uint8_t *data, uint8_t len)
{
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) {
        c ^= data[i];
    }
    return c;
}

void NrfApp_Init(void)
{
    NRF24L01_Init();
    HAL_Delay(100);
    NRF24L01_Check();
    NRF24L01_RX_Mode();
}

uint8_t NrfApp_Check(void)
{
    NRF24L01_Init();
    HAL_Delay(100);
    return NRF24L01_Check();
}

uint8_t NrfApp_Poll(void)
{
    if (!NRF24L01_Receive()) return 0;

    NRF_CamData_t frame;
    memcpy(&frame, NRF24L01_RxPacket, sizeof(frame));

    if (frame.head != NRF_FRAME_HEAD || frame.tail != NRF_FRAME_TAIL) {
        return 0;
    }
    if (checksum_xor(NRF24L01_RxPacket, 18) != frame.checksum) {
        return 0;
    }

    s_data = frame;

    if (s_data.flags & NRF_FLAG_FIRE) {
        s_fire_flag = 1;
    }
    return 1;
}

const NRF_CamData_t *NrfApp_GetData(void)
{
    return &s_data;
}

uint8_t NrfApp_IsFire(void)
{
    uint8_t f = s_fire_flag;
    s_fire_flag = 0;   /* 读一次后清除, 由上层决定何时再次采样 */
    return f;
}

uint8_t NrfApp_HasValidPosition(void)
{
    return (s_data.flags & NRF_FLAG_POS_VALID) ? 1U : 0U;
}

float NrfApp_GetPlatformXCm(void)
{
    return (float)s_data.platform_x_mm * 0.1f;
}

float NrfApp_GetPlatformYCm(void)
{
    return (float)s_data.platform_y_mm * 0.1f;
}

uint8_t NrfApp_GetFireCount(void)
{
    return (s_data.fire_count > 2U) ? 2U : s_data.fire_count;
}

uint8_t NrfApp_GetFireCm(uint8_t index, float *x_cm, float *y_cm)
{
    if (index >= NrfApp_GetFireCount() || x_cm == NULL || y_cm == NULL) {
        return 0;
    }

    if (index == 0U && !(s_data.flags & NRF_FLAG_FIRE1_VALID)) {
        return 0;
    }
    if (index == 1U && !(s_data.flags & NRF_FLAG_FIRE2_VALID)) {
        return 0;
    }

    *x_cm = (float)s_data.fire_x_mm[index] * 0.1f;
    *y_cm = (float)s_data.fire_y_mm[index] * 0.1f;
    return 1;
}
