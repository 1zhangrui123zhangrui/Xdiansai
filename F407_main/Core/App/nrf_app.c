/**
 * @file    nrf_app.c
 */
#include "nrf_app.h"
#include "24l01.h"
#include <string.h>

static NRF_CamData_t s_data = {0};
static uint8_t       s_fire_flag = 0;

void NrfApp_Init(void)
{
    NRF24L01_Init();
    NRF24L01_RX_Mode();
}

uint8_t NrfApp_Poll(void)
{
    if (!NRF24L01_Receive()) return 0;

    /* NRF24L01_RxPacket 已更新为 8 字节 */
    memcpy(&s_data, NRF24L01_RxPacket, sizeof(NRF_CamData_t));

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

float NrfApp_GetDxCm(void)
{
    return (float)s_data.dx_0p1mm * 0.01f;   /* 0.1mm → cm */
}

float NrfApp_GetDyCm(void)
{
    return (float)s_data.dy_0p1mm * 0.01f;
}
