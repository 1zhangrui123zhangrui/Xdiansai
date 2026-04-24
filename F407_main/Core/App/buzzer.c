/**
 * @file    buzzer.c
 */
#include "buzzer.h"
#include "platform_config.h"

static uint8_t  s_pending_beeps   = 0;
static uint8_t  s_beep_phase      = 0;   /* 0=OFF, 1=ON */
static uint32_t s_last_tick       = 0;

void Buzzer_Init(void)
{
    /* GPIO 已由 CubeMX 初始化, 此处只确保关闭 */
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, BUZZER_OFF_LEVEL);
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, BUZZER_ON_LEVEL);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, BUZZER_OFF_LEVEL);
}

void Buzzer_Beep(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        Buzzer_On();
        HAL_Delay(BUZZER_BEEP_ON_MS);
        Buzzer_Off();
        if (i < times - 1) {
            HAL_Delay(BUZZER_BEEP_OFF_MS);
        }
    }
}

void Buzzer_BeepAsync(uint8_t times)
{
    s_pending_beeps = times;
    s_beep_phase    = 1;
    s_last_tick     = HAL_GetTick();
    Buzzer_On();
}

void Buzzer_Tick(void)
{
    if (s_pending_beeps == 0) return;

    uint32_t now  = HAL_GetTick();
    uint32_t dur  = (s_beep_phase == 1) ? BUZZER_BEEP_ON_MS : BUZZER_BEEP_OFF_MS;

    if (now - s_last_tick < dur) return;

    s_last_tick = now;

    if (s_beep_phase == 1) {
        /* 结束一声 ON */
        Buzzer_Off();
        s_pending_beeps--;
        if (s_pending_beeps > 0) {
            s_beep_phase = 0;   /* 进入 OFF 间隔 */
        }
        /* beeps==0 时停止, 保持 OFF */
    } else {
        /* 结束 OFF 间隔, 开始下一声 */
        s_beep_phase = 1;
        Buzzer_On();
    }
}
