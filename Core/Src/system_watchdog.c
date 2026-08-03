#include "system_watchdog.h"

#include "main.h"

enum {
    SYSTEM_IWDG_KEY_ENABLE = 0xCCCCU,
    SYSTEM_IWDG_KEY_RELOAD = 0xAAAAU,
    SYSTEM_IWDG_KEY_WRITE_ACCESS = 0x5555U,
    SYSTEM_IWDG_PRESCALER_256 = 6U,
    SYSTEM_IWDG_RELOAD_2_SECONDS = 249U,
    SYSTEM_IWDG_MAX_WINDOW = 0x0FFFU,
    SYSTEM_IWDG_UPDATE_TIMEOUT_MS = 100U,
};

static uint32_t reset_reason = SYSTEM_RESET_REASON_NONE;
static bool watchdog_started = false;

void system_watchdog_capture_reset_reason(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_PIN;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_BROWNOUT;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_SOFTWARE;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_IWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_WWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_LOW_POWER;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) != RESET) {
        reset_reason |= SYSTEM_RESET_REASON_OPTION_BYTES;
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

bool system_watchdog_start(void)
{
    __HAL_DBGMCU_FREEZE_IWDG();

    IWDG->KR = SYSTEM_IWDG_KEY_ENABLE;
    IWDG->KR = SYSTEM_IWDG_KEY_WRITE_ACCESS;
    IWDG->PR = SYSTEM_IWDG_PRESCALER_256;
    IWDG->RLR = SYSTEM_IWDG_RELOAD_2_SECONDS;
    IWDG->WINR = SYSTEM_IWDG_MAX_WINDOW;

    const uint32_t started_ms = HAL_GetTick();
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)) != 0U) {
        if ((HAL_GetTick() - started_ms) >= SYSTEM_IWDG_UPDATE_TIMEOUT_MS) {
            return false;
        }
    }

    IWDG->KR = SYSTEM_IWDG_KEY_RELOAD;
    watchdog_started = true;
    return true;
}

void system_watchdog_refresh(void)
{
    if (watchdog_started) {
        IWDG->KR = SYSTEM_IWDG_KEY_RELOAD;
    }
}

uint32_t system_watchdog_reset_reason(void)
{
    return reset_reason;
}
