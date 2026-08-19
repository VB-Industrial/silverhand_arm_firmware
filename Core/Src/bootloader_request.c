#include "bootloader_request.h"

#include "fdcan.h"
#include "main.h"
#include "motor.h"
#include "robot_config.h"

static const uint32_t BOOT_REQUEST_MAGIC = 0xB00710ADUL;
enum {
    BOOT_CFG_NOMINAL_SHIFT = 11U,
    BOOT_CFG_DATA_SHIFT = 19U,
    BOOT_CFG_FD_MODE_BIT = 27U,
    BOOT_CFG_BITRATE_SWITCH_BIT = 28U,
};

static volatile bool request_pending;
static uint32_t request_scheduled_ms;

void bootloader_request_schedule(void)
{
    request_pending = true;
    request_scheduled_ms = HAL_GetTick();
}

bool bootloader_request_pending(void)
{
    return request_pending;
}

void bootloader_request_process(void)
{
    if (!request_pending || ((HAL_GetTick() - request_scheduled_ms) < 100U)) {
        return;
    }
    request_pending = false;

    (void)motor_arm(false);
    (void)HAL_FDCAN_Stop(&hfdcan1);

    const uint32_t packed_config =
        (uint32_t)kRobotJointProfile->node_id |
        (1UL << BOOT_CFG_NOMINAL_SHIFT) |
        (1UL << BOOT_CFG_DATA_SHIFT) |
        (1UL << BOOT_CFG_FD_MODE_BIT) |
        (1UL << BOOT_CFG_BITRATE_SWITCH_BIT);

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    TAMP->BKP1R = packed_config;
    TAMP->BKP2R = 0U;
    HAL_PWR_DisableBkUpAccess();

    HAL_Delay(20U);
    NVIC_SystemReset();
}
