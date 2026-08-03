#ifndef INC_SYSTEM_WATCHDOG_H_
#define INC_SYSTEM_WATCHDOG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum system_reset_reason {
    SYSTEM_RESET_REASON_NONE = 0U,
    SYSTEM_RESET_REASON_PIN = 1UL << 0U,
    SYSTEM_RESET_REASON_BROWNOUT = 1UL << 1U,
    SYSTEM_RESET_REASON_SOFTWARE = 1UL << 2U,
    SYSTEM_RESET_REASON_IWDG = 1UL << 3U,
    SYSTEM_RESET_REASON_WWDG = 1UL << 4U,
    SYSTEM_RESET_REASON_LOW_POWER = 1UL << 5U,
    SYSTEM_RESET_REASON_OPTION_BYTES = 1UL << 6U,
};

void system_watchdog_capture_reset_reason(void);
bool system_watchdog_start(void);
void system_watchdog_refresh(void);
uint32_t system_watchdog_reset_reason(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSTEM_WATCHDOG_H_ */
