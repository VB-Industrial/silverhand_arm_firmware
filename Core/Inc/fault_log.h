#ifndef INC_FAULT_LOG_H_
#define INC_FAULT_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fault_log_record {
    uint32_t magic;
    uint8_t version;
    uint8_t size;
    uint8_t joint_id;
    uint8_t valid;
    uint32_t sequence;
    uint32_t uptime_ms;
    uint32_t fault_mask;
    uint32_t tmc_gstat;
    uint32_t tmc_drv_status;
    uint32_t crc32;
} fault_log_record;

void fault_log_init(uint8_t joint_id);
bool fault_log_append(uint32_t uptime_ms, uint32_t fault_mask, uint32_t tmc_gstat, uint32_t tmc_drv_status);

bool fault_log_available(void);
uint32_t fault_log_count(void);
bool fault_log_get_last(fault_log_record* record);

#ifdef __cplusplus
}
#endif

#endif /* INC_FAULT_LOG_H_ */
