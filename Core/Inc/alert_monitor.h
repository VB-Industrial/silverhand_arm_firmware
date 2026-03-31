#ifndef INC_ALERT_MONITOR_H_
#define INC_ALERT_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

class AlertMonitor
{
public:
    struct UpdateResult
    {
        bool stop_motion;
        bool sync_offset;
    };

    UpdateResult update(uint32_t now_ms, bool has_output_encoder, float encoder_angle_rad, float tmc_angle_rad);
    bool ack_fail();
    int32_t current_level() const;
    bool motion_allowed() const;

private:
    UpdateResult update_encoder_zero(bool has_output_encoder);
    UpdateResult update_slip_mismatch(uint32_t now_ms, float encoder_angle_rad, float tmc_angle_rad);

    int32_t slip_warning_level_ = 0;
    bool slip_fault_active_ = false;
    bool mismatch_active_ = false;
    uint32_t mismatch_start_ts_ms_ = 0U;
};

#endif /* INC_ALERT_MONITOR_H_ */
