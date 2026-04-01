#include "alert_monitor.h"
#include "utility.h"

constexpr float kSlipThresholdRad = 10.0F * static_cast<float>(M_PI) / 180.0F;
constexpr uint32_t kSlipRecoveryDurationMs = 3000U;
constexpr bool kEnableSlipAutoResync = false;

AlertMonitor::UpdateResult AlertMonitor::update(
    const uint32_t now_ms,
    const bool has_output_encoder,
    const float encoder_angle_rad,
    const float tmc_angle_rad)
{
    UpdateResult result = update_encoder_zero(has_output_encoder);
    if (!has_output_encoder) {
        return result;
    }

    const UpdateResult mismatch_result = update_slip_mismatch(now_ms, encoder_angle_rad, tmc_angle_rad);
    result.stop_motion = result.stop_motion || mismatch_result.stop_motion;
    result.sync_offset = result.sync_offset || mismatch_result.sync_offset;
    return result;
}

AlertMonitor::UpdateResult AlertMonitor::update_encoder_zero(const bool has_output_encoder)
{
    UpdateResult result{false, false};
    if (!has_output_encoder) {
        mismatch_active_ = false;
        mismatch_start_ts_ms_ = 0U;
    }
    return result;
}

AlertMonitor::UpdateResult AlertMonitor::update_slip_mismatch(
    const uint32_t now_ms,
    const float encoder_angle_rad,
    const float tmc_angle_rad)
{
    UpdateResult result{false, false};

    const float mismatch_rad = angular_abs_diff_radians(encoder_angle_rad, tmc_angle_rad);
    if (mismatch_rad <= kSlipThresholdRad) {
        mismatch_active_ = false;
        mismatch_start_ts_ms_ = 0U;
        return result;
    }

    if (!mismatch_active_) {
        mismatch_active_ = true;
        mismatch_start_ts_ms_ = now_ms;
        if (slip_warning_level_ < 3) {
            ++slip_warning_level_;
        }
        if (slip_warning_level_ >= 3) {
            slip_warning_level_ = 3;
            slip_fault_active_ = true;
            result.stop_motion = true;
        }
    }

    if (kEnableSlipAutoResync &&
        !slip_fault_active_ &&
        ((now_ms - mismatch_start_ts_ms_) >= kSlipRecoveryDurationMs)) {
        slip_warning_level_ = 0;
        mismatch_active_ = false;
        mismatch_start_ts_ms_ = 0U;
        result.sync_offset = true;
    }

    return result;
}

bool AlertMonitor::ack_fail()
{
    slip_fault_active_ = false;
    slip_warning_level_ = 0;
    mismatch_active_ = false;
    mismatch_start_ts_ms_ = 0U;
    return true;
}

int32_t AlertMonitor::current_level() const
{
    return slip_fault_active_ ? 3 : slip_warning_level_;
}

bool AlertMonitor::motion_allowed() const
{
    return !slip_fault_active_;
}
