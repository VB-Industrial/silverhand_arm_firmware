#include "encoder_calibration.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace {
constexpr int32_t kEncoderTicksPerTurn = 16384;
constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;
constexpr int32_t kMaximumSpanTicks = 2 * kEncoderTicksPerTurn;
constexpr int32_t kMaximumManualTravelTicks = 4 * kEncoderTicksPerTurn;
constexpr int32_t kMinimumSpanTicks = 32;
constexpr int32_t kMaximumCorrectionTicks = 2048;
constexpr int32_t kHardLimitToleranceTicks = 16;
}

void EncoderCalibration::initialize(const uint8_t joint_id, const bool encoder_inverted, const uint16_t raw)
{
    joint_id_ = joint_id;
    encoder_inverted_ = encoder_inverted;
    previous_raw_ = raw;
    has_stored_data_ = encoder_calibration_storage_load(joint_id_, &data_);
}

bool EncoderCalibration::begin(const bool encoder_available, const uint16_t raw)
{
    if (!encoder_available || blocks_normal_control()) {
        return false;
    }
    previous_raw_ = raw;
    position_ticks_ = 0;
    previous_position_ticks_ = 0;
    manual_total_travel_ = 0;
    error_ = EncoderCalibrationError::None;
    state_ = EncoderCalibrationState::WaitLimitA;
    return true;
}

bool EncoderCalibration::begin_auto(const uint32_t now_ms, const bool encoder_available, const uint16_t raw)
{
    if (!encoder_available || blocks_normal_control()) {
        return false;
    }
    const uint16_t saved_zero_raw = data_.zero_raw;
    const uint8_t saved_zero_valid = data_.zero_valid;
    data_ = {};
    data_.zero_raw = saved_zero_raw;
    data_.zero_valid = saved_zero_valid;
    previous_raw_ = raw;
    position_ticks_ = 0;
    previous_position_ticks_ = 0;
    manual_total_travel_ = 0;
    error_ = EncoderCalibrationError::None;
    state_started_ms_ = now_ms;
    stall_anchor_ms_ = now_ms;
    stall_anchor_ticks_ = 0;
    state_ = EncoderCalibrationState::AutoSeekLimitA;
    return true;
}

bool EncoderCalibration::calibrate_zero(const uint16_t raw)
{
    if ((state_ != EncoderCalibrationState::Idle) || !has_stored_data_) {
        return false;
    }
    encoder_calibration_data updated = data_;
    updated.zero_raw = raw;
    updated.zero_valid = 1U;
    if (!encoder_calibration_storage_save(joint_id_, &updated)) {
        return false;
    }
    data_ = updated;
    has_stored_data_ = true;
    return true;
}

bool EncoderCalibration::advance(const uint32_t now_ms, const bool encoder_available, const uint16_t raw)
{
    if (!encoder_available) {
        fail(EncoderCalibrationError::EncoderUnavailable);
        return false;
    }
    observe_raw(raw);
    switch (state_) {
    case EncoderCalibrationState::WaitLimitA: {
        const uint16_t saved_zero_raw = data_.zero_raw;
        const uint8_t saved_zero_valid = data_.zero_valid;
        data_ = {};
        data_.zero_raw = saved_zero_raw;
        data_.zero_valid = saved_zero_valid;
        data_.limit_a_raw = raw;
        position_ticks_ = 0;
        previous_position_ticks_ = 0;
        manual_total_travel_ = 0;
        previous_raw_ = raw;
        state_ = EncoderCalibrationState::WaitLimitB;
        return true;
    }
    case EncoderCalibrationState::WaitLimitB: {
        const int32_t span = position_ticks_;
        data_.limit_b_raw = raw;
        data_.manual_span_ticks = span;
        if ((std::abs(span) < kMinimumSpanTicks) ||
            (std::abs(span) >= kMaximumSpanTicks) ||
            (manual_total_travel_ >= kMaximumManualTravelTicks)) {
            fail(EncoderCalibrationError::InvalidManualSpan);
            return false;
        }
        const int32_t span_abs = (span < 0) ? -span : span;
        const int32_t margin = std::clamp<int32_t>(span_abs / 50, 16, 200);
        data_.safe_margin_ticks = static_cast<uint16_t>(margin);
        const int32_t direction = (span > 0) ? 1 : -1;
        safe_start_ticks_ = direction * margin;
        safe_end_ticks_ = span - (direction * margin);
        const int32_t safe_span = std::abs(safe_end_ticks_ - safe_start_ticks_);
        data_.point_count = static_cast<uint16_t>(std::min<int32_t>(ENCODER_CALIBRATION_MAX_POINTS, safe_span + 1));
        state_ = EncoderCalibrationState::Ready;
        return true;
    }
    case EncoderCalibrationState::Ready:
        state_started_ms_ = now_ms;
        state_ = EncoderCalibrationState::MoveToA;
        return true;
    default:
        return false;
    }
}

void EncoderCalibration::abort()
{
    if (state_ != EncoderCalibrationState::Idle) {
        state_ = EncoderCalibrationState::Aborted;
    }
}

void EncoderCalibration::fail(const EncoderCalibrationError error)
{
    error_ = error;
    state_ = EncoderCalibrationState::Failed;
}

EncoderCalibration::Action EncoderCalibration::update(
    const uint32_t now_ms,
    const bool encoder_available,
    const uint16_t raw,
    const int32_t tmc_position_steps)
{
    Action action{};
    if (state_ == EncoderCalibrationState::Idle) {
        previous_raw_ = raw;
        return action;
    }
    if (!encoder_available) {
        fail(EncoderCalibrationError::EncoderUnavailable);
        action.command_velocity = true;
        action.disable_driver = true;
        return action;
    }

    previous_position_ticks_ = position_ticks_;
    observe_raw(raw);
    if ((state_ == EncoderCalibrationState::WaitLimitA) ||
        (state_ == EncoderCalibrationState::WaitLimitB) ||
        (state_ == EncoderCalibrationState::Ready)) {
        return action;
    }
    if (((state_ == EncoderCalibrationState::MoveToA) ||
         (state_ == EncoderCalibrationState::SweepToB) ||
         (state_ == EncoderCalibrationState::AutoSeekLimitA) ||
         (state_ == EncoderCalibrationState::AutoBackoffA) ||
         (state_ == EncoderCalibrationState::AutoSeekLimitB)) &&
        ((now_ms - state_started_ms_) > kMotionTimeoutMs)) {
        fail(EncoderCalibrationError::MotionTimeout);
        action.command_velocity = true;
        action.disable_driver = true;
        return action;
    }
    if ((state_ == EncoderCalibrationState::MoveToA) ||
        (state_ == EncoderCalibrationState::SettleAtA) ||
        (state_ == EncoderCalibrationState::SweepToB)) {
        const int32_t low = std::min<int32_t>(0, data_.manual_span_ticks) - kHardLimitToleranceTicks;
        const int32_t high = std::max<int32_t>(0, data_.manual_span_ticks) + kHardLimitToleranceTicks;
        if ((position_ticks_ < low) || (position_ticks_ > high)) {
            fail(EncoderCalibrationError::WrongDirection);
            action.command_velocity = true;
            action.disable_driver = true;
            return action;
        }
    }

    switch (state_) {
    case EncoderCalibrationState::AutoSeekLimitA:
        action.command_velocity = true;
        action.velocity_steps = kCalibrationVelocitySteps;
        if (auto_stall_detected(now_ms)) {
            action.velocity_steps = 0;
            data_.limit_a_raw = raw;
            position_ticks_ = 0;
            previous_position_ticks_ = 0;
            previous_raw_ = raw;
            stall_anchor_ticks_ = 0;
            stall_anchor_ms_ = now_ms;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::AutoBackoffA;
        }
        break;
    case EncoderCalibrationState::AutoBackoffA: {
        action.command_velocity = true;
        const int32_t raw_sign_for_positive_tmc = encoder_inverted_ ? -1 : 1;
        const int32_t target = -raw_sign_for_positive_tmc * 200;
        action.velocity_steps = -kCalibrationVelocitySteps;
        if (reached(target, previous_position_ticks_, position_ticks_)) {
            stall_anchor_ticks_ = position_ticks_;
            stall_anchor_ms_ = now_ms;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::AutoSeekLimitB;
        }
        break;
    }
    case EncoderCalibrationState::AutoSeekLimitB:
        action.command_velocity = true;
        action.velocity_steps = -kCalibrationVelocitySteps;
        if (std::abs(position_ticks_) >= kMaximumSpanTicks) {
            fail(EncoderCalibrationError::InvalidManualSpan);
            action.velocity_steps = 0;
            action.disable_driver = true;
        } else if (auto_stall_detected(now_ms)) {
            action.velocity_steps = 0;
            if (!prepare_automatic_span(raw)) {
                fail(EncoderCalibrationError::InvalidManualSpan);
                action.disable_driver = true;
            } else {
                state_started_ms_ = now_ms;
                state_ = EncoderCalibrationState::MoveToA;
            }
        }
        break;
    case EncoderCalibrationState::MoveToA:
        if (reached(safe_start_ticks_, previous_position_ticks_, position_ticks_)) {
            state_ = EncoderCalibrationState::SettleAtA;
            state_started_ms_ = now_ms;
            action.command_velocity = true;
        } else {
            action.command_velocity = true;
            action.velocity_steps = velocity_toward(safe_start_ticks_);
        }
        break;
    case EncoderCalibrationState::SettleAtA:
        action.command_velocity = true;
        if ((now_ms - state_started_ms_) >= kSettleTimeMs) {
            safe_start_ticks_ = position_ticks_;
            previous_position_ticks_ = position_ticks_;
            tmc_samples_.fill(0);
            tmc_samples_[0] = 0;
            next_point_ = 1U;
            action.zero_tmc_position = true;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::SweepToB;
        }
        break;
    case EncoderCalibrationState::SweepToB:
        action.command_velocity = true;
        action.velocity_steps = velocity_toward(safe_end_ticks_);
        while ((next_point_ < data_.point_count) &&
               reached(point_target(next_point_), previous_position_ticks_, position_ticks_)) {
            tmc_samples_[next_point_] = tmc_position_steps;
            ++next_point_;
        }
        if (reached(safe_end_ticks_, previous_position_ticks_, position_ticks_)) {
            action.velocity_steps = 0;
            tmc_samples_[data_.point_count - 1U] = tmc_position_steps;
            state_ = EncoderCalibrationState::Processing;
        }
        break;
    case EncoderCalibrationState::Processing:
        action.command_velocity = true;
        if (!process_table()) {
            fail(EncoderCalibrationError::InvalidTable);
            action.disable_driver = true;
        } else {
            state_ = EncoderCalibrationState::Saving;
        }
        break;
    case EncoderCalibrationState::Saving:
        action.command_velocity = true;
        if (!encoder_calibration_storage_save(joint_id_, &data_)) {
            fail(EncoderCalibrationError::Storage);
        } else {
            has_stored_data_ = true;
            state_ = EncoderCalibrationState::Complete;
        }
        action.disable_driver = true;
        break;
    case EncoderCalibrationState::Failed:
    case EncoderCalibrationState::Aborted:
        action.command_velocity = true;
        action.disable_driver = true;
        break;
    default:
        break;
    }
    return action;
}

int32_t EncoderCalibration::observe_raw(const uint16_t raw)
{
    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(previous_raw_);
    if (delta > kHalfTurnTicks) {
        delta -= kEncoderTicksPerTurn;
    } else if (delta < -kHalfTurnTicks) {
        delta += kEncoderTicksPerTurn;
    }
    previous_raw_ = raw;
    position_ticks_ += delta;
    if (state_ == EncoderCalibrationState::WaitLimitB) {
        manual_total_travel_ += std::abs(delta);
    }
    return delta;
}

int32_t EncoderCalibration::velocity_toward(const int32_t target) const
{
    const int32_t desired_raw_sign = (target >= position_ticks_) ? 1 : -1;
    const int32_t raw_sign_for_positive_tmc = encoder_inverted_ ? -1 : 1;
    return desired_raw_sign * raw_sign_for_positive_tmc * kCalibrationVelocitySteps;
}

bool EncoderCalibration::reached(const int32_t target, const int32_t previous, const int32_t current) const
{
    return ((previous <= target) && (current >= target)) || ((previous >= target) && (current <= target));
}

int32_t EncoderCalibration::point_target(const uint16_t index) const
{
    const int64_t span = static_cast<int64_t>(safe_end_ticks_) - safe_start_ticks_;
    return safe_start_ticks_ + static_cast<int32_t>((span * index) / (data_.point_count - 1U));
}

bool EncoderCalibration::process_table()
{
    const uint16_t count = data_.point_count;
    const int32_t tmc_span = tmc_samples_[count - 1U];
    const int32_t raw_span = safe_end_ticks_ - safe_start_ticks_;
    if ((count < 2U) || (tmc_span == 0) || (raw_span == 0)) {
        return false;
    }
    data_.tmc_span_steps = tmc_span;
    std::array<int16_t, ENCODER_CALIBRATION_MAX_POINTS> raw_corrections{};
    for (uint16_t i = 0U; i < count; ++i) {
        const int64_t actual = (static_cast<int64_t>(raw_span) * i) / (count - 1U);
        const int64_t ideal = (static_cast<int64_t>(tmc_samples_[i]) * raw_span) / tmc_span;
        const int64_t correction = ideal - actual;
        if ((correction < -kMaximumCorrectionTicks) || (correction > kMaximumCorrectionTicks)) {
            return false;
        }
        raw_corrections[i] = static_cast<int16_t>(correction);
        if ((i > 0U) &&
            (((tmc_span > 0) && (tmc_samples_[i] <= tmc_samples_[i - 1U])) ||
             ((tmc_span < 0) && (tmc_samples_[i] >= tmc_samples_[i - 1U])))) {
            return false;
        }
    }
    data_.correction_ticks[0] = raw_corrections[0];
    for (uint16_t i = 1U; i + 1U < count; ++i) {
        data_.correction_ticks[i] = static_cast<int16_t>(
            (static_cast<int32_t>(raw_corrections[i - 1U]) +
             (2 * static_cast<int32_t>(raw_corrections[i])) +
             static_cast<int32_t>(raw_corrections[i + 1U])) / 4);
    }
    data_.correction_ticks[count - 1U] = raw_corrections[count - 1U];
    const int32_t node_spacing = std::abs(raw_span) / (count - 1U);
    const int32_t raw_direction = (raw_span > 0) ? 1 : -1;
    for (uint16_t i = 1U; i < count; ++i) {
        const int32_t corrected_step = node_spacing +
            (raw_direction * (data_.correction_ticks[i] - data_.correction_ticks[i - 1U]));
        if (corrected_step <= 0) {
            return false;
        }
    }
    return true;
}

bool EncoderCalibration::auto_stall_detected(const uint32_t now_ms)
{
    if (std::abs(position_ticks_ - stall_anchor_ticks_) > kAutoStallMotionTicks) {
        stall_anchor_ticks_ = position_ticks_;
        stall_anchor_ms_ = now_ms;
    }
    return ((now_ms - state_started_ms_) >= kAutoStartupGraceMs) &&
           ((now_ms - stall_anchor_ms_) >= kAutoStallTimeMs);
}

bool EncoderCalibration::prepare_automatic_span(const uint16_t raw)
{
    const int32_t span = position_ticks_;
    const int32_t span_abs = (span < 0) ? -span : span;
    data_.limit_b_raw = raw;
    data_.manual_span_ticks = span;
    manual_total_travel_ = span_abs;
    if ((span_abs < kMinimumSpanTicks) || (span_abs >= kMaximumSpanTicks)) {
        return false;
    }
    const int32_t margin = std::clamp<int32_t>(span_abs / 50, 16, 200);
    data_.safe_margin_ticks = static_cast<uint16_t>(margin);
    const int32_t direction = (span > 0) ? 1 : -1;
    safe_start_ticks_ = direction * margin;
    safe_end_ticks_ = span - (direction * margin);
    const int32_t safe_span = std::abs(safe_end_ticks_ - safe_start_ticks_);
    data_.point_count = static_cast<uint16_t>(std::min<int32_t>(ENCODER_CALIBRATION_MAX_POINTS, safe_span + 1));
    return true;
}

EncoderCalibrationState EncoderCalibration::state() const { return state_; }
EncoderCalibrationError EncoderCalibration::error() const { return error_; }

int32_t EncoderCalibration::progress_percent() const
{
    if (state_ == EncoderCalibrationState::SweepToB) {
        return static_cast<int32_t>((100U * next_point_) / data_.point_count);
    }
    return (state_ == EncoderCalibrationState::Complete) ? 100 : 0;
}

bool EncoderCalibration::blocks_normal_control() const { return state_ != EncoderCalibrationState::Idle; }
const encoder_calibration_data& EncoderCalibration::data() const { return data_; }
bool EncoderCalibration::has_stored_data() const { return has_stored_data_; }
int32_t EncoderCalibration::manual_total_travel() const { return manual_total_travel_; }
bool EncoderCalibration::zero_valid() const { return data_.zero_valid != 0U; }
uint16_t EncoderCalibration::zero_raw() const { return data_.zero_raw; }
