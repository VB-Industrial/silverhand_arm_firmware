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

bool EncoderCalibration::begin_backlash(
    const uint32_t now_ms,
    const bool encoder_available,
    const uint16_t raw)
{
    if (!encoder_available || blocks_normal_control() || !has_stored_data_) {
        return false;
    }

    int32_t current_position_ticks = 0;
    if (!locate_stored_position(raw, current_position_ticks) ||
        !prepare_backlash_rock(current_position_ticks)) {
        return false;
    }

    previous_raw_ = raw;
    position_ticks_ = current_position_ticks;
    previous_position_ticks_ = current_position_ticks;
    manual_total_travel_ = 0;
    error_ = EncoderCalibrationError::None;
    state_started_ms_ = now_ms;
    state_ = EncoderCalibrationState::MoveToRockStart;
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
         (state_ == EncoderCalibrationState::ReverseSweepToA) ||
         (state_ == EncoderCalibrationState::MoveToRockStart) ||
         (state_ == EncoderCalibrationState::RockSweep) ||
         (state_ == EncoderCalibrationState::MoveToMiddle) ||
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
        (state_ == EncoderCalibrationState::SweepToB) ||
        (state_ == EncoderCalibrationState::SettleAtB) ||
        (state_ == EncoderCalibrationState::ReverseSweepToA) ||
        (state_ == EncoderCalibrationState::MoveToRockStart) ||
        (state_ == EncoderCalibrationState::RockSettle) ||
        (state_ == EncoderCalibrationState::RockSweep) ||
        (state_ == EncoderCalibrationState::MoveToMiddle) ||
        (state_ == EncoderCalibrationState::SettleAtMiddle)) {
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
            reverse_tmc_samples_.fill(0);
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
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::SettleAtB;
        }
        break;
    case EncoderCalibrationState::SettleAtB:
        action.command_velocity = true;
        if ((now_ms - state_started_ms_) >= kSettleTimeMs) {
            previous_position_ticks_ = position_ticks_;
            next_point_ = data_.point_count - 1U;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::ReverseSweepToA;
        }
        break;
    case EncoderCalibrationState::ReverseSweepToA:
        action.command_velocity = true;
        action.velocity_steps = velocity_toward(safe_start_ticks_);
        while ((next_point_ < data_.point_count) &&
               reached(point_target(next_point_), previous_position_ticks_, position_ticks_)) {
            reverse_tmc_samples_[next_point_] = tmc_position_steps;
            if (next_point_ == 0U) {
                next_point_ = data_.point_count;
            } else {
                --next_point_;
            }
        }
        if (reached(safe_start_ticks_, previous_position_ticks_, position_ticks_)) {
            action.velocity_steps = 0;
            reverse_tmc_samples_[0] = tmc_position_steps;
            state_ = EncoderCalibrationState::Processing;
        }
        break;
    case EncoderCalibrationState::Processing: {
        action.command_velocity = true;
        const int32_t middle_ticks = safe_start_ticks_ + ((safe_end_ticks_ - safe_start_ticks_) / 2);
        if (!process_table() || !prepare_backlash_rock(middle_ticks)) {
            fail(EncoderCalibrationError::InvalidTable);
            action.disable_driver = true;
        } else {
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::MoveToRockStart;
        }
        break;
    }
    case EncoderCalibrationState::MoveToRockStart:
        action.command_velocity = true;
        action.velocity_steps = velocity_toward(rock_high_ticks_);
        if (reached(rock_high_ticks_, previous_position_ticks_, position_ticks_)) {
            action.velocity_steps = 0;
            rock_at_high_ = true;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::RockSettle;
        }
        break;
    case EncoderCalibrationState::RockSettle:
        action.command_velocity = true;
        if ((now_ms - state_started_ms_) >= kSettleTimeMs) {
            if (backlash_sample_count_ >= kBacklashSampleCount) {
                if (!finalize_backlash()) {
                    fail(EncoderCalibrationError::InvalidTable);
                    action.disable_driver = true;
                } else {
                    state_started_ms_ = now_ms;
                    state_ = EncoderCalibrationState::MoveToMiddle;
                }
            } else {
                rock_start_position_ticks_ = position_ticks_;
                rock_start_tmc_steps_ = tmc_position_steps;
                rock_target_ticks_ = rock_at_high_ ? rock_low_ticks_ : rock_high_ticks_;
                state_started_ms_ = now_ms;
                state_ = EncoderCalibrationState::RockSweep;
            }
        }
        break;
    case EncoderCalibrationState::RockSweep:
        action.command_velocity = true;
        action.velocity_steps = velocity_toward(rock_target_ticks_);
        if (reached(rock_target_ticks_, previous_position_ticks_, position_ticks_)) {
            action.velocity_steps = 0;
            if (!record_backlash_sample(position_ticks_, tmc_position_steps)) {
                fail(EncoderCalibrationError::InvalidTable);
                action.disable_driver = true;
            } else {
                rock_at_high_ = !rock_at_high_;
                state_started_ms_ = now_ms;
                state_ = EncoderCalibrationState::RockSettle;
            }
        }
        break;
    case EncoderCalibrationState::MoveToMiddle:
        action.command_velocity = true;
        action.velocity_steps = velocity_toward(rock_middle_ticks_);
        if (reached(rock_middle_ticks_, previous_position_ticks_, position_ticks_)) {
            action.velocity_steps = 0;
            state_started_ms_ = now_ms;
            state_ = EncoderCalibrationState::SettleAtMiddle;
        }
        break;
    case EncoderCalibrationState::SettleAtMiddle:
        action.command_velocity = true;
        if ((now_ms - state_started_ms_) >= kSettleTimeMs) {
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
    if (count < 2U) {
        return false;
    }
    const int32_t forward_span = tmc_samples_[count - 1U] - tmc_samples_[0];
    const int32_t reverse_span = reverse_tmc_samples_[count - 1U] - reverse_tmc_samples_[0];
    const int32_t raw_span = safe_end_ticks_ - safe_start_ticks_;
    if ((forward_span == 0) || (reverse_span == 0) || (raw_span == 0) ||
        ((forward_span > 0) != (reverse_span > 0))) {
        return false;
    }

    const int64_t span_difference = std::abs(
        static_cast<int64_t>(forward_span) - static_cast<int64_t>(reverse_span));
    const int64_t larger_span = std::max(
        std::abs(static_cast<int64_t>(forward_span)),
        std::abs(static_cast<int64_t>(reverse_span)));
    if ((span_difference * 20) > larger_span) {
        return false;
    }

    data_.tmc_span_steps = static_cast<int32_t>(
        (static_cast<int64_t>(forward_span) + static_cast<int64_t>(reverse_span)) / 2);
    std::array<int16_t, ENCODER_CALIBRATION_MAX_POINTS> raw_corrections{};
    for (uint16_t i = 0U; i < count; ++i) {
        const int64_t actual = (static_cast<int64_t>(raw_span) * i) / (count - 1U);
        const int64_t forward_ideal =
            (static_cast<int64_t>(tmc_samples_[i] - tmc_samples_[0]) * raw_span) / forward_span;
        const int64_t reverse_ideal =
            (static_cast<int64_t>(reverse_tmc_samples_[i] - reverse_tmc_samples_[0]) * raw_span) /
            reverse_span;
        const int64_t correction = ((forward_ideal + reverse_ideal) / 2) - actual;
        if ((correction < -kMaximumCorrectionTicks) || (correction > kMaximumCorrectionTicks)) {
            return false;
        }
        raw_corrections[i] = static_cast<int16_t>(correction);
        if ((i > 0U) &&
            (((forward_span > 0) &&
              ((tmc_samples_[i] <= tmc_samples_[i - 1U]) ||
               (reverse_tmc_samples_[i] <= reverse_tmc_samples_[i - 1U]))) ||
             ((forward_span < 0) &&
              ((tmc_samples_[i] >= tmc_samples_[i - 1U]) ||
               (reverse_tmc_samples_[i] >= reverse_tmc_samples_[i - 1U]))))) {
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

int32_t EncoderCalibration::corrected_position_ticks(const int32_t position_ticks) const
{
    const int32_t direction = (data_.manual_span_ticks >= 0) ? 1 : -1;
    const int32_t safe_start = direction * static_cast<int32_t>(data_.safe_margin_ticks);
    const int32_t safe_end = data_.manual_span_ticks - safe_start;
    const int32_t safe_low = std::min(safe_start, safe_end);
    const int32_t safe_high = std::max(safe_start, safe_end);
    const int32_t table_position = std::clamp(position_ticks, safe_low, safe_high);
    const int64_t span = std::abs(static_cast<int64_t>(safe_end) - safe_start);
    if ((span == 0) || (data_.point_count < 2U)) {
        return position_ticks;
    }
    const int64_t distance = std::abs(static_cast<int64_t>(table_position) - safe_start);
    const int64_t scaled = distance * (data_.point_count - 1U);
    const uint16_t index = static_cast<uint16_t>(std::min<int64_t>(scaled / span, data_.point_count - 1U));
    int32_t correction = data_.correction_ticks[index];
    if (index + 1U < data_.point_count) {
        const int64_t remainder = scaled % span;
        const int32_t difference =
            static_cast<int32_t>(data_.correction_ticks[index + 1U]) - data_.correction_ticks[index];
        correction += static_cast<int32_t>((static_cast<int64_t>(difference) * remainder) / span);
    }
    return position_ticks + correction;
}

bool EncoderCalibration::prepare_backlash_rock(const int32_t center_ticks)
{
    const int32_t safe_low = std::min(safe_start_ticks_, safe_end_ticks_);
    const int32_t safe_high = std::max(safe_start_ticks_, safe_end_ticks_);
    const int32_t available_half_range = std::min(
        center_ticks - safe_low,
        safe_high - center_ticks);
    const int32_t half_range = std::min(kBacklashRockHalfRangeTicks, available_half_range);
    if ((available_half_range <= 0) ||
        (half_range < kMinimumBacklashRockHalfRangeTicks) ||
        (data_.tmc_span_steps == 0)) {
        return false;
    }

    rock_middle_ticks_ = center_ticks;
    rock_low_ticks_ = rock_middle_ticks_ - half_range;
    rock_high_ticks_ = rock_middle_ticks_ + half_range;
    rock_target_ticks_ = rock_high_ticks_;
    backlash_sample_count_ = 0U;
    backlash_samples_.fill(0);
    data_.backlash_steps = 0;
    return true;
}

bool EncoderCalibration::locate_stored_position(const uint16_t raw, int32_t& position_ticks)
{
    const int32_t span = data_.manual_span_ticks;
    if (span == 0) {
        return false;
    }
    const int32_t direction = (span > 0) ? 1 : -1;
    safe_start_ticks_ = direction * static_cast<int32_t>(data_.safe_margin_ticks);
    safe_end_ticks_ = span - safe_start_ticks_;
    return locate_position_in_stored_span(raw, true, position_ticks);
}

bool EncoderCalibration::locate_position_in_stored_span(
    const uint16_t raw,
    const bool safe_only,
    int32_t& position_ticks) const
{
    const int32_t span = data_.manual_span_ticks;
    if (span == 0) {
        return false;
    }
    const int32_t direction = (span > 0) ? 1 : -1;
    const int32_t margin = safe_only ? direction * static_cast<int32_t>(data_.safe_margin_ticks) : 0;
    const int32_t range_start = margin;
    const int32_t range_end = span - margin;
    const int32_t tolerance = safe_only ? 0 : kHardLimitToleranceTicks;
    const int32_t range_low = std::min(range_start, range_end) - tolerance;
    const int32_t range_high = std::max(range_start, range_end) + tolerance;
    const int32_t middle = range_start + ((range_end - range_start) / 2);
    const int32_t base = static_cast<int32_t>(raw) - static_cast<int32_t>(data_.limit_a_raw);

    bool found = false;
    int32_t best_distance = std::numeric_limits<int32_t>::max();
    for (int32_t turn = -2; turn <= 2; ++turn) {
        const int32_t candidate = base + (turn * kEncoderTicksPerTurn);
        if ((candidate < range_low) || (candidate > range_high)) {
            continue;
        }
        const int32_t distance = std::abs(candidate - middle);
        if (!found || (distance < best_distance)) {
            position_ticks = candidate;
            best_distance = distance;
            found = true;
        }
    }
    return found;
}

bool EncoderCalibration::record_backlash_sample(
    const int32_t position_ticks,
    const int32_t tmc_position_steps)
{
    if (backlash_sample_count_ >= kBacklashSampleCount) {
        return false;
    }
    const int64_t encoder_delta = std::abs(
        static_cast<int64_t>(corrected_position_ticks(position_ticks)) -
        corrected_position_ticks(rock_start_position_ticks_));
    const int64_t tmc_delta = std::abs(
        static_cast<int64_t>(tmc_position_steps) - rock_start_tmc_steps_);
    const int64_t raw_span = std::abs(static_cast<int64_t>(safe_end_ticks_) - safe_start_ticks_);
    const int64_t tmc_span = std::abs(static_cast<int64_t>(data_.tmc_span_steps));
    if ((encoder_delta == 0) || (raw_span == 0)) {
        return false;
    }
    const int64_t expected_tmc_delta = ((encoder_delta * tmc_span) + (raw_span / 2)) / raw_span;
    const int64_t backlash = std::max<int64_t>(0, tmc_delta - expected_tmc_delta);
    if (backlash > std::numeric_limits<int32_t>::max()) {
        return false;
    }
    backlash_samples_[backlash_sample_count_] = static_cast<int32_t>(backlash);
    ++backlash_sample_count_;
    return true;
}

bool EncoderCalibration::finalize_backlash()
{
    if (backlash_sample_count_ != kBacklashSampleCount) {
        return false;
    }
    std::array<int32_t, kBacklashSampleCount - kBacklashDiscardedSamples> retained{};
    std::copy(
        backlash_samples_.begin() + kBacklashDiscardedSamples,
        backlash_samples_.end(),
        retained.begin());
    std::sort(retained.begin(), retained.end());
    const size_t middle = retained.size() / 2U;
    const int64_t median =
        (static_cast<int64_t>(retained[middle - 1U]) + retained[middle]) / 2;
    if (median <= 0) {
        return false;
    }
    data_.backlash_steps = static_cast<int32_t>(median);
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
        return static_cast<int32_t>((35U * next_point_) / data_.point_count);
    }
    if (state_ == EncoderCalibrationState::SettleAtB) {
        return 35;
    }
    if (state_ == EncoderCalibrationState::ReverseSweepToA) {
        const uint32_t completed = (next_point_ < data_.point_count)
            ? (data_.point_count - 1U - next_point_)
            : data_.point_count;
        return 35 + static_cast<int32_t>((35U * completed) / data_.point_count);
    }
    if ((state_ == EncoderCalibrationState::Processing) ||
        (state_ == EncoderCalibrationState::MoveToRockStart)) {
        return 70;
    }
    if ((state_ == EncoderCalibrationState::RockSettle) ||
        (state_ == EncoderCalibrationState::RockSweep)) {
        return 70 + static_cast<int32_t>((25U * backlash_sample_count_) / kBacklashSampleCount);
    }
    if ((state_ == EncoderCalibrationState::MoveToMiddle) ||
        (state_ == EncoderCalibrationState::SettleAtMiddle) ||
        (state_ == EncoderCalibrationState::Saving)) {
        return 95;
    }
    return (state_ == EncoderCalibrationState::Complete) ? 100 : 0;
}

bool EncoderCalibration::blocks_normal_control() const { return state_ != EncoderCalibrationState::Idle; }
const encoder_calibration_data& EncoderCalibration::data() const { return data_; }
bool EncoderCalibration::has_stored_data() const { return has_stored_data_; }
bool EncoderCalibration::calibrated_position_ticks(
    const uint16_t raw,
    int32_t& ticks_from_zero) const
{
    if (!has_stored_data_ || !zero_valid()) {
        return false;
    }
    int32_t position_ticks = 0;
    int32_t zero_ticks = 0;
    if (!locate_position_in_stored_span(raw, false, position_ticks) ||
        !locate_position_in_stored_span(data_.zero_raw, false, zero_ticks)) {
        return false;
    }
    ticks_from_zero = corrected_position_ticks(position_ticks) - corrected_position_ticks(zero_ticks);
    return true;
}
int32_t EncoderCalibration::manual_total_travel() const { return manual_total_travel_; }
bool EncoderCalibration::zero_valid() const { return data_.zero_valid != 0U; }
uint16_t EncoderCalibration::zero_raw() const { return data_.zero_raw; }
