#include "motor.h"

#include "main.h"
#include "robot_config.h"

#include <algorithm>
#include <array>
#include <cmath>

#include "encoder_calibration.hpp"
#include "fault_manager.hpp"
#include "tmc5160_state.hpp"

extern "C" {
#include "as50xx.h"
#include "tmc5160.h"
#include "utility.h"
}

namespace
{
constexpr uint32_t kDegradedLedTogglePeriodMs = 100U;
constexpr uint8_t kEncoderErrorStreakThreshold = 3U;
constexpr uint8_t kEncoderValidStreakThreshold = 10U;
constexpr float kEncoderRadiansPerTick =
    (2.0F * static_cast<float>(M_PI)) / static_cast<float>(_ENCODER_READMASK + 1);
constexpr uint32_t kFilterReferencePeriodMs = 20U;
constexpr size_t frames_for_ms(const uint32_t duration_ms)
{
    return (duration_ms + MOTOR_UPDATE_PERIOD_MS - 1U) / MOTOR_UPDATE_PERIOD_MS;
}
constexpr float kFusionSpikeThresholdRad = 24.0F * kEncoderRadiansPerTick;
constexpr float kFusionSpikeConsistencyRad = 24.0F * kEncoderRadiansPerTick;
constexpr float kExternalMotionThresholdRad = 24.0F * kEncoderRadiansPerTick;
constexpr uint8_t kFusionSpikeConfirmationFrames =
    static_cast<uint8_t>(frames_for_ms(60U));
constexpr float kFusionMaximumEncoderRateRadS = 3.0F;
constexpr float kFusionEncoderRateMarginRad = 4.0F * kEncoderRadiansPerTick;
constexpr size_t kSlipWindowSamples = frames_for_ms(1500U);
constexpr uint8_t kSlipPersistentWindows =
    static_cast<uint8_t>(frames_for_ms(60U));
constexpr float kSlipBacklashMultiplier = 10.0F;
constexpr float kBacklashLockMotionRad = 8.0F * kEncoderRadiansPerTick;
constexpr uint8_t kBacklashLockConfirmationFrames =
    static_cast<uint8_t>(frames_for_ms(160U));
constexpr uint8_t kStateConfirmationFrames =
    static_cast<uint8_t>(frames_for_ms(60U));
constexpr float kBacklashMismatchMarginRad = 8.0F * kEncoderRadiansPerTick;
constexpr float kManualEncoderTimeConstantMs = 100.0F;
constexpr uint32_t kManualQuietWindowMs = 500U;
constexpr float kManualQuietSpanRad = 24.0F * kEncoderRadiansPerTick;
constexpr uint32_t kServoCommandTimeoutMs = 1000U;
constexpr float kServoKp = 4.0F;
constexpr float kServoMaximumCorrectionVelocityRadS =
    15.0F * static_cast<float>(M_PI) / 180.0F;
constexpr float kPositionRegisterAccelerationRadS2 = 1.0F;
constexpr float kLimitAssumedDecelerationRadS2 = 1.0F;
constexpr float kLimitReactionTimeS = 0.02F;
constexpr float kServoStopToleranceRad = 0.01F * static_cast<float>(M_PI) / 180.0F;
constexpr float kServoResumeToleranceRad = 0.03F * static_cast<float>(M_PI) / 180.0F;
constexpr uint32_t kServiceStopTimeoutMs = 250U;

uint16_t g_encoder_angle_raw = 0U;
uint32_t g_zero_enc_runtime = 0U;
uint32_t g_prev_fusion_ts_ms = 0U;
uint32_t g_last_degraded_led_toggle_ms = 0U;
uint8_t g_encoder_error_streak = 0U;
uint8_t g_encoder_valid_streak = 0U;
float g_fused_angle_rad = 0.0F;
float g_fused_velocity_rad_s = 0.0F;
float g_startup_tmc_angle_offset_rad = 0.0F;
float g_fusion_encoder_angle_rad = 0.0F;
float g_fusion_tmc_angle_rad = 0.0F;
float g_fusion_offset_rad = 0.0F;
float g_fusion_encoder_error_rad = 0.0F;
float g_fusion_backlash_rad = 0.0F;
float g_fusion_innovation_rad = 0.0F;
float g_fusion_applied_correction_rad = 0.0F;
float g_slip_window_residual_rad = 0.0F;
float g_slip_pending_tmc_delta_rad = 0.0F;
float g_slip_previous_encoder_angle_rad = 0.0F;
float g_previous_observed_encoder_angle_rad = 0.0F;
float g_spike_candidate_innovation_rad = 0.0F;
float g_stationary_encoder_min_rad = 0.0F;
float g_stationary_encoder_max_rad = 0.0F;
float g_manual_encoder_window_min_rad = 0.0F;
float g_manual_encoder_window_max_rad = 0.0F;
float g_hybrid_takeup_tmc_travel_rad = 0.0F;
float g_hybrid_takeup_encoder_travel_rad = 0.0F;
float g_hybrid_lock_offset_sum_rad = 0.0F;
float g_fusion_encoder_weight = 1.0F;
float g_servo_target_position_rad = 0.0F;
float g_servo_position_error_rad = 0.0F;
float g_servo_command_velocity_rad_s = 0.0F;
float g_servo_maximum_correction_velocity_rad_s = kServoMaximumCorrectionVelocityRadS;
float g_servo_tracking_velocity_rad_s = 0.0F;
float g_direct_requested_velocity_rad_s = 0.0F;
int32_t g_prev_fusion_tmc_steps = 0;
int32_t g_encoder_raw_unwrapped = 0;
int32_t g_encoder_raw_unwrapped_min = 0;
int32_t g_encoder_raw_unwrapped_max = 0;
int32_t g_encoder_maximum_frame_delta = 0;
int32_t g_servo_tracking_velocity_steps = -1;
int32_t g_servo_acceleration_steps = -1;
int32_t g_direct_applied_velocity_steps = 0;
uint32_t g_servo_last_command_ms = 0U;
uint32_t g_calibration_jog_last_command_ms = 0U;
uint32_t g_manual_encoder_window_started_ms = 0U;
uint32_t g_stationary_encoder_window_started_ms = 0U;
uint32_t g_fusion_rejected_spike_count = 0U;
uint32_t g_slip_persistent_residual_count = 0U;
uint8_t g_spike_candidate_frames = 0U;
uint8_t g_slip_persistent_windows = 0U;
uint8_t g_hybrid_lock_confirmation_frames = 0U;
uint8_t g_motion_mismatch_confirmation_frames = 0U;
uint8_t g_external_motion_confirmation_frames = 0U;
size_t g_slip_window_index = 0U;
size_t g_slip_window_count = 0U;
bool g_output_encoder_available = false;
bool g_output_encoder_degraded = false;
bool g_encoder_last_read_ok = false;
bool g_fusion_initialized = false;
bool g_fusion_uses_calibrated_encoder = false;
bool g_servo_target_valid = false;
bool g_servo_at_target = false;
bool g_servo_requires_controller = false;
bool g_servo_service_position_tracking = false;
bool g_custom_motion_profile_active = false;
bool g_direct_applied_velocity_valid = false;
bool g_calibration_jog_active = false;
bool g_encoder_raw_statistics_initialized = false;
bool g_spike_gate_open = false;
bool g_stationary_encoder_reference_valid = false;
bool g_previous_driver_enabled = false;
bool g_slip_candidate = false;
int8_t g_hybrid_motion_direction = 0;
motor_hybrid_state g_hybrid_state = MOTOR_HYBRID_STATE_UNKNOWN;
std::array<float, kSlipWindowSamples> g_slip_residual_window{};
motor_servo_state g_servo_state = MOTOR_SERVO_STATE_INACTIVE;
motor_control_mode g_control_mode = MOTOR_CONTROL_MODE_HOLD;

FaultManager g_fault_manager;
Tmc5160StateMachine g_tmc_driver;
EncoderCalibration g_encoder_calibration;

struct JointLimitEnvelope {
    float hard_lower_rad = 0.0F;
    float soft_lower_rad = 0.0F;
    float soft_upper_rad = 0.0F;
    float hard_upper_rad = 0.0F;
    bool active = false;
};

float manipulator_to_native_radians(const float manipulator_angle_rad)
{
    return static_cast<float>(kRobotJointProfile->direction) * manipulator_angle_rad;
}

float native_to_manipulator_radians(const float native_angle_rad)
{
    return static_cast<float>(kRobotJointProfile->direction) * native_angle_rad;
}

int32_t apply_output_encoder_direction(const int32_t ticks)
{
    return (kRobotJointProfile->output_encoder_inverted != 0) ? -ticks : ticks;
}

float raw_encoder_angle_radians(void)
{
    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;

    int32_t delta_ticks = static_cast<int32_t>(g_encoder_angle_raw) - static_cast<int32_t>(g_zero_enc_runtime);
    if (delta_ticks > kHalfTurnTicks) {
        delta_ticks -= kEncoderTicksPerTurn;
    } else if (delta_ticks < -kHalfTurnTicks) {
        delta_ticks += kEncoderTicksPerTurn;
    }
    delta_ticks = apply_output_encoder_direction(delta_ticks);

    const float radians_per_tick = (2.0F * static_cast<float>(M_PI)) / static_cast<float>(kEncoderTicksPerTurn);
    return static_cast<float>(delta_ticks) * radians_per_tick;
}

void update_encoder_raw_statistics(const uint16_t raw)
{
    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;
    static uint16_t previous_raw = 0U;

    if (!g_encoder_raw_statistics_initialized) {
        previous_raw = raw;
        g_encoder_raw_unwrapped = static_cast<int32_t>(raw);
        g_encoder_raw_unwrapped_min = g_encoder_raw_unwrapped;
        g_encoder_raw_unwrapped_max = g_encoder_raw_unwrapped;
        g_encoder_maximum_frame_delta = 0;
        g_encoder_raw_statistics_initialized = true;
        return;
    }

    int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(previous_raw);
    if (delta > kHalfTurnTicks) {
        delta -= kEncoderTicksPerTurn;
    } else if (delta < -kHalfTurnTicks) {
        delta += kEncoderTicksPerTurn;
    }
    previous_raw = raw;
    g_encoder_raw_unwrapped += delta;
    g_encoder_raw_unwrapped_min = std::min(g_encoder_raw_unwrapped_min, g_encoder_raw_unwrapped);
    g_encoder_raw_unwrapped_max = std::max(g_encoder_raw_unwrapped_max, g_encoder_raw_unwrapped);
    g_encoder_maximum_frame_delta = std::max(g_encoder_maximum_frame_delta, std::abs(delta));
}

float encoder_angle_radians(bool* const calibrated = nullptr)
{
    int32_t calibrated_ticks = 0;
    const bool calibration_applied =
        g_encoder_calibration.calibrated_position_ticks(g_encoder_angle_raw, calibrated_ticks);
    if (calibrated != nullptr) {
        *calibrated = calibration_applied;
    }
    if (!calibration_applied) {
        return raw_encoder_angle_radians();
    }

    calibrated_ticks = apply_output_encoder_direction(calibrated_ticks);
    constexpr float kRadiansPerEncoderTick =
        (2.0F * static_cast<float>(M_PI)) / static_cast<float>(_ENCODER_READMASK + 1);
    return static_cast<float>(calibrated_ticks) * kRadiansPerEncoderTick;
}

float tmc_angle_radians(void)
{
    return steps_to_rads(tmc5160_position_read(), kRobotJointProfile->joint_full_steps);
}

float tmc_corrected_angle_radians(void)
{
    return tmc_angle_radians() + g_startup_tmc_angle_offset_rad;
}

float tmc_delta_output_radians(const int32_t delta_steps)
{
    const encoder_calibration_data& calibration = g_encoder_calibration.data();
    if (g_encoder_calibration.has_stored_data() &&
        (calibration.tmc_span_steps != 0) &&
        (calibration.manual_span_ticks != 0)) {
        const int32_t direction = (calibration.manual_span_ticks > 0) ? 1 : -1;
        const int32_t calibrated_span_ticks = calibration.manual_span_ticks -
            (2 * direction * static_cast<int32_t>(calibration.safe_margin_ticks));
        const float raw_delta_ticks =
            (static_cast<float>(delta_steps) * static_cast<float>(calibrated_span_ticks)) /
            static_cast<float>(calibration.tmc_span_steps);
        constexpr float kRadiansPerEncoderTick =
            (2.0F * static_cast<float>(M_PI)) / static_cast<float>(_ENCODER_READMASK + 1);
        return static_cast<float>(apply_output_encoder_direction(1)) *
               raw_delta_ticks * kRadiansPerEncoderTick;
    }
    return steps_to_rads(delta_steps, kRobotJointProfile->joint_full_steps);
}

float calibrated_backlash_radians(void)
{
    const int32_t backlash_steps = std::max<int32_t>(g_encoder_calibration.data().backlash_steps, 0);
    return std::fabs(tmc_delta_output_radians(backlash_steps));
}

JointLimitEnvelope joint_limit_envelope(void)
{
    JointLimitEnvelope limits{};
    int32_t hard_a_ticks = 0;
    int32_t soft_a_ticks = 0;
    int32_t soft_b_ticks = 0;
    int32_t hard_b_ticks = 0;
    if (!g_encoder_calibration.calibrated_limit_ticks(
            hard_a_ticks,
            soft_a_ticks,
            soft_b_ticks,
            hard_b_ticks)) {
        return limits;
    }

    constexpr float kRadiansPerEncoderTick =
        (2.0F * static_cast<float>(M_PI)) / static_cast<float>(_ENCODER_READMASK + 1);
    const float hard_a_rad = native_to_manipulator_radians(
        static_cast<float>(apply_output_encoder_direction(hard_a_ticks)) * kRadiansPerEncoderTick);
    const float soft_a_rad = native_to_manipulator_radians(
        static_cast<float>(apply_output_encoder_direction(soft_a_ticks)) * kRadiansPerEncoderTick);
    const float soft_b_rad = native_to_manipulator_radians(
        static_cast<float>(apply_output_encoder_direction(soft_b_ticks)) * kRadiansPerEncoderTick);
    const float hard_b_rad = native_to_manipulator_radians(
        static_cast<float>(apply_output_encoder_direction(hard_b_ticks)) * kRadiansPerEncoderTick);

    limits.hard_lower_rad = std::min(hard_a_rad, hard_b_rad);
    limits.hard_upper_rad = std::max(hard_a_rad, hard_b_rad);
    limits.soft_lower_rad = std::min(soft_a_rad, soft_b_rad);
    limits.soft_upper_rad = std::max(soft_a_rad, soft_b_rad);
    const float travel_rad = limits.hard_upper_rad - limits.hard_lower_rad;
    const float maximum_direct_velocity_rad_s =
        kRobotJointProfile->maximum_direct_velocity_rad_s;
    const float required_soft_width_rad =
        ((maximum_direct_velocity_rad_s * maximum_direct_velocity_rad_s) /
         (2.0F * kLimitAssumedDecelerationRadS2)) +
        (maximum_direct_velocity_rad_s * kLimitReactionTimeS);
    const float minimum_soft_width_rad = std::min(
        required_soft_width_rad,
        travel_rad * 0.25F);
    limits.soft_lower_rad = std::max(
        limits.soft_lower_rad,
        limits.hard_lower_rad + minimum_soft_width_rad);
    limits.soft_upper_rad = std::min(
        limits.soft_upper_rad,
        limits.hard_upper_rad - minimum_soft_width_rad);
    limits.active =
        std::isfinite(limits.hard_lower_rad) &&
        std::isfinite(limits.soft_lower_rad) &&
        std::isfinite(limits.soft_upper_rad) &&
        std::isfinite(limits.hard_upper_rad) &&
        (limits.hard_lower_rad < limits.soft_lower_rad) &&
        (limits.soft_lower_rad < limits.soft_upper_rad) &&
        (limits.soft_upper_rad < limits.hard_upper_rad);
    return limits;
}

void joint_velocity_bounds(
    const JointLimitEnvelope& limits,
    const float position_rad,
    const float maximum_joint_velocity_rad_s,
    float& minimum_velocity_rad_s,
    float& maximum_velocity_rad_s)
{
    minimum_velocity_rad_s = -maximum_joint_velocity_rad_s;
    maximum_velocity_rad_s = maximum_joint_velocity_rad_s;
    if (!limits.active) {
        minimum_velocity_rad_s = 0.0F;
        maximum_velocity_rad_s = 0.0F;
        return;
    }

    if (position_rad <= limits.hard_lower_rad) {
        minimum_velocity_rad_s = 0.0F;
    } else if (position_rad < limits.soft_lower_rad) {
        const float fraction = std::clamp(
            (position_rad - limits.hard_lower_rad) /
                (limits.soft_lower_rad - limits.hard_lower_rad),
            0.0F,
            1.0F);
        minimum_velocity_rad_s = -maximum_joint_velocity_rad_s * fraction;
    }

    if (position_rad >= limits.hard_upper_rad) {
        maximum_velocity_rad_s = 0.0F;
    } else if (position_rad > limits.soft_upper_rad) {
        const float fraction = std::clamp(
            (limits.hard_upper_rad - position_rad) /
                (limits.hard_upper_rad - limits.soft_upper_rad),
            0.0F,
            1.0F);
        maximum_velocity_rad_s = maximum_joint_velocity_rad_s * fraction;
    }
}

float limit_joint_velocity(
    const float requested_velocity_rad_s,
    const float maximum_joint_velocity_rad_s)
{
    float minimum_velocity_rad_s = -maximum_joint_velocity_rad_s;
    float maximum_velocity_rad_s = maximum_joint_velocity_rad_s;
    joint_velocity_bounds(
        joint_limit_envelope(),
        motor_fused_angle_manipulator(),
        maximum_joint_velocity_rad_s,
        minimum_velocity_rad_s,
        maximum_velocity_rad_s);
    return std::clamp(
        requested_velocity_rad_s,
        minimum_velocity_rad_s,
        maximum_velocity_rad_s);
}

void sync_tmc_offset_to_encoder(void)
{
    g_startup_tmc_angle_offset_rad = 0.0F;
    if (g_output_encoder_available) {
        g_startup_tmc_angle_offset_rad = encoder_angle_radians() - tmc_angle_radians();
    }
}

void reset_fusion_tracking(const uint32_t now_ms)
{
    const int32_t tmc_steps = tmc5160_position_read();
    g_prev_fusion_tmc_steps = tmc_steps;
    g_prev_fusion_ts_ms = now_ms;
    g_fusion_encoder_angle_rad = encoder_angle_radians(&g_fusion_uses_calibrated_encoder);
    g_fusion_tmc_angle_rad = g_output_encoder_available
        ? g_fusion_encoder_angle_rad
        : tmc_corrected_angle_radians();
    g_fused_angle_rad = g_fusion_tmc_angle_rad;
    g_fusion_offset_rad = g_fused_angle_rad - g_fusion_tmc_angle_rad;
    g_fusion_encoder_error_rad = 0.0F;
    g_fusion_backlash_rad = calibrated_backlash_radians();
    g_fusion_innovation_rad = 0.0F;
    g_fusion_applied_correction_rad = 0.0F;
    g_slip_window_residual_rad = 0.0F;
    g_slip_pending_tmc_delta_rad = 0.0F;
    g_slip_previous_encoder_angle_rad = g_fusion_encoder_angle_rad;
    g_previous_observed_encoder_angle_rad = g_fusion_encoder_angle_rad;
    g_spike_candidate_innovation_rad = 0.0F;
    g_spike_candidate_frames = 0U;
    g_spike_gate_open = false;
    g_stationary_encoder_reference_valid = false;
    g_stationary_encoder_min_rad = g_fusion_encoder_angle_rad;
    g_stationary_encoder_max_rad = g_fusion_encoder_angle_rad;
    g_stationary_encoder_window_started_ms = now_ms;
    g_external_motion_confirmation_frames = 0U;
    g_manual_encoder_window_started_ms = now_ms;
    g_manual_encoder_window_min_rad = g_fusion_encoder_angle_rad;
    g_manual_encoder_window_max_rad = g_fusion_encoder_angle_rad;
    g_slip_candidate = false;
    g_slip_persistent_windows = 0U;
    g_slip_window_index = 0U;
    g_slip_window_count = 0U;
    g_slip_residual_window.fill(0.0F);
    g_hybrid_takeup_tmc_travel_rad = 0.0F;
    g_hybrid_takeup_encoder_travel_rad = 0.0F;
    g_hybrid_motion_direction = 0;
    g_hybrid_lock_confirmation_frames = 0U;
    g_hybrid_lock_offset_sum_rad = 0.0F;
    g_motion_mismatch_confirmation_frames = 0U;
    g_hybrid_state = MOTOR_HYBRID_STATE_UNKNOWN;
    g_fusion_encoder_weight = 0.0F;
    g_previous_driver_enabled = g_tmc_driver.is_enabled();
    g_fused_velocity_rad_s = 0.0F;
    g_fusion_initialized = true;
}

bool hybrid_state_is_takeup(void)
{
    return (g_hybrid_state == MOTOR_HYBRID_STATE_TAKEUP_POSITIVE) ||
           (g_hybrid_state == MOTOR_HYBRID_STATE_TAKEUP_NEGATIVE);
}

bool hybrid_state_is_locked_for_direction(const int8_t direction)
{
    return ((direction > 0) &&
            (g_hybrid_state == MOTOR_HYBRID_STATE_LOCKED_POSITIVE)) ||
           ((direction < 0) &&
            (g_hybrid_state == MOTOR_HYBRID_STATE_LOCKED_NEGATIVE));
}

void begin_backlash_takeup(const int8_t direction)
{
    g_hybrid_motion_direction = direction;
    g_hybrid_takeup_tmc_travel_rad = 0.0F;
    g_hybrid_takeup_encoder_travel_rad = 0.0F;
    g_hybrid_lock_confirmation_frames = 0U;
    g_hybrid_lock_offset_sum_rad = 0.0F;
    g_motion_mismatch_confirmation_frames = 0U;
    g_slip_window_residual_rad = 0.0F;
    g_slip_persistent_windows = 0U;
    g_slip_window_index = 0U;
    g_slip_window_count = 0U;
    g_slip_residual_window.fill(0.0F);
    g_slip_candidate = false;
    g_hybrid_state = (direction > 0)
        ? MOTOR_HYBRID_STATE_TAKEUP_POSITIVE
        : MOTOR_HYBRID_STATE_TAKEUP_NEGATIVE;
}

void begin_manual_tracking(const uint32_t now_ms)
{
    g_hybrid_state = MOTOR_HYBRID_STATE_MANUAL;
    g_hybrid_motion_direction = 0;
    g_manual_encoder_window_started_ms = now_ms;
    g_manual_encoder_window_min_rad = g_fusion_encoder_angle_rad;
    g_manual_encoder_window_max_rad = g_fusion_encoder_angle_rad;
}

void set_output_encoder_available(const bool available, const uint32_t now_ms)
{
    const bool was_available = g_output_encoder_available;
    g_output_encoder_available = available;
    g_output_encoder_degraded = kRobotJointProfile->has_output_encoder && !available;

    if (available && !was_available) {
        sync_tmc_offset_to_encoder();
        reset_fusion_tracking(now_ms);
    }
}

void update_fusion_state(const uint32_t now_ms)
{
    if (!g_fusion_initialized) {
        reset_fusion_tracking(now_ms);
        return;
    }

    const int32_t tmc_steps = tmc5160_position_read();
    const int32_t delta_tmc_steps = static_cast<int32_t>(
        static_cast<uint32_t>(tmc_steps) - static_cast<uint32_t>(g_prev_fusion_tmc_steps));
    const float delta_tmc_rad = tmc_delta_output_radians(delta_tmc_steps);
    const float previous_fused_angle = g_fused_angle_rad;
    const float tmc_predicted_angle = previous_fused_angle + delta_tmc_rad;
    float predicted_angle = tmc_predicted_angle;
    g_fusion_encoder_weight = 0.0F;

    g_fusion_tmc_angle_rad += delta_tmc_rad;
    g_fusion_encoder_angle_rad = encoder_angle_radians(&g_fusion_uses_calibrated_encoder);
    g_fusion_encoder_error_rad = 0.0F;
    g_fusion_innovation_rad = 0.0F;
    g_fusion_applied_correction_rad = 0.0F;
    g_slip_pending_tmc_delta_rad += delta_tmc_rad;

    constexpr float kMotionDirectionThresholdRad = 0.25F * kEncoderRadiansPerTick;
    const float manipulator_tmc_delta = native_to_manipulator_radians(delta_tmc_rad);
    const int8_t motion_direction =
        (manipulator_tmc_delta > kMotionDirectionThresholdRad) ? 1 :
        ((manipulator_tmc_delta < -kMotionDirectionThresholdRad) ? -1 : 0);
    if (motion_direction != 0) {
        g_stationary_encoder_reference_valid = false;
        g_stationary_encoder_window_started_ms = now_ms;
        g_external_motion_confirmation_frames = 0U;
        const bool takeup_direction_changed =
            hybrid_state_is_takeup() &&
            (motion_direction != g_hybrid_motion_direction);
        const bool currently_locked =
            (g_hybrid_state == MOTOR_HYBRID_STATE_LOCKED_POSITIVE) ||
            (g_hybrid_state == MOTOR_HYBRID_STATE_LOCKED_NEGATIVE);
        const bool locked_direction_changed =
            currently_locked &&
            !hybrid_state_is_locked_for_direction(motion_direction);
        const bool recover_mismatch_in_opposite_direction =
            (g_hybrid_state == MOTOR_HYBRID_STATE_MOTION_MISMATCH) &&
            (motion_direction != g_hybrid_motion_direction);
        if ((g_hybrid_state == MOTOR_HYBRID_STATE_UNKNOWN) ||
            (g_hybrid_state == MOTOR_HYBRID_STATE_MANUAL) ||
            takeup_direction_changed ||
            locked_direction_changed ||
            recover_mismatch_in_opposite_direction) {
            begin_backlash_takeup(motion_direction);
        }
    }

    const bool driver_enabled = g_tmc_driver.is_enabled();
    if (!driver_enabled && g_previous_driver_enabled) {
        begin_manual_tracking(now_ms);
    }
    if (!driver_enabled) {
        g_slip_candidate = false;
        g_slip_persistent_windows = 0U;
        g_slip_window_index = 0U;
        g_slip_window_count = 0U;
        g_slip_window_residual_rad = 0.0F;
        g_slip_residual_window.fill(0.0F);
    }

    if (hybrid_state_is_takeup()) {
        predicted_angle = previous_fused_angle;
        const float directed_tmc_delta =
            static_cast<float>(g_hybrid_motion_direction) * manipulator_tmc_delta;
        if (directed_tmc_delta > 0.0F) {
            g_hybrid_takeup_tmc_travel_rad += directed_tmc_delta;
        }
    }

    if (g_output_encoder_available) {
        const float innovation = g_fusion_encoder_angle_rad - tmc_predicted_angle;
        g_fusion_innovation_rad = innovation;
        const uint32_t dt_ms = now_ms - g_prev_fusion_ts_ms;
        const float maximum_encoder_delta =
            (kFusionMaximumEncoderRateRadS * static_cast<float>(dt_ms) / 1000.0F) +
            kFusionEncoderRateMarginRad;
        const float observed_encoder_delta =
            g_fusion_encoder_angle_rad - g_previous_observed_encoder_angle_rad;
        g_previous_observed_encoder_angle_rad = g_fusion_encoder_angle_rad;
        const bool physically_plausible =
            std::fabs(observed_encoder_delta) <= maximum_encoder_delta;
        const bool large_encoder_step =
            std::fabs(observed_encoder_delta) > kFusionSpikeThresholdRad;
        bool accept_encoder = physically_plausible;

        if (!physically_plausible) {
            accept_encoder = false;
            g_spike_candidate_frames = 0U;
            g_spike_gate_open = false;
        } else if (!large_encoder_step) {
            g_spike_candidate_frames = 0U;
            g_spike_gate_open = false;
        } else if (!g_spike_gate_open) {
            const bool consistent_candidate =
                (g_spike_candidate_frames > 0U) &&
                ((observed_encoder_delta > 0.0F) ==
                 (g_spike_candidate_innovation_rad > 0.0F)) &&
                (std::fabs(observed_encoder_delta - g_spike_candidate_innovation_rad) <=
                 kFusionSpikeConsistencyRad);
            if (!consistent_candidate) {
                g_spike_candidate_frames = 1U;
                g_spike_candidate_innovation_rad = observed_encoder_delta;
            } else {
                ++g_spike_candidate_frames;
                g_spike_candidate_innovation_rad =
                    0.5F * (g_spike_candidate_innovation_rad + observed_encoder_delta);
            }
            accept_encoder = g_spike_candidate_frames >= kFusionSpikeConfirmationFrames;
            g_spike_gate_open = accept_encoder;
        }

        if (accept_encoder) {
            const float accepted_encoder_delta =
                g_fusion_encoder_angle_rad - g_slip_previous_encoder_angle_rad;
            const float accepted_tmc_delta = g_slip_pending_tmc_delta_rad;
            const float residual_delta =
                accepted_encoder_delta - accepted_tmc_delta;

            if (motion_direction == 0) {
                if (!g_stationary_encoder_reference_valid) {
                    g_stationary_encoder_reference_valid = true;
                    g_stationary_encoder_min_rad = g_fusion_encoder_angle_rad;
                    g_stationary_encoder_max_rad = g_fusion_encoder_angle_rad;
                    g_stationary_encoder_window_started_ms = now_ms;
                    g_external_motion_confirmation_frames = 0U;
                }
                g_stationary_encoder_min_rad = std::min(
                    g_stationary_encoder_min_rad,
                    g_fusion_encoder_angle_rad);
                g_stationary_encoder_max_rad = std::max(
                    g_stationary_encoder_max_rad,
                    g_fusion_encoder_angle_rad);
                const float external_span =
                    g_stationary_encoder_max_rad - g_stationary_encoder_min_rad;
                if (external_span > kExternalMotionThresholdRad) {
                    if (g_external_motion_confirmation_frames < 255U) {
                        ++g_external_motion_confirmation_frames;
                    }
                } else {
                    g_external_motion_confirmation_frames = 0U;
                    if ((now_ms - g_stationary_encoder_window_started_ms) >=
                        kManualQuietWindowMs) {
                        g_stationary_encoder_window_started_ms = now_ms;
                        g_stationary_encoder_min_rad = g_fusion_encoder_angle_rad;
                        g_stationary_encoder_max_rad = g_fusion_encoder_angle_rad;
                    }
                }
            }
            if (g_slip_window_count == kSlipWindowSamples) {
                g_slip_window_residual_rad -= g_slip_residual_window[g_slip_window_index];
            } else {
                ++g_slip_window_count;
            }
            g_slip_residual_window[g_slip_window_index] = residual_delta;
            g_slip_window_residual_rad += residual_delta;
            g_slip_window_index = (g_slip_window_index + 1U) % kSlipWindowSamples;
            g_slip_previous_encoder_angle_rad = g_fusion_encoder_angle_rad;
            g_slip_pending_tmc_delta_rad = 0.0F;

            if (g_hybrid_state == MOTOR_HYBRID_STATE_MANUAL) {
                const float manual_alpha = 1.0F - std::exp(
                    -static_cast<float>(dt_ms) / kManualEncoderTimeConstantMs);
                predicted_angle =
                    tmc_predicted_angle + (manual_alpha * innovation);
                g_fusion_encoder_weight = manual_alpha;
                g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;

                g_manual_encoder_window_min_rad = std::min(
                    g_manual_encoder_window_min_rad,
                    g_fusion_encoder_angle_rad);
                g_manual_encoder_window_max_rad = std::max(
                    g_manual_encoder_window_max_rad,
                    g_fusion_encoder_angle_rad);
                if ((now_ms - g_manual_encoder_window_started_ms) >=
                    kManualQuietWindowMs) {
                    const float window_span =
                        g_manual_encoder_window_max_rad - g_manual_encoder_window_min_rad;
                    if (window_span <= kManualQuietSpanRad) {
                        g_hybrid_state = MOTOR_HYBRID_STATE_UNKNOWN;
                        g_hybrid_motion_direction = 0;
                        g_stationary_encoder_reference_valid = true;
                        g_stationary_encoder_min_rad = g_fusion_encoder_angle_rad;
                        g_stationary_encoder_max_rad = g_fusion_encoder_angle_rad;
                        g_stationary_encoder_window_started_ms = now_ms;
                        g_external_motion_confirmation_frames = 0U;
                    } else {
                        g_manual_encoder_window_started_ms = now_ms;
                        g_manual_encoder_window_min_rad = g_fusion_encoder_angle_rad;
                        g_manual_encoder_window_max_rad = g_fusion_encoder_angle_rad;
                    }
                }
            } else if (hybrid_state_is_takeup()) {
                predicted_angle = g_fusion_encoder_angle_rad;
                g_fusion_encoder_weight = 1.0F;
                g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;

                const float directed_encoder_delta =
                    static_cast<float>(g_hybrid_motion_direction) *
                    native_to_manipulator_radians(accepted_encoder_delta);
                const float directed_accepted_tmc_delta =
                    static_cast<float>(g_hybrid_motion_direction) *
                    native_to_manipulator_radians(accepted_tmc_delta);
                g_hybrid_takeup_encoder_travel_rad = std::max(
                    0.0F,
                    g_hybrid_takeup_encoder_travel_rad + directed_encoder_delta);

                const bool encoder_follows_motor =
                    (directed_accepted_tmc_delta > kMotionDirectionThresholdRad) &&
                    (directed_encoder_delta >
                     (0.25F * directed_accepted_tmc_delta));
                const bool encoder_contradicts_motor =
                    directed_encoder_delta < -kMotionDirectionThresholdRad;
                if (encoder_contradicts_motor) {
                    g_hybrid_lock_confirmation_frames = 0U;
                    g_hybrid_lock_offset_sum_rad = 0.0F;
                } else if (encoder_follows_motor ||
                           (g_hybrid_lock_confirmation_frames > 0U)) {
                    if (g_hybrid_lock_confirmation_frames < 255U) {
                        ++g_hybrid_lock_confirmation_frames;
                    }
                    g_hybrid_lock_offset_sum_rad +=
                        g_fusion_encoder_angle_rad - g_fusion_tmc_angle_rad;
                }

                if ((g_hybrid_lock_confirmation_frames >= kBacklashLockConfirmationFrames) &&
                    (g_hybrid_takeup_encoder_travel_rad >= kBacklashLockMotionRad)) {
                    const float averaged_lock_offset =
                        g_hybrid_lock_offset_sum_rad /
                        static_cast<float>(g_hybrid_lock_confirmation_frames);
                    predicted_angle = g_fusion_tmc_angle_rad + averaged_lock_offset;
                    g_fusion_applied_correction_rad =
                        predicted_angle - tmc_predicted_angle;
                    g_hybrid_state = (g_hybrid_motion_direction > 0)
                        ? MOTOR_HYBRID_STATE_LOCKED_POSITIVE
                        : MOTOR_HYBRID_STATE_LOCKED_NEGATIVE;
                    g_slip_window_residual_rad = 0.0F;
                    g_slip_persistent_windows = 0U;
                    g_slip_window_index = 0U;
                    g_slip_window_count = 0U;
                    g_slip_residual_window.fill(0.0F);
                    g_slip_candidate = false;
                } else if (
                    (g_hybrid_takeup_tmc_travel_rad >
                     (g_fusion_backlash_rad + kBacklashMismatchMarginRad)) &&
                    (g_hybrid_takeup_encoder_travel_rad < kBacklashLockMotionRad)) {
                    g_hybrid_state = MOTOR_HYBRID_STATE_MOTION_MISMATCH;
                }
            }

            if ((motion_direction == 0) &&
                (g_hybrid_state != MOTOR_HYBRID_STATE_MANUAL) &&
                (g_external_motion_confirmation_frames >= kStateConfirmationFrames)) {
                begin_manual_tracking(now_ms);
                const float manual_alpha = 1.0F - std::exp(
                    -static_cast<float>(dt_ms) / kManualEncoderTimeConstantMs);
                predicted_angle =
                    tmc_predicted_angle + (manual_alpha * innovation);
                g_fusion_encoder_weight = manual_alpha;
                g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;
            } else if (g_hybrid_state == MOTOR_HYBRID_STATE_MOTION_MISMATCH) {
                predicted_angle = g_fusion_encoder_angle_rad;
                g_fusion_encoder_weight = 1.0F;
                g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;
            }

            const bool slip_detection_enabled =
                driver_enabled &&
                !g_encoder_calibration.blocks_normal_control() &&
                (motion_direction != 0);
            const float slip_threshold_rad =
                kSlipBacklashMultiplier * g_fusion_backlash_rad;
            const bool slip_threshold_valid =
                std::isfinite(slip_threshold_rad) &&
                (slip_threshold_rad > 0.0F);
            const bool locked_motion =
                slip_detection_enabled &&
                hybrid_state_is_locked_for_direction(motion_direction);
            if (locked_motion &&
                (std::fabs(g_slip_window_residual_rad) > kBacklashMismatchMarginRad)) {
                if (g_motion_mismatch_confirmation_frames < 255U) {
                    ++g_motion_mismatch_confirmation_frames;
                }
            } else if (g_hybrid_state != MOTOR_HYBRID_STATE_MOTION_MISMATCH) {
                g_motion_mismatch_confirmation_frames = 0U;
            }
            if ((g_motion_mismatch_confirmation_frames >= kStateConfirmationFrames) &&
                (g_hybrid_state != MOTOR_HYBRID_STATE_MOTION_MISMATCH)) {
                g_hybrid_state = MOTOR_HYBRID_STATE_MOTION_MISMATCH;
                g_hybrid_lock_confirmation_frames = 0U;
                predicted_angle = g_fusion_encoder_angle_rad;
                g_fusion_encoder_weight = 1.0F;
                g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;
            }
            const bool persistent_now =
                slip_detection_enabled &&
                !hybrid_state_is_takeup() &&
                (g_hybrid_state != MOTOR_HYBRID_STATE_UNKNOWN) &&
                (g_hybrid_state != MOTOR_HYBRID_STATE_MANUAL) &&
                slip_threshold_valid &&
                (std::fabs(g_slip_window_residual_rad) > slip_threshold_rad);
            if (persistent_now) {
                if (g_slip_persistent_windows < kSlipPersistentWindows) {
                    ++g_slip_persistent_windows;
                }
                if (!g_slip_candidate &&
                    (g_slip_persistent_windows >= kSlipPersistentWindows)) {
                    g_slip_candidate = true;
                    ++g_slip_persistent_residual_count;
                    g_hybrid_state = MOTOR_HYBRID_STATE_MOTION_MISMATCH;
                    predicted_angle = g_fusion_encoder_angle_rad;
                    g_fusion_encoder_weight = 1.0F;
                    g_fusion_applied_correction_rad = predicted_angle - tmc_predicted_angle;
                    g_fault_manager.latch_motor_slip();
                }
            } else if (g_hybrid_state != MOTOR_HYBRID_STATE_MOTION_MISMATCH) {
                g_slip_persistent_windows = 0U;
                g_slip_candidate = false;
            }

            if ((g_hybrid_state == MOTOR_HYBRID_STATE_MOTION_MISMATCH) &&
                !g_slip_candidate &&
                slip_detection_enabled &&
                slip_threshold_valid &&
                (std::fabs(g_slip_window_residual_rad) < slip_threshold_rad)) {
                const float directed_encoder_delta =
                    static_cast<float>(motion_direction) *
                    native_to_manipulator_radians(accepted_encoder_delta);
                const float directed_tmc_delta =
                    static_cast<float>(motion_direction) *
                    native_to_manipulator_radians(accepted_tmc_delta);
                const bool motion_consistent =
                    (directed_tmc_delta > kMotionDirectionThresholdRad) &&
                    (directed_encoder_delta > (0.25F * directed_tmc_delta)) &&
                    (std::fabs(residual_delta) <= kBacklashMismatchMarginRad);
                if (motion_consistent) {
                    if (g_hybrid_lock_confirmation_frames < 255U) {
                        ++g_hybrid_lock_confirmation_frames;
                    }
                } else {
                    g_hybrid_lock_confirmation_frames = 0U;
                }
                if (g_hybrid_lock_confirmation_frames >= kStateConfirmationFrames) {
                    g_hybrid_state = (motion_direction > 0)
                        ? MOTOR_HYBRID_STATE_LOCKED_POSITIVE
                        : MOTOR_HYBRID_STATE_LOCKED_NEGATIVE;
                    g_motion_mismatch_confirmation_frames = 0U;
                    g_slip_window_residual_rad = 0.0F;
                    g_slip_persistent_windows = 0U;
                    g_slip_window_index = 0U;
                    g_slip_window_count = 0U;
                    g_slip_residual_window.fill(0.0F);
                }
            }
        } else {
            ++g_fusion_rejected_spike_count;
        }
        g_fusion_encoder_error_rad = g_fusion_encoder_angle_rad - predicted_angle;
    }

    g_fused_angle_rad = predicted_angle;
    g_fusion_offset_rad = g_fused_angle_rad - g_fusion_tmc_angle_rad;
    g_fusion_backlash_rad = calibrated_backlash_radians();

    const uint32_t dt_ms = now_ms - g_prev_fusion_ts_ms;
    if (dt_ms > 0U) {
        const float measured_velocity =
            (g_fused_angle_rad - previous_fused_angle) / (static_cast<float>(dt_ms) / 1000.0F);
        const float reference_alpha = std::clamp(
            kRobotJointProfile->velocity_encoder_lpf_alpha,
            0.0F,
            1.0F);
        const float period_ratio =
            static_cast<float>(dt_ms) / static_cast<float>(kFilterReferencePeriodMs);
        const float alpha =
            1.0F - std::pow(1.0F - reference_alpha, period_ratio);
        g_fused_velocity_rad_s = (alpha * measured_velocity) +
                                 ((1.0F - alpha) * g_fused_velocity_rad_s);
    }
    g_prev_fusion_tmc_steps = tmc_steps;
    g_prev_fusion_ts_ms = now_ms;
    g_previous_driver_enabled = driver_enabled;
}

bool manipulator_radians_to_tmc_steps(
    const float manipulator_angle_rad,
    int32_t& target_position_steps)
{
    const encoder_calibration_data& calibration = g_encoder_calibration.data();
    if (!g_encoder_calibration.has_stored_data() ||
        (calibration.tmc_span_steps == 0) ||
        (calibration.manual_span_ticks == 0)) {
        return false;
    }

    const float target_delta_rad =
        manipulator_angle_rad - motor_fused_angle_manipulator();
    const float radians_per_tmc_step = tmc_delta_output_radians(1);
    if (!std::isfinite(radians_per_tmc_step) ||
        (std::fabs(radians_per_tmc_step) < 1.0e-12F)) {
        return false;
    }

    const double delta_steps =
        static_cast<double>(target_delta_rad) /
        static_cast<double>(radians_per_tmc_step);
    const double absolute_target_steps =
        static_cast<double>(tmc5160_position_read()) + std::round(delta_steps);
    if (!std::isfinite(absolute_target_steps) ||
        (absolute_target_steps < static_cast<double>(INT32_MIN)) ||
        (absolute_target_steps > static_cast<double>(INT32_MAX))) {
        return false;
    }
    target_position_steps = static_cast<int32_t>(absolute_target_steps);
    return true;
}

void clear_servo_target(void)
{
    g_servo_target_valid = false;
    g_servo_at_target = false;
    g_servo_requires_controller = false;
    g_servo_service_position_tracking = false;
    g_servo_state = MOTOR_SERVO_STATE_INACTIVE;
    g_servo_position_error_rad = 0.0F;
    g_servo_command_velocity_rad_s = 0.0F;
    g_servo_maximum_correction_velocity_rad_s = kServoMaximumCorrectionVelocityRadS;
    g_servo_tracking_velocity_rad_s = 0.0F;
    g_servo_tracking_velocity_steps = -1;
}

bool command_servo_velocity(const float manipulator_velocity_rad_s)
{
    const float limited_velocity_rad_s = limit_joint_velocity(
        manipulator_velocity_rad_s,
        kRobotJointProfile->maximum_servo_velocity_rad_s);
    const float native_velocity_rad_s = manipulator_to_native_radians(limited_velocity_rad_s);
    int32_t velocity_steps = 0;
    if (!radians_to_steps_checked(
            native_velocity_rad_s,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &velocity_steps)) {
        return false;
    }

    tmc5160_move(velocity_steps);
    g_servo_command_velocity_rad_s = limited_velocity_rad_s;
    return true;
}

bool apply_direct_velocity(const float requested_velocity_rad_s)
{
    const float limited_velocity_rad_s = limit_joint_velocity(
        requested_velocity_rad_s,
        kRobotJointProfile->maximum_direct_velocity_rad_s);
    const float native_velocity_rad_s = manipulator_to_native_radians(limited_velocity_rad_s);
    int32_t velocity_steps = 0;
    if (!radians_to_steps_checked(
            native_velocity_rad_s,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &velocity_steps)) {
        return false;
    }
    if (!g_direct_applied_velocity_valid ||
        (velocity_steps != g_direct_applied_velocity_steps)) {
        tmc5160_move(velocity_steps);
        g_direct_applied_velocity_steps = velocity_steps;
        g_direct_applied_velocity_valid = true;
    }
    return true;
}

void restore_default_motion_profile(void)
{
    if (g_custom_motion_profile_active) {
        tmc5160_apply_default_motion_profile();
        g_custom_motion_profile_active = false;
        g_servo_acceleration_steps = -1;
    }
}

bool apply_servo_acceleration(const float acceleration_rad_s2)
{
    if (acceleration_rad_s2 == 0.0F) {
        restore_default_motion_profile();
        return true;
    }

    int32_t acceleration_steps = 0;
    if (!radians_to_steps_checked(
            acceleration_rad_s2,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &acceleration_steps) ||
        (acceleration_steps <= 0)) {
        return false;
    }
    if (!g_custom_motion_profile_active ||
        (acceleration_steps != g_servo_acceleration_steps)) {
        tmc5160_acceleration(static_cast<uint32_t>(acceleration_steps));
        g_custom_motion_profile_active = true;
        g_servo_acceleration_steps = acceleration_steps;
    }
    return true;
}

void start_servo_tracking(
    const uint32_t now_ms,
    const float target_position_rad,
    const int32_t target_position_steps,
    const int32_t velocity_steps,
    const float velocity_rad_s,
    const bool requires_controller)
{
    g_servo_target_position_rad = target_position_rad;
    g_servo_position_error_rad = target_position_rad - motor_fused_angle_manipulator();
    g_servo_command_velocity_rad_s = velocity_rad_s;
    g_servo_last_command_ms = now_ms;
    g_servo_target_valid = true;
    g_servo_at_target = false;
    g_servo_requires_controller = requires_controller;
    g_servo_service_position_tracking = false;
    g_servo_maximum_correction_velocity_rad_s = kServoMaximumCorrectionVelocityRadS;
    g_servo_tracking_velocity_rad_s = std::fabs(velocity_rad_s);
    g_servo_tracking_velocity_steps = std::abs(velocity_steps);
    g_servo_state = MOTOR_SERVO_STATE_TRACKING;
    tmc5160_position(target_position_steps, velocity_steps);
    g_control_mode = MOTOR_CONTROL_MODE_SERVO;
}

void update_servo_control(const uint32_t now_ms)
{
    if (!g_servo_target_valid) {
        return;
    }
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() ||
        (g_servo_requires_controller && !g_fault_manager.remote_motion_allowed()) ||
        !g_tmc_driver.is_enabled()) {
        clear_servo_target();
        return;
    }

    const bool command_stream_alive =
        (now_ms - g_servo_last_command_ms) < kServoCommandTimeoutMs;
    if (g_servo_state == MOTOR_SERVO_STATE_TRACKING) {
        const float tracking_error_rad =
            g_servo_target_position_rad - motor_fused_angle_manipulator();
        const float tracking_direction =
            (tracking_error_rad >= 0.0F) ? 1.0F : -1.0F;
        const float limited_tracking_velocity_rad_s = limit_joint_velocity(
            tracking_direction * g_servo_tracking_velocity_rad_s,
            kRobotJointProfile->maximum_servo_velocity_rad_s);
        int32_t limited_tracking_velocity_steps = 0;
        if (!radians_to_steps_checked(
                std::fabs(limited_tracking_velocity_rad_s),
                static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
                &limited_tracking_velocity_steps)) {
            tmc5160_move(0);
            clear_servo_target();
            g_control_mode = MOTOR_CONTROL_MODE_HOLD;
            return;
        }
        if (limited_tracking_velocity_steps != g_servo_tracking_velocity_steps) {
            tmc5160_velocity(limited_tracking_velocity_steps);
            g_servo_tracking_velocity_steps = limited_tracking_velocity_steps;
        }
        g_servo_command_velocity_rad_s = limited_tracking_velocity_rad_s;

        if (std::fabs(tracking_error_rad) <= kServoStopToleranceRad) {
            g_servo_service_position_tracking = false;
            g_servo_command_velocity_rad_s = 0.0F;
            g_servo_state = MOTOR_SERVO_STATE_SETTLING;
        }

        if (g_servo_state != MOTOR_SERVO_STATE_TRACKING) {
            // Continue below and let the fused-angle loop enter AT_TARGET.
        } else if (g_servo_service_position_tracking) {
            if (!tmc5160_position_reached()) {
                return;
            }
            g_servo_service_position_tracking = false;
            g_servo_command_velocity_rad_s = 0.0F;
        } else if (command_stream_alive) {
            return;
        }
        // The TMC keeps approaching the last position while commands are fresh.
        // A silent command stream transfers that last target to the fused-angle loop.
        g_servo_state = MOTOR_SERVO_STATE_SETTLING;
    }

    // Do not wrap this error: the public joint coordinate intentionally spans
    // -2*pi..+2*pi even though the mechanism itself turns less than one revolution.
    g_servo_position_error_rad =
        g_servo_target_position_rad - motor_fused_angle_manipulator();
    const float absolute_error_rad = std::fabs(g_servo_position_error_rad);
    if (g_servo_at_target) {
        if (absolute_error_rad > kServoResumeToleranceRad) {
            g_servo_at_target = false;
            g_servo_state = MOTOR_SERVO_STATE_SETTLING;
        }
    } else if (absolute_error_rad <= kServoStopToleranceRad) {
        g_servo_at_target = true;
        g_servo_state = MOTOR_SERVO_STATE_AT_TARGET;
    }

    if (g_servo_at_target) {
        tmc5160_move(0);
        g_servo_command_velocity_rad_s = 0.0F;
        return;
    }

    const float correction_velocity_rad_s = std::clamp(
        kServoKp * g_servo_position_error_rad,
        -g_servo_maximum_correction_velocity_rad_s,
        g_servo_maximum_correction_velocity_rad_s);
    if (!command_servo_velocity(correction_velocity_rad_s)) {
        tmc5160_move(0);
        clear_servo_target();
        g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    }
}

void update_direct_control(void)
{
    if (g_control_mode != MOTOR_CONTROL_MODE_DIRECT) {
        return;
    }
    if (!apply_direct_velocity(g_direct_requested_velocity_rad_s)) {
        tmc5160_move(0);
        g_direct_applied_velocity_valid = false;
        g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    }
}

void update_calibration_jog(const uint32_t now_ms)
{
    if (!g_calibration_jog_active) {
        return;
    }
    if (!g_fault_manager.motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        ((now_ms - g_calibration_jog_last_command_ms) >= kServoCommandTimeoutMs)) {
        tmc5160_move(0);
        g_fault_manager.note_velocity_command(now_ms, false);
        g_calibration_jog_active = false;
        g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    }
}

void cancel_calibration_jog(void)
{
    if (!g_calibration_jog_active) {
        return;
    }
    tmc5160_move(0);
    g_fault_manager.note_velocity_command(HAL_GetTick(), false);
    g_calibration_jog_active = false;
    g_control_mode = MOTOR_CONTROL_MODE_HOLD;
}

void update_encoder_status(const uint32_t now_ms)
{
    if (!kRobotJointProfile->has_output_encoder) {
        set_output_encoder_available(false, now_ms);
        return;
    }

    if (!g_encoder_last_read_ok) {
        g_encoder_valid_streak = 0U;
        if (g_encoder_error_streak < 255U) {
            ++g_encoder_error_streak;
        }
        if (g_output_encoder_available && (g_encoder_error_streak >= kEncoderErrorStreakThreshold)) {
            set_output_encoder_available(false, now_ms);
        }
        return;
    }

    g_encoder_error_streak = 0U;
    if (g_encoder_valid_streak < 255U) {
        ++g_encoder_valid_streak;
    }
    if (g_output_encoder_degraded && (g_encoder_valid_streak >= kEncoderValidStreakThreshold)) {
        set_output_encoder_available(true, now_ms);
    }
}

void update_faults(const uint32_t now_ms)
{
    const Tmc5160Error tmc_error = g_tmc_driver.error();
    const bool fusion_offset_available =
        SR_ENABLE_FUSION_OFFSET_FAULT &&
        g_output_encoder_available &&
        g_fusion_uses_calibrated_encoder &&
        !g_encoder_calibration.blocks_normal_control() &&
        g_tmc_driver.is_enabled() &&
        (tmc_error == Tmc5160Error::None);
    const FaultManager::UpdateResult result = g_fault_manager.update(
        now_ms,
        fusion_offset_available,
        g_fusion_offset_rad,
        g_fusion_backlash_rad,
        tmc_error,
        g_tmc_driver.fault_snapshot());

    const StopReason stop_reason = g_fault_manager.stop_reason();
    const bool service_servo_ignores_network_stop =
        g_servo_target_valid &&
        !g_servo_requires_controller &&
        ((stop_reason == StopReason::NetworkOffline) ||
         (stop_reason == StopReason::ControllerOffline));
    if (result.stop_motion && !service_servo_ignores_network_stop) {
        tmc5160_move(0);
        clear_servo_target();
        g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    }
}

void update_calibration(const uint32_t now_ms)
{
    const EncoderCalibrationState calibration_state = g_encoder_calibration.state();
    const bool automatic_motion =
        (calibration_state == EncoderCalibrationState::MoveToA) ||
        (calibration_state == EncoderCalibrationState::SettleAtA) ||
        (calibration_state == EncoderCalibrationState::SweepToB) ||
        (calibration_state == EncoderCalibrationState::SettleAtB) ||
        (calibration_state == EncoderCalibrationState::ReverseSweepToA) ||
        (calibration_state == EncoderCalibrationState::MoveToRockStart) ||
        (calibration_state == EncoderCalibrationState::RockSettle) ||
        (calibration_state == EncoderCalibrationState::RockSweep) ||
        (calibration_state == EncoderCalibrationState::MoveToMiddle) ||
        (calibration_state == EncoderCalibrationState::SettleAtMiddle) ||
        (calibration_state == EncoderCalibrationState::Processing) ||
        (calibration_state == EncoderCalibrationState::Saving) ||
        (calibration_state == EncoderCalibrationState::AutoSeekLimitA) ||
        (calibration_state == EncoderCalibrationState::AutoBackoffA) ||
        (calibration_state == EncoderCalibrationState::AutoSeekLimitB) ||
        (calibration_state == EncoderCalibrationState::MoveToBStart);
    if (automatic_motion && !g_tmc_driver.is_enabled()) {
        g_encoder_calibration.fail(EncoderCalibrationError::Driver);
    }
    const EncoderCalibration::Action action = g_encoder_calibration.update(
        now_ms,
        g_output_encoder_available,
        g_encoder_angle_raw,
        tmc5160_position_read());
    if (action.zero_tmc_position) {
        tmc5160_set_zero();
        sync_tmc_offset_to_encoder();
        reset_fusion_tracking(now_ms);
    }
    if (action.command_velocity) {
        tmc5160_move(action.velocity_steps);
    }
    if (action.disable_driver && g_tmc_driver.is_enabled()) {
        g_tmc_driver.disable();
    }
    if (g_encoder_calibration.blocks_normal_control()) {
        g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
    }
}

void update_degraded_led(const uint32_t now_ms)
{
    if (!g_output_encoder_degraded) {
        return;
    }

    if ((now_ms - g_last_degraded_led_toggle_ms) >= kDegradedLedTogglePeriodMs) {
        g_last_degraded_led_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
}
}  // namespace

extern "C" void motor_init(void)
{
    const uint32_t now_ms = HAL_GetTick();

    g_fault_manager.initialize(now_ms, kRobotJointProfile->joint_index);
    g_tmc_driver.initialize(static_cast<uint8_t>(kRobotJointProfile->init_irun));
    clear_servo_target();
    g_control_mode = MOTOR_CONTROL_MODE_HOLD;

    g_zero_enc_runtime = kRobotJointProfile->default_zero_enc;

    if (kRobotJointProfile->has_output_encoder) {
        // AS50xx replies are pipelined: the first SPI exchange after power-up
        // returns the response to an earlier/undefined command.  Prime the
        // pipeline before using an angle to anchor the absolute joint state.
        uint16_t discarded_encoder_frame = 0U;
        (void)as50_readAngle(&discarded_encoder_frame, 100);
        g_encoder_last_read_ok = as50_readAngle(&g_encoder_angle_raw, 100);
        if (g_encoder_last_read_ok) {
            update_encoder_raw_statistics(g_encoder_angle_raw);
        }
        set_output_encoder_available(g_encoder_last_read_ok, now_ms);
    } else {
        g_encoder_angle_raw = 0U;
        g_encoder_last_read_ok = false;
        set_output_encoder_available(false, now_ms);
    }

    g_encoder_calibration.initialize(
        kRobotJointProfile->joint_index,
        kRobotJointProfile->output_encoder_inverted != 0,
        g_encoder_angle_raw);
    if (g_encoder_calibration.zero_valid()) {
        g_zero_enc_runtime = g_encoder_calibration.zero_raw();
    }
    g_encoder_error_streak = g_encoder_last_read_ok ? 0U : 1U;
    g_encoder_valid_streak = g_encoder_last_read_ok ? 1U : 0U;
    sync_tmc_offset_to_encoder();
    reset_fusion_tracking(now_ms);
    g_last_degraded_led_toggle_ms = now_ms;
}

extern "C" void motor_update(const uint32_t now_ms)
{
    g_tmc_driver.update(now_ms);

    if (kRobotJointProfile->has_output_encoder) {
        g_encoder_last_read_ok = as50_readAngle(&g_encoder_angle_raw, 100);
        if (g_encoder_last_read_ok) {
            update_encoder_raw_statistics(g_encoder_angle_raw);
        }
    } else {
        g_encoder_angle_raw = 0U;
        g_encoder_last_read_ok = false;
    }

    update_encoder_status(now_ms);
    update_calibration(now_ms);
    update_fusion_state(now_ms);
    update_calibration_jog(now_ms);
    update_faults(now_ms);
    update_direct_control();
    update_servo_control(now_ms);
    update_degraded_led(now_ms);
}

extern "C" bool motor_command(
    const float position_rad,
    const float velocity_rad_s,
    const float acceleration_rad_s2)
{
    if (!std::isfinite(position_rad) ||
        !std::isfinite(velocity_rad_s) ||
        !std::isfinite(acceleration_rad_s2) ||
        (acceleration_rad_s2 < 0.0F)) {
        return false;
    }

    const JointLimitEnvelope limits = joint_limit_envelope();
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.remote_motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        !limits.active ||
        (position_rad < limits.hard_lower_rad) ||
        (position_rad > limits.hard_upper_rad)) {
        return false;
    }

    const float requested_tracking_velocity_rad_s = std::copysign(
        std::min(
            std::fabs(velocity_rad_s),
            kRobotJointProfile->maximum_servo_velocity_rad_s),
        position_rad - motor_fused_angle_manipulator());
    const float limited_tracking_velocity_rad_s =
        limit_joint_velocity(
            requested_tracking_velocity_rad_s,
            kRobotJointProfile->maximum_servo_velocity_rad_s);

    int32_t target_position_steps = 0;
    if (!manipulator_radians_to_tmc_steps(position_rad, target_position_steps)) {
        return false;
    }

    int32_t feedforward_velocity_steps = 0;
    if (!radians_to_steps_checked(
            std::fabs(limited_tracking_velocity_rad_s),
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &feedforward_velocity_steps)) {
        return false;
    }

    if (!apply_servo_acceleration(acceleration_rad_s2)) {
        return false;
    }

    const uint32_t now_ms = HAL_GetTick();
    // Exact equality is intentional. A very slow but changing trajectory must
    // stay in feed-forward tracking rather than being mistaken for a fixed target.
    const bool same_target =
        g_servo_target_valid && (position_rad == g_servo_target_position_rad);
    g_fault_manager.note_velocity_command(now_ms, false);
    if (same_target) {
        g_servo_last_command_ms = now_ms;
        g_servo_state = g_servo_at_target
            ? MOTOR_SERVO_STATE_AT_TARGET
            : MOTOR_SERVO_STATE_SETTLING;
    } else {
        start_servo_tracking(
            now_ms,
            position_rad,
            target_position_steps,
            feedforward_velocity_steps,
            limited_tracking_velocity_rad_s,
            true);
    }
    g_control_mode = MOTOR_CONTROL_MODE_SERVO;
    return true;
}

extern "C" bool motor_move(const int32_t velocity_command)
{
    cancel_calibration_jog();
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        !joint_limit_envelope().active) {
        return false;
    }
    const float native_velocity_rad_s = steps_to_rads(
        velocity_command,
        static_cast<int32_t>(kRobotJointProfile->joint_full_steps));
    const float manipulator_velocity_rad_s =
        native_to_manipulator_radians(native_velocity_rad_s);
    clear_servo_target();
    restore_default_motion_profile();
    g_direct_requested_velocity_rad_s = manipulator_velocity_rad_s;
    g_direct_applied_velocity_valid = false;
    if (!apply_direct_velocity(g_direct_requested_velocity_rad_s)) {
        return false;
    }
    g_fault_manager.note_velocity_command(HAL_GetTick(), velocity_command != 0);
    g_control_mode = (velocity_command == 0)
        ? MOTOR_CONTROL_MODE_HOLD
        : MOTOR_CONTROL_MODE_DIRECT;
    return true;
}

extern "C" bool motor_set_tmc_position_steps(const int32_t target_position_steps)
{
    cancel_calibration_jog();
    const JointLimitEnvelope limits = joint_limit_envelope();
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        !limits.active) {
        return false;
    }

    const int32_t current_position_steps = tmc5160_position_read();
    const int32_t delta_steps = static_cast<int32_t>(
        static_cast<uint32_t>(target_position_steps) -
        static_cast<uint32_t>(current_position_steps));
    const float target_position_rad = motor_fused_angle_manipulator() +
        native_to_manipulator_radians(tmc_delta_output_radians(delta_steps));
    if (!std::isfinite(target_position_rad) ||
        (target_position_rad < limits.hard_lower_rad) ||
        (target_position_rad > limits.hard_upper_rad)) {
        return false;
    }

    int32_t velocity_steps = 0;
    if (!radians_to_steps_checked(
            kRobotJointProfile->maximum_servo_velocity_rad_s,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &velocity_steps) ||
        (velocity_steps <= 0)) {
        return false;
    }

    clear_servo_target();
    restore_default_motion_profile();
    g_fault_manager.note_velocity_command(HAL_GetTick(), false);
    tmc5160_position(target_position_steps, velocity_steps);
    g_control_mode = MOTOR_CONTROL_MODE_TMC_POSITION;
    return true;
}

extern "C" bool motor_move_calibration(const int32_t velocity_command)
{
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() ||
        !g_tmc_driver.is_enabled()) {
        return false;
    }
    clear_servo_target();
    restore_default_motion_profile();
    tmc5160_move(velocity_command);
    const uint32_t now_ms = HAL_GetTick();
    g_fault_manager.note_velocity_command(now_ms, velocity_command != 0);
    g_calibration_jog_last_command_ms = now_ms;
    g_calibration_jog_active = velocity_command != 0;
    g_control_mode = g_calibration_jog_active
        ? MOTOR_CONTROL_MODE_CALIBRATION
        : MOTOR_CONTROL_MODE_HOLD;
    return true;
}

extern "C" bool motor_move_radians_per_second(const float velocity_rad_s)
{
    cancel_calibration_jog();
    if (!std::isfinite(velocity_rad_s) ||
        g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.remote_motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        !joint_limit_envelope().active) {
        return false;
    }

    clear_servo_target();
    restore_default_motion_profile();
    g_direct_requested_velocity_rad_s = velocity_rad_s;
    g_direct_applied_velocity_valid = false;
    if (!apply_direct_velocity(g_direct_requested_velocity_rad_s)) {
        return false;
    }
    g_fault_manager.note_velocity_command(HAL_GetTick(), velocity_rad_s != 0.0F);
    g_control_mode = (velocity_rad_s == 0.0F)
        ? MOTOR_CONTROL_MODE_HOLD
        : MOTOR_CONTROL_MODE_DIRECT;
    return true;
}

extern "C" bool motor_set_position_radians(const float target_position_rad)
{
    cancel_calibration_jog();
    const JointLimitEnvelope limits = joint_limit_envelope();
    if (!std::isfinite(target_position_rad) ||
        g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() ||
        !g_tmc_driver.is_enabled() ||
        !limits.active ||
        (target_position_rad < limits.hard_lower_rad) ||
        (target_position_rad > limits.hard_upper_rad)) {
        return false;
    }
    const uint32_t now_ms = HAL_GetTick();
    int32_t acceleration_steps = 0;
    if (!radians_to_steps_checked(
            kPositionRegisterAccelerationRadS2,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &acceleration_steps)) {
        return false;
    }
    g_fault_manager.note_velocity_command(now_ms, false);
    g_servo_target_position_rad = target_position_rad;
    g_servo_position_error_rad =
        target_position_rad - motor_fused_angle_manipulator();
    g_servo_command_velocity_rad_s = std::copysign(
        kRobotJointProfile->maximum_servo_velocity_rad_s,
        g_servo_position_error_rad);
    g_servo_last_command_ms = now_ms;
    g_servo_target_valid = true;
    g_servo_at_target = false;
    g_servo_requires_controller = false;
    g_servo_service_position_tracking = false;
    g_servo_maximum_correction_velocity_rad_s =
        kRobotJointProfile->maximum_servo_velocity_rad_s;
    g_servo_tracking_velocity_rad_s = kRobotJointProfile->maximum_servo_velocity_rad_s;
    g_servo_tracking_velocity_steps = -1;
    g_servo_state = MOTOR_SERVO_STATE_SETTLING;
    tmc5160_acceleration(static_cast<uint32_t>(acceleration_steps));
    g_custom_motion_profile_active = true;
    tmc5160_move(0);
    g_control_mode = MOTOR_CONTROL_MODE_SERVO;
    return true;
}

extern "C" bool motor_arm(const bool armed)
{
    cancel_calibration_jog();
    if (g_encoder_calibration.blocks_normal_control()) {
        return false;
    }
    clear_servo_target();
    if (armed) {
        if (!g_fault_manager.motion_allowed()) {
            return false;
        }
        const bool enabled = g_tmc_driver.enable();
        if (enabled) {
            g_custom_motion_profile_active = false;
            sync_tmc_offset_to_encoder();
            reset_fusion_tracking(HAL_GetTick());
            g_control_mode = MOTOR_CONTROL_MODE_HOLD;
        }
        return enabled;
    }
    const bool disabled = g_tmc_driver.disable();
    if (disabled) {
        g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    }
    return disabled;
}

extern "C" bool motor_driver_enabled(void)
{
    return g_tmc_driver.is_enabled();
}

extern "C" int32_t motor_driver_state(void)
{
    return static_cast<int32_t>(g_tmc_driver.state());
}

extern "C" int32_t motor_driver_error(void)
{
    return static_cast<int32_t>(g_tmc_driver.error());
}

extern "C" int32_t motor_control_mode_get(void)
{
    return static_cast<int32_t>(g_control_mode);
}

extern "C" void motor_note_heartbeat(const uint32_t now_ms, const uint8_t source_node_id)
{
    g_fault_manager.note_heartbeat(
        now_ms,
        source_node_id == kRobotJointProfile->controller_node_id);
}

extern "C" int32_t motor_position_steps(void)
{
    return tmc5160_position_read();
}

extern "C" int32_t motor_velocity_steps(void)
{
    return tmc5160_velocity_read();
}

extern "C" uint16_t motor_encoder_raw(void)
{
    return g_encoder_angle_raw;
}

extern "C" bool motor_encoder_get_diagnostics(motor_encoder_diagnostics* const diagnostics)
{
    if (diagnostics == nullptr) {
        return false;
    }

    as50_diagnostics_t encoder_diagnostics{};
    as50_getDiagnostics(&encoder_diagnostics);
    diagnostics->raw_frame = encoder_diagnostics.raw_frame;
    diagnostics->transfer_count = encoder_diagnostics.transfer_count;
    diagnostics->error_count = encoder_diagnostics.error_count;
    diagnostics->last_hal_status = static_cast<int32_t>(encoder_diagnostics.last_hal_status);
    diagnostics->raw_unwrapped_min = g_encoder_raw_unwrapped_min;
    diagnostics->raw_unwrapped_max = g_encoder_raw_unwrapped_max;
    diagnostics->raw_unwrapped_span =
        g_encoder_raw_unwrapped_max - g_encoder_raw_unwrapped_min;
    diagnostics->maximum_frame_delta = g_encoder_maximum_frame_delta;
    diagnostics->last_read_ok = encoder_diagnostics.last_read_ok;
    diagnostics->has_valid_angle = encoder_diagnostics.has_valid_angle;
    return true;
}

extern "C" bool motor_fusion_get_diagnostics(motor_fusion_diagnostics* const diagnostics)
{
    if (diagnostics == nullptr) {
        return false;
    }
    diagnostics->encoder_angle_rad = native_to_manipulator_radians(g_fusion_encoder_angle_rad);
    diagnostics->tmc_angle_rad = native_to_manipulator_radians(g_fusion_tmc_angle_rad);
    diagnostics->offset_rad = native_to_manipulator_radians(g_fusion_offset_rad);
    diagnostics->fused_angle_rad = native_to_manipulator_radians(g_fused_angle_rad);
    diagnostics->encoder_error_rad = native_to_manipulator_radians(g_fusion_encoder_error_rad);
    diagnostics->backlash_rad = g_fusion_backlash_rad;
    diagnostics->innovation_rad = native_to_manipulator_radians(g_fusion_innovation_rad);
    diagnostics->applied_correction_rad =
        native_to_manipulator_radians(g_fusion_applied_correction_rad);
    diagnostics->slip_window_residual_rad =
        native_to_manipulator_radians(g_slip_window_residual_rad);
    diagnostics->rejected_spike_count = g_fusion_rejected_spike_count;
    diagnostics->persistent_residual_count = g_slip_persistent_residual_count;
    diagnostics->takeup_tmc_travel_rad = g_hybrid_takeup_tmc_travel_rad;
    diagnostics->takeup_encoder_travel_rad = g_hybrid_takeup_encoder_travel_rad;
    diagnostics->encoder_weight = g_fusion_encoder_weight;
    diagnostics->hybrid_state = g_hybrid_state;
    diagnostics->slip_candidate = g_slip_candidate;
    diagnostics->slip_latched =
        (g_fault_manager.latched_faults() & FaultMotorSlip) != 0U;
    diagnostics->calibrated_encoder = g_fusion_uses_calibrated_encoder;
    return true;
}

extern "C" bool motor_servo_get_diagnostics(motor_servo_diagnostics* const diagnostics)
{
    if (diagnostics == nullptr) {
        return false;
    }
    diagnostics->target_position_rad = g_servo_target_position_rad;
    diagnostics->position_error_rad = g_servo_position_error_rad;
    diagnostics->command_velocity_rad_s = g_servo_command_velocity_rad_s;
    diagnostics->command_age_ms = g_servo_target_valid
        ? HAL_GetTick() - g_servo_last_command_ms
        : 0U;
    diagnostics->state = g_servo_state;
    return true;
}

extern "C" bool motor_limit_get_diagnostics(motor_limit_diagnostics* const diagnostics)
{
    if (diagnostics == nullptr) {
        return false;
    }
    const JointLimitEnvelope limits = joint_limit_envelope();
    float minimum_velocity_rad_s = -kRobotJointProfile->maximum_direct_velocity_rad_s;
    float maximum_velocity_rad_s = kRobotJointProfile->maximum_direct_velocity_rad_s;
    joint_velocity_bounds(
        limits,
        motor_fused_angle_manipulator(),
        kRobotJointProfile->maximum_direct_velocity_rad_s,
        minimum_velocity_rad_s,
        maximum_velocity_rad_s);
    diagnostics->hard_lower_rad = limits.hard_lower_rad;
    diagnostics->soft_lower_rad = limits.soft_lower_rad;
    diagnostics->soft_upper_rad = limits.soft_upper_rad;
    diagnostics->hard_upper_rad = limits.hard_upper_rad;
    diagnostics->current_position_rad = motor_fused_angle_manipulator();
    diagnostics->minimum_velocity_rad_s = minimum_velocity_rad_s;
    diagnostics->maximum_velocity_rad_s = maximum_velocity_rad_s;
    diagnostics->active = limits.active;
    return true;
}

extern "C" bool motor_auto_calibration_start(void)
{
    cancel_calibration_jog();
    if (!g_fault_manager.motion_allowed() ||
        g_encoder_calibration.blocks_normal_control() ||
        !g_output_encoder_available) {
        return false;
    }
    clear_servo_target();
    restore_default_motion_profile();
    if (!tmc5160_stop(kServiceStopTimeoutMs)) {
        return false;
    }
    if (!g_tmc_driver.is_enabled() && !g_tmc_driver.enable()) {
        return false;
    }
    constexpr uint8_t kMaximumAutoCalibrationCurrent = 3U;
    const uint8_t calibration_current = std::min<uint8_t>(
        static_cast<uint8_t>(kRobotJointProfile->init_irun),
        kMaximumAutoCalibrationCurrent);
    tmc5160_set_run_current(calibration_current);
    if (!g_encoder_calibration.begin_auto(HAL_GetTick(), true, g_encoder_angle_raw)) {
        return false;
    }
    g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
    return true;
}

extern "C" bool motor_manual_calibration_command(const int32_t command)
{
    cancel_calibration_jog();
    if (command == 1) {
        if (!g_fault_manager.motion_allowed() || !g_output_encoder_available) {
            return false;
        }
        clear_servo_target();
        restore_default_motion_profile();
        if (!tmc5160_stop(kServiceStopTimeoutMs) ||
            (g_tmc_driver.is_enabled() && !g_tmc_driver.disable())) {
            return false;
        }
        if (!g_encoder_calibration.begin_manual(true, g_encoder_angle_raw)) {
            return false;
        }
        g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
        return true;
    }
    if ((command < 2) || (command > 5) || !g_output_encoder_available) {
        return false;
    }
    if (command == 5) {
        if (g_encoder_calibration.state() != EncoderCalibrationState::Ready) {
            return false;
        }
        constexpr uint8_t kMaximumManualCalibrationCurrent = 3U;
        const uint8_t calibration_current = std::min<uint8_t>(
            static_cast<uint8_t>(kRobotJointProfile->init_irun),
            kMaximumManualCalibrationCurrent);
        tmc5160_set_run_current(calibration_current);
        if (!g_tmc_driver.is_enabled() && !g_tmc_driver.enable()) {
            g_encoder_calibration.fail(EncoderCalibrationError::Driver);
            return false;
        }
    }
    return g_encoder_calibration.advance_manual(
        HAL_GetTick(), true, g_encoder_angle_raw, command);
}

extern "C" bool motor_backlash_calibration_start(void)
{
    if (!g_fault_manager.motion_allowed() ||
        g_encoder_calibration.blocks_normal_control() ||
        !g_output_encoder_available ||
        !g_encoder_calibration.has_stored_data()) {
        return false;
    }

    clear_servo_target();
    restore_default_motion_profile();
    if (!tmc5160_stop(kServiceStopTimeoutMs)) {
        return false;
    }
    if (!g_tmc_driver.is_enabled() && !g_tmc_driver.enable()) {
        return false;
    }
    if (!g_encoder_calibration.begin_backlash(
            HAL_GetTick(),
            true,
            g_encoder_angle_raw)) {
        return false;
    }

    constexpr uint8_t kMaximumBacklashCalibrationCurrent = 3U;
    const uint8_t calibration_current = std::min<uint8_t>(
        static_cast<uint8_t>(kRobotJointProfile->init_irun),
        kMaximumBacklashCalibrationCurrent);
    tmc5160_set_run_current(calibration_current);
    g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
    return true;
}

extern "C" bool motor_zero_calibrate(void)
{
    if (!g_fault_manager.motion_allowed() ||
        g_encoder_calibration.blocks_normal_control() ||
        !g_output_encoder_available ||
        !g_encoder_calibration.has_stored_data()) {
        return false;
    }

    clear_servo_target();
    if (!tmc5160_stop(kServiceStopTimeoutMs)) {
        return false;
    }
    if (!g_encoder_calibration.calibrate_zero(g_encoder_angle_raw)) {
        return false;
    }

    g_zero_enc_runtime = g_encoder_angle_raw;
    tmc5160_set_zero();
    sync_tmc_offset_to_encoder();
    reset_fusion_tracking(HAL_GetTick());
    g_control_mode = MOTOR_CONTROL_MODE_HOLD;
    return true;
}

extern "C" int32_t motor_calibration_state(void)
{
    return static_cast<int32_t>(g_encoder_calibration.state());
}

extern "C" int32_t motor_calibration_error(void)
{
    return static_cast<int32_t>(g_encoder_calibration.error());
}

extern "C" int32_t motor_calibration_progress(void)
{
    return g_encoder_calibration.progress_percent();
}

extern "C" void motor_calibration_data(
    int32_t* const values,
    const uint8_t capacity,
    uint8_t* const count)
{
    if (count != nullptr) {
        *count = 0U;
    }
    if ((values == nullptr) || (count == nullptr) || (capacity < 8U)) {
        return;
    }
    const encoder_calibration_data& data = g_encoder_calibration.data();
    int32_t measured_full_steps = 0;
    const int64_t safe_span_ticks =
        std::abs(static_cast<int64_t>(data.manual_span_ticks)) -
        (2LL * static_cast<int64_t>(data.safe_margin_ticks));
    if ((safe_span_ticks > 0) && (data.tmc_span_steps != 0)) {
        const int64_t scaled =
            (std::abs(static_cast<int64_t>(data.tmc_span_steps)) * 16384LL) /
            safe_span_ticks;
        measured_full_steps = static_cast<int32_t>(
            std::min<int64_t>(scaled, std::numeric_limits<int32_t>::max()));
    }
    values[0] = g_encoder_calibration.has_stored_data() ? 1 : 0;
    values[1] = data.manual_span_ticks;
    values[2] = data.tmc_span_steps;
    values[3] = measured_full_steps;
    values[4] = static_cast<int32_t>(kRobotJointProfile->joint_full_steps);
    values[5] = data.backlash_steps;
    values[6] = static_cast<int32_t>(data.point_count);
    values[7] = static_cast<int32_t>(data.safe_margin_ticks);
    *count = 8U;
}

extern "C" float motor_fused_angle_manipulator(void)
{
    return native_to_manipulator_radians(g_fused_angle_rad);
}

extern "C" float motor_fused_velocity_manipulator(void)
{
    return native_to_manipulator_radians(g_fused_velocity_rad_s);
}

extern "C" bool motor_ack_fail(void)
{
    const bool fusion_offset_fault =
        (g_fault_manager.latched_faults() & FaultFusionOffsetExceeded) != 0U;
    if (fusion_offset_fault &&
        (!g_output_encoder_available ||
         !g_fusion_uses_calibrated_encoder ||
         g_encoder_calibration.blocks_normal_control() ||
         (g_tmc_driver.error() != Tmc5160Error::None))) {
        return false;
    }
    const bool sync_offset = g_fault_manager.acknowledge();
    if (sync_offset) {
        sync_tmc_offset_to_encoder();
        reset_fusion_tracking(HAL_GetTick());
    }
    return true;
}

extern "C" int32_t motor_fail_level(void)
{
    return static_cast<int32_t>(g_fault_manager.level());
}

extern "C" bool motor_slip_latched(void)
{
    return (g_fault_manager.latched_faults() & FaultMotorSlip) != 0U;
}

extern "C" uint32_t motor_fault_active(void)
{
    return g_fault_manager.active_faults();
}

extern "C" uint32_t motor_fault_latched(void)
{
    return g_fault_manager.latched_faults();
}

extern "C" int32_t motor_network_state(void)
{
    return static_cast<int32_t>(g_fault_manager.network_state());
}

extern "C" int32_t motor_controller_state(void)
{
    return static_cast<int32_t>(g_fault_manager.controller_state());
}

extern "C" int32_t motor_stop_reason(void)
{
    return static_cast<int32_t>(g_fault_manager.stop_reason());
}

extern "C" bool motor_fault_log_last(fault_log_record* record)
{
    return (record != nullptr) && g_fault_manager.last_log_record(*record);
}
