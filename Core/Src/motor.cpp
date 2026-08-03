#include "motor.h"

#include "main.h"
#include "robot_config.h"

#include <cmath>

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
constexpr uint8_t kEncoderZeroStreakThreshold = 3U;
constexpr uint8_t kEncoderValidStreakThreshold = 10U;
constexpr float kVelocityZeroThresholdRadS = 0.0001F;
constexpr float kLargePositionErrorThresholdRad = 5.0F * static_cast<float>(M_PI) / 180.0F;
constexpr int32_t kDefaultPositionVelocitySteps = 10000;
constexpr int32_t kFastPositionVelocitySteps = 30000;
constexpr int32_t kFullThrottlePositionVelocitySteps = 30000;

uint16_t g_encoder_angle_raw = 0U;
uint32_t g_zero_enc_runtime = 0U;
uint16_t g_prev_enc_angle = 0U;
uint32_t g_prev_fusion_ts_ms = 0U;
uint32_t g_last_degraded_led_toggle_ms = 0U;
uint8_t g_encoder_zero_streak = 0U;
uint8_t g_encoder_valid_streak = 0U;
float g_enc_velocity_lpf_rad_s = 0.0F;
float g_fused_angle_rad = 0.0F;
float g_fused_velocity_rad_s = 0.0F;
float g_startup_tmc_angle_offset_rad = 0.0F;
bool g_output_encoder_available = false;
bool g_output_encoder_degraded = false;

FaultManager g_fault_manager;
Tmc5160StateMachine g_tmc_driver;

float manipulator_to_native_radians(const float manipulator_angle_rad)
{
    return static_cast<float>(kRobotJointProfile->direction) * manipulator_angle_rad;
}

float native_to_manipulator_radians(const float native_angle_rad)
{
    return static_cast<float>(kRobotJointProfile->direction) * native_angle_rad;
}

float encoder_fused_angle_radians(void)
{
    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;

    int32_t delta_ticks = static_cast<int32_t>(g_encoder_angle_raw) - static_cast<int32_t>(g_zero_enc_runtime);
    if (delta_ticks > kHalfTurnTicks) {
        delta_ticks -= kEncoderTicksPerTurn;
    } else if (delta_ticks < -kHalfTurnTicks) {
        delta_ticks += kEncoderTicksPerTurn;
    }

    const float radians_per_tick = (2.0F * static_cast<float>(M_PI)) / static_cast<float>(kEncoderTicksPerTurn);
    return static_cast<float>(delta_ticks) * radians_per_tick;
}

float tmc_angle_radians(void)
{
    return steps_to_rads(tmc5160_position_read(), kRobotJointProfile->joint_full_steps);
}

float tmc_corrected_angle_radians(void)
{
    return tmc_angle_radians() + g_startup_tmc_angle_offset_rad;
}

float tmc_velocity_radians_per_second(void)
{
    return steps_to_rads(tmc5160_velocity_read(), kRobotJointProfile->joint_full_steps);
}

void sync_tmc_offset_to_encoder(void)
{
    g_startup_tmc_angle_offset_rad = 0.0F;
    if (g_output_encoder_available) {
        g_startup_tmc_angle_offset_rad = encoder_fused_angle_radians() - tmc_angle_radians();
    }
}

void reset_fusion_tracking(const uint32_t now_ms)
{
    g_prev_enc_angle = g_encoder_angle_raw;
    g_prev_fusion_ts_ms = now_ms;
    g_enc_velocity_lpf_rad_s = 0.0F;
    g_fused_angle_rad = tmc_corrected_angle_radians();
    g_fused_velocity_rad_s = 0.0F;
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
    const float tmc_angle = tmc_corrected_angle_radians();
    const float tmc_velocity = tmc_velocity_radians_per_second();

    if (!g_output_encoder_available) {
        g_fused_angle_rad = tmc_angle;
        g_fused_velocity_rad_s = tmc_velocity;
        g_prev_enc_angle = g_encoder_angle_raw;
        g_prev_fusion_ts_ms = now_ms;
        g_enc_velocity_lpf_rad_s = 0.0F;
        return;
    }

    const float encoder_angle = encoder_fused_angle_radians();
    g_fused_angle_rad = (kRobotJointProfile->angle_encoder_weight * encoder_angle) +
                        (kRobotJointProfile->angle_tmc_weight * tmc_angle);

    if (g_prev_fusion_ts_ms == 0U) {
        g_prev_enc_angle = g_encoder_angle_raw;
        g_prev_fusion_ts_ms = now_ms;
        g_fused_velocity_rad_s = tmc_velocity;
        return;
    }

    const uint32_t dt_ms = now_ms - g_prev_fusion_ts_ms;
    if (dt_ms == 0U) {
        g_fused_velocity_rad_s = tmc_velocity;
        return;
    }

    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;
    int32_t delta_ticks = static_cast<int32_t>(g_encoder_angle_raw) - static_cast<int32_t>(g_prev_enc_angle);
    if (delta_ticks > kHalfTurnTicks) {
        delta_ticks -= kEncoderTicksPerTurn;
    } else if (delta_ticks < -kHalfTurnTicks) {
        delta_ticks += kEncoderTicksPerTurn;
    }

    const float dt_s = static_cast<float>(dt_ms) / 1000.0F;
    const float radians_per_tick = (2.0F * static_cast<float>(M_PI)) / static_cast<float>(kEncoderTicksPerTurn);
    const float enc_velocity_raw = (static_cast<float>(delta_ticks) * radians_per_tick) / dt_s;
    const float lpf_alpha = kRobotJointProfile->velocity_encoder_lpf_alpha;
    g_enc_velocity_lpf_rad_s = (lpf_alpha * enc_velocity_raw) + ((1.0F - lpf_alpha) * g_enc_velocity_lpf_rad_s);

    g_fused_velocity_rad_s = (kRobotJointProfile->velocity_tmc_weight * tmc_velocity) +
                             (kRobotJointProfile->velocity_encoder_weight * g_enc_velocity_lpf_rad_s);

    g_prev_enc_angle = g_encoder_angle_raw;
    g_prev_fusion_ts_ms = now_ms;
}

int32_t manipulator_radians_to_tmc_steps(const float manipulator_angle_rad)
{
    const float native_target_angle_rad = manipulator_to_native_radians(manipulator_angle_rad);
    const float tmc_target_angle_rad = native_target_angle_rad - g_startup_tmc_angle_offset_rad;
    return rad_to_steps(tmc_target_angle_rad, kRobotJointProfile->joint_full_steps);
}

void update_encoder_status(const uint32_t now_ms)
{
    if (!kRobotJointProfile->has_output_encoder) {
        set_output_encoder_available(false, now_ms);
        return;
    }

    if (g_encoder_angle_raw == 0U) {
        g_encoder_valid_streak = 0U;
        if (g_encoder_zero_streak < 255U) {
            ++g_encoder_zero_streak;
        }
        if (g_output_encoder_available && (g_encoder_zero_streak >= kEncoderZeroStreakThreshold)) {
            set_output_encoder_available(false, now_ms);
        }
        return;
    }

    g_encoder_zero_streak = 0U;
    if (g_encoder_valid_streak < 255U) {
        ++g_encoder_valid_streak;
    }
    if (g_output_encoder_degraded && (g_encoder_valid_streak >= kEncoderValidStreakThreshold)) {
        set_output_encoder_available(true, now_ms);
    }
}

void update_faults(const uint32_t now_ms)
{
    const FaultManager::UpdateResult result = g_fault_manager.update(
        now_ms,
        g_output_encoder_available,
        encoder_fused_angle_radians(),
        tmc_corrected_angle_radians(),
        g_tmc_driver.error(),
        g_tmc_driver.fault_snapshot());

    if (result.stop_motion) {
        tmc5160_move(0);
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

    g_zero_enc_runtime = kRobotJointProfile->default_zero_enc;

    if (kRobotJointProfile->has_output_encoder) {
        as50_readAngle(&g_encoder_angle_raw, 100);
        set_output_encoder_available(g_encoder_angle_raw != 0U, now_ms);
    } else {
        g_encoder_angle_raw = 0U;
        set_output_encoder_available(false, now_ms);
    }

    g_encoder_zero_streak = (g_encoder_angle_raw == 0U) ? 1U : 0U;
    g_encoder_valid_streak = (g_encoder_angle_raw != 0U) ? 1U : 0U;
    sync_tmc_offset_to_encoder();
    reset_fusion_tracking(now_ms);
    g_last_degraded_led_toggle_ms = now_ms;
}

extern "C" void motor_update(const uint32_t now_ms)
{
    g_tmc_driver.update(now_ms);

    if (kRobotJointProfile->has_output_encoder) {
        as50_readAngle(&g_encoder_angle_raw, 100);
    } else {
        g_encoder_angle_raw = 0U;
    }

    update_encoder_status(now_ms);
    update_fusion_state(now_ms);
    update_faults(now_ms);
    update_degraded_led(now_ms);
}

extern "C" void motor_command(
    const float position_rad,
    const float velocity_rad_s,
    const float acceleration_rad_s2)
{
    if (!g_fault_manager.remote_motion_allowed() || !g_tmc_driver.is_enabled()) {
        return;
    }

    const int32_t target_position_steps = manipulator_radians_to_tmc_steps(position_rad);
    const float position_error_rad =
        angular_abs_diff_radians(position_rad, motor_fused_angle_manipulator());

    if (acceleration_rad_s2 != 0.0F) {
        tmc5160_move(rad_to_steps(acceleration_rad_s2, kRobotJointProfile->joint_full_steps));
    } else if ((fabsf(velocity_rad_s) < kVelocityZeroThresholdRadS) && (velocity_rad_s != 0.0F)) {
        tmc5160_velocity(kFastPositionVelocitySteps);
        tmc5160_position(target_position_steps, kFastPositionVelocitySteps);
    } else if (velocity_rad_s == 0.0F) {
        const int32_t position_velocity_steps =
            (position_error_rad > kLargePositionErrorThresholdRad)
                ? kFastPositionVelocitySteps
                : kFullThrottlePositionVelocitySteps;
        tmc5160_acceleration(kRobotJointProfile->joint_full_steps);
        tmc5160_velocity(position_velocity_steps);
        tmc5160_position(target_position_steps, position_velocity_steps);
    } else {
        tmc5160_velocity(kFastPositionVelocitySteps);
        tmc5160_position(target_position_steps, kFastPositionVelocitySteps);
    }
}

extern "C" void motor_move(const int32_t velocity_command)
{
    if (!g_fault_manager.motion_allowed() || !g_tmc_driver.is_enabled()) {
        return;
    }
    tmc5160_move(velocity_command);
}

extern "C" void motor_set_position_steps(const int32_t target_position_steps)
{
    if (!g_fault_manager.motion_allowed() || !g_tmc_driver.is_enabled()) {
        return;
    }
    tmc5160_apply_default_motion_profile();
    tmc5160_position(target_position_steps, kDefaultPositionVelocitySteps);
}

extern "C" bool motor_arm(const bool armed)
{
    if (armed) {
        const bool enabled = g_tmc_driver.enable();
        if (enabled) {
            sync_tmc_offset_to_encoder();
        }
        return enabled;
    }
    return g_tmc_driver.disable();
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

extern "C" void motor_note_heartbeat(const uint32_t now_ms)
{
    g_fault_manager.note_heartbeat(now_ms);
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
    const bool sync_offset = g_fault_manager.acknowledge();
    if (sync_offset) {
        sync_tmc_offset_to_encoder();
    }
    return true;
}

extern "C" int32_t motor_fail_level(void)
{
    return static_cast<int32_t>(g_fault_manager.level());
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

extern "C" int32_t motor_stop_reason(void)
{
    return static_cast<int32_t>(g_fault_manager.stop_reason());
}

extern "C" uint32_t motor_fault_log_count(void)
{
    return g_fault_manager.log_count();
}

extern "C" bool motor_fault_log_last(fault_log_record* record)
{
    return (record != nullptr) && g_fault_manager.last_log_record(*record);
}
