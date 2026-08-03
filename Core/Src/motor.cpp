#include "motor.h"

#include "main.h"
#include "robot_config.h"

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
constexpr float kVelocityZeroThresholdRadS = 0.0001F;
constexpr float kLargePositionErrorThresholdRad = 5.0F * static_cast<float>(M_PI) / 180.0F;
constexpr int32_t kDefaultPositionVelocitySteps = 10000;
constexpr int32_t kFastPositionVelocitySteps = 30000;
constexpr int32_t kFullThrottlePositionVelocitySteps = 30000;
constexpr uint32_t kZeroCalibrationStopTimeoutMs = 250U;

uint16_t g_encoder_angle_raw = 0U;
uint32_t g_zero_enc_runtime = 0U;
uint16_t g_prev_enc_angle = 0U;
uint32_t g_prev_fusion_ts_ms = 0U;
uint32_t g_last_degraded_led_toggle_ms = 0U;
uint8_t g_encoder_error_streak = 0U;
uint8_t g_encoder_valid_streak = 0U;
float g_enc_velocity_lpf_rad_s = 0.0F;
float g_fused_angle_rad = 0.0F;
float g_fused_velocity_rad_s = 0.0F;
float g_startup_tmc_angle_offset_rad = 0.0F;
bool g_output_encoder_available = false;
bool g_output_encoder_degraded = false;
bool g_encoder_last_read_ok = false;
motor_control_mode g_control_mode = MOTOR_CONTROL_MODE_HOLD;

FaultManager g_fault_manager;
Tmc5160StateMachine g_tmc_driver;
EncoderCalibration g_encoder_calibration;

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
    delta_ticks = apply_output_encoder_direction(delta_ticks);

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
    delta_ticks = apply_output_encoder_direction(delta_ticks);

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

bool manipulator_radians_to_tmc_steps(
    const float manipulator_angle_rad,
    int32_t& target_position_steps)
{
    const float native_target_angle_rad = manipulator_to_native_radians(manipulator_angle_rad);
    const float tmc_target_angle_rad = native_target_angle_rad - g_startup_tmc_angle_offset_rad;
    return radians_to_steps_checked(
        tmc_target_angle_rad,
        static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
        &target_position_steps);
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
    const FaultManager::UpdateResult result = g_fault_manager.update(
        now_ms,
        g_output_encoder_available,
        encoder_fused_angle_radians(),
        tmc_corrected_angle_radians(),
        g_tmc_driver.error(),
        g_tmc_driver.fault_snapshot());

    if (result.stop_motion) {
        tmc5160_move(0);
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
        (calibration_state == EncoderCalibrationState::Processing) ||
        (calibration_state == EncoderCalibrationState::Saving) ||
        (calibration_state == EncoderCalibrationState::AutoSeekLimitA) ||
        (calibration_state == EncoderCalibrationState::AutoBackoffA) ||
        (calibration_state == EncoderCalibrationState::AutoSeekLimitB);
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
    g_control_mode = MOTOR_CONTROL_MODE_HOLD;

    g_zero_enc_runtime = kRobotJointProfile->default_zero_enc;

    if (kRobotJointProfile->has_output_encoder) {
        g_encoder_last_read_ok = as50_readAngle(&g_encoder_angle_raw, 100);
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
    } else {
        g_encoder_angle_raw = 0U;
        g_encoder_last_read_ok = false;
    }

    update_encoder_status(now_ms);
    update_calibration(now_ms);
    update_fusion_state(now_ms);
    update_faults(now_ms);
    update_degraded_led(now_ms);
}

extern "C" bool motor_command(
    const float position_rad,
    const float velocity_rad_s,
    const float acceleration_rad_s2)
{
    if (!std::isfinite(position_rad) ||
        !std::isfinite(velocity_rad_s) ||
        !std::isfinite(acceleration_rad_s2)) {
        return false;
    }

    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.remote_motion_allowed() || !g_tmc_driver.is_enabled()) {
        return false;
    }

    int32_t target_position_steps = 0;
    if (!manipulator_radians_to_tmc_steps(position_rad, target_position_steps)) {
        return false;
    }

    int32_t direct_velocity_steps = 0;
    if ((acceleration_rad_s2 != 0.0F) &&
        !radians_to_steps_checked(
            acceleration_rad_s2,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &direct_velocity_steps)) {
        return false;
    }

    g_fault_manager.note_velocity_command(HAL_GetTick(), false);
    const float position_error_rad =
        angular_abs_diff_radians(position_rad, motor_fused_angle_manipulator());

    if (acceleration_rad_s2 != 0.0F) {
        tmc5160_move(direct_velocity_steps);
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
    g_control_mode = MOTOR_CONTROL_MODE_SERVO;
    return true;
}

extern "C" bool motor_move(const int32_t velocity_command)
{
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() || !g_tmc_driver.is_enabled()) {
        return false;
    }
    tmc5160_move(velocity_command);
    g_fault_manager.note_velocity_command(HAL_GetTick(), velocity_command != 0);
    g_control_mode = (velocity_command == 0)
        ? MOTOR_CONTROL_MODE_HOLD
        : MOTOR_CONTROL_MODE_DIRECT;
    return true;
}

extern "C" bool motor_move_radians_per_second(const float velocity_rad_s)
{
    if (!std::isfinite(velocity_rad_s) ||
        !g_fault_manager.remote_motion_allowed() ||
        !g_tmc_driver.is_enabled()) {
        return false;
    }

    const float native_velocity_rad_s = manipulator_to_native_radians(velocity_rad_s);
    int32_t velocity_steps = 0;
    if (!radians_to_steps_checked(
            native_velocity_rad_s,
            static_cast<int32_t>(kRobotJointProfile->joint_full_steps),
            &velocity_steps)) {
        return false;
    }

    return motor_move(velocity_steps);
}

extern "C" void motor_set_position_steps(const int32_t target_position_steps)
{
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_fault_manager.motion_allowed() || !g_tmc_driver.is_enabled()) {
        return;
    }
    g_fault_manager.note_velocity_command(HAL_GetTick(), false);
    tmc5160_apply_default_motion_profile();
    tmc5160_position(target_position_steps, kDefaultPositionVelocitySteps);
    g_control_mode = MOTOR_CONTROL_MODE_SERVO;
}

extern "C" bool motor_arm(const bool armed)
{
    if (g_encoder_calibration.blocks_normal_control()) {
        return false;
    }
    if (armed) {
        const bool enabled = g_tmc_driver.enable();
        if (enabled) {
            sync_tmc_offset_to_encoder();
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
    diagnostics->last_read_ok = encoder_diagnostics.last_read_ok;
    diagnostics->has_valid_angle = encoder_diagnostics.has_valid_angle;
    return true;
}

extern "C" bool motor_calibration_command(const int32_t command)
{
    if (command == 1) {
        tmc5160_move(0);
        if (!g_tmc_driver.disable()) {
            g_encoder_calibration.fail(EncoderCalibrationError::Driver);
            return false;
        }
        const bool started = g_encoder_calibration.begin(g_output_encoder_available, g_encoder_angle_raw);
        if (started) {
            g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
        }
        return started;
    }
    if (command == 2) {
        tmc5160_move(0);
        g_encoder_calibration.abort();
        if (g_tmc_driver.is_enabled()) {
            g_tmc_driver.disable();
        }
        g_control_mode = MOTOR_CONTROL_MODE_CALIBRATION;
        return true;
    }
    return false;
}

extern "C" bool motor_calibration_next(void)
{
    const bool starts_motion = g_encoder_calibration.state() == EncoderCalibrationState::Ready;
    if (!g_encoder_calibration.advance(HAL_GetTick(), g_output_encoder_available, g_encoder_angle_raw)) {
        return false;
    }
    if (starts_motion && !g_tmc_driver.enable()) {
        g_encoder_calibration.fail(EncoderCalibrationError::Driver);
        return false;
    }
    if (starts_motion) {
        constexpr uint8_t kMaximumCalibrationCurrent = 5U;
        const uint8_t calibration_current = std::min<uint8_t>(
            static_cast<uint8_t>(kRobotJointProfile->init_irun),
            kMaximumCalibrationCurrent);
        tmc5160_set_run_current(calibration_current);
    }
    return true;
}

extern "C" bool motor_auto_calibration_start(void)
{
    if (g_encoder_calibration.blocks_normal_control() || !g_output_encoder_available) {
        return false;
    }
    tmc5160_move(0);
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

extern "C" bool motor_zero_calibrate(void)
{
    if (g_encoder_calibration.blocks_normal_control() ||
        !g_output_encoder_available ||
        !g_encoder_calibration.has_stored_data()) {
        return false;
    }

    tmc5160_move(0);
    const uint32_t stop_started_ms = HAL_GetTick();
    while ((tmc5160_velocity_read() != 0) &&
           ((HAL_GetTick() - stop_started_ms) < kZeroCalibrationStopTimeoutMs)) {
        HAL_Delay(1U);
    }
    if (tmc5160_velocity_read() != 0) {
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

extern "C" void motor_calibration_result(int32_t* const values, const uint8_t capacity, uint8_t* const count)
{
    if ((values == nullptr) || (count == nullptr) || (capacity < 12U)) {
        return;
    }
    const encoder_calibration_data& data = g_encoder_calibration.data();
    values[0] = motor_calibration_state();
    values[1] = motor_calibration_error();
    values[2] = motor_calibration_progress();
    values[3] = data.limit_a_raw;
    values[4] = data.limit_b_raw;
    values[5] = data.manual_span_ticks;
    values[6] = data.safe_margin_ticks;
    values[7] = data.point_count;
    values[8] = data.tmc_span_steps;
    values[9] = data.zero_valid;
    values[10] = data.zero_raw;
    values[11] = g_encoder_calibration.manual_total_travel();
    *count = 12U;
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

extern "C" int32_t motor_controller_state(void)
{
    return static_cast<int32_t>(g_fault_manager.controller_state());
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
