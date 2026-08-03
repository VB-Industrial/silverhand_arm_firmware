#include "fault_manager.hpp"

#include "utility.h"

namespace
{
constexpr float kPositionMismatchThresholdRad = 10.0F * static_cast<float>(M_PI) / 180.0F;
constexpr uint8_t kPositionMismatchFaultCount = 3U;
constexpr uint32_t kNetworkStartupGraceMs = 5000U;
constexpr uint32_t kHeartbeatTimeoutMs = 2500U;
constexpr uint32_t kVelocityCommandTimeoutMs = 1000U;
// TODO: Enable after output-encoder calibration is applied.
constexpr bool kPositionMismatchCheckEnabled = false;

constexpr uint32_t kTmcFaultMask =
    FaultTmcCommunication |
    FaultTmcEnableReadback |
    FaultTmcConfiguration |
    FaultTmcUndervoltage |
    FaultTmcOvertemperatureWarning |
    FaultTmcOvertemperature |
    FaultTmcShortCircuit |
    FaultTmcCriticalStatus;

constexpr uint32_t kFaultLevelMask =
    FaultTmcCommunication |
    FaultTmcEnableReadback |
    FaultTmcConfiguration |
    FaultTmcUndervoltage |
    FaultTmcOvertemperature |
    FaultTmcShortCircuit |
    FaultTmcCriticalStatus;

constexpr uint32_t kPersistentFaultMask =
    FaultTmcOvertemperature |
    FaultTmcShortCircuit;
}  // namespace

void FaultManager::initialize(const uint32_t now_ms, const uint8_t joint_id)
{
    startup_ms_ = now_ms;
    network_state_ = NetworkState::Starting;
    controller_state_ = NetworkState::Starting;
    fault_log_init(joint_id);
    if (!fault_log_available()) {
        active_faults_ |= FaultEepromUnavailable;
        latched_faults_ |= FaultEepromUnavailable;
    }
    update_level();
}

void FaultManager::note_heartbeat(const uint32_t now_ms, const bool from_controller)
{
    heartbeat_seen_ = true;
    last_heartbeat_ms_ = now_ms;
    network_state_ = NetworkState::Online;

    if (from_controller) {
        controller_heartbeat_seen_ = true;
        last_controller_heartbeat_ms_ = now_ms;
        controller_state_ = NetworkState::Online;
    }
}

void FaultManager::note_velocity_command(const uint32_t now_ms, const bool active)
{
    velocity_command_active_ = active;
    last_velocity_command_ms_ = now_ms;
}

FaultManager::UpdateResult FaultManager::update(
    const uint32_t now_ms,
    const bool has_output_encoder,
    const float encoder_angle_rad,
    const float tmc_angle_rad,
    const Tmc5160Error tmc_error,
    const tmc5160_fault_snapshot& tmc_snapshot)
{
    UpdateResult result{false};

    if (update_network(now_ms)) {
        result.stop_motion = true;
        if (((active_faults_ & kFaultLevelMask) == 0U) && !mismatch_fault_latched_) {
            stop_reason_ = StopReason::NetworkOffline;
        }
    }
    if (update_controller(now_ms)) {
        result.stop_motion = true;
        if (((active_faults_ & kFaultLevelMask) == 0U) && !mismatch_fault_latched_) {
            stop_reason_ = StopReason::ControllerOffline;
        }
    }
    if (update_command_watchdog(now_ms)) {
        result.stop_motion = true;
        latched_faults_ |= FaultCommandTimeout;
        stop_reason_ = StopReason::CommandTimeout;
    }
    if (kPositionMismatchCheckEnabled &&
        update_position_mismatch(has_output_encoder, encoder_angle_rad, tmc_angle_rad)) {
        result.stop_motion = true;
        stop_reason_ = StopReason::PositionMismatch;
    }

    const uint32_t previous_tmc_faults = active_faults_ & kFaultLevelMask;
    update_tmc_faults(tmc_error, tmc_snapshot);
    if (((active_faults_ & kFaultLevelMask) != 0U) && (previous_tmc_faults == 0U)) {
        stop_reason_ = StopReason::TmcDriver;
    }

    if (!fault_log_available()) {
        active_faults_ |= FaultEepromUnavailable;
    }
    if (eeprom_write_failed_) {
        active_faults_ |= FaultEepromWrite;
    }

    latched_faults_ |= active_faults_;
    update_persistent_log(now_ms, tmc_snapshot);
    update_level();
    if (result.stop_motion) {
        velocity_command_active_ = false;
    }
    return result;
}

bool FaultManager::acknowledge()
{
    const bool sync_offset = (latched_faults_ & FaultPositionMismatch) != 0U;
    if (sync_offset) {
        active_faults_ &= ~FaultPositionMismatch;
        mismatch_warning_count_ = 0U;
        mismatch_active_ = false;
        mismatch_fault_latched_ = false;
    }

    latched_faults_ &= active_faults_;
    update_level();
    return sync_offset;
}

bool FaultManager::motion_allowed() const
{
    return !mismatch_fault_latched_;
}

bool FaultManager::remote_motion_allowed() const
{
    return motion_allowed() && (controller_state_ == NetworkState::Online);
}

uint32_t FaultManager::active_faults() const
{
    return active_faults_;
}

uint32_t FaultManager::latched_faults() const
{
    return latched_faults_;
}

FaultLevel FaultManager::level() const
{
    return level_;
}

NetworkState FaultManager::network_state() const
{
    return network_state_;
}

NetworkState FaultManager::controller_state() const
{
    return controller_state_;
}

StopReason FaultManager::stop_reason() const
{
    return stop_reason_;
}

uint32_t FaultManager::log_count() const
{
    return fault_log_count();
}

bool FaultManager::last_log_record(fault_log_record& record) const
{
    return fault_log_get_last(&record);
}

bool FaultManager::update_network(const uint32_t now_ms)
{
    const NetworkState previous_state = network_state_;
    if (heartbeat_seen_) {
        if ((now_ms - last_heartbeat_ms_) >= kHeartbeatTimeoutMs) {
            network_state_ = NetworkState::Offline;
        }
    } else if ((now_ms - startup_ms_) >= kNetworkStartupGraceMs) {
        network_state_ = NetworkState::Offline;
    }
    return (previous_state != NetworkState::Offline) && (network_state_ == NetworkState::Offline);
}

bool FaultManager::update_controller(const uint32_t now_ms)
{
    if (!controller_heartbeat_seen_) {
        return false;
    }

    const NetworkState previous_state = controller_state_;
    if ((now_ms - last_controller_heartbeat_ms_) >= kHeartbeatTimeoutMs) {
        controller_state_ = NetworkState::Offline;
    }
    return (previous_state == NetworkState::Online) &&
           (controller_state_ == NetworkState::Offline);
}

bool FaultManager::update_command_watchdog(const uint32_t now_ms)
{
    if (!velocity_command_active_ ||
        ((now_ms - last_velocity_command_ms_) < kVelocityCommandTimeoutMs)) {
        return false;
    }

    velocity_command_active_ = false;
    return true;
}

bool FaultManager::update_position_mismatch(
    const bool available,
    const float encoder_angle_rad,
    const float tmc_angle_rad)
{
    if (!available) {
        active_faults_ &= ~FaultPositionMismatch;
        mismatch_active_ = false;
        return false;
    }

    const float mismatch_rad = angular_abs_diff_radians(encoder_angle_rad, tmc_angle_rad);
    if (mismatch_rad <= kPositionMismatchThresholdRad) {
        active_faults_ &= ~FaultPositionMismatch;
        mismatch_active_ = false;
        return false;
    }

    active_faults_ |= FaultPositionMismatch;
    if (mismatch_active_) {
        return false;
    }

    mismatch_active_ = true;
    if (mismatch_warning_count_ < kPositionMismatchFaultCount) {
        ++mismatch_warning_count_;
    }
    if (mismatch_warning_count_ >= kPositionMismatchFaultCount) {
        mismatch_fault_latched_ = true;
        return true;
    }
    return false;
}

void FaultManager::update_tmc_faults(
    const Tmc5160Error error,
    const tmc5160_fault_snapshot& snapshot)
{
    active_faults_ &= ~kTmcFaultMask;

    switch (error) {
    case Tmc5160Error::Communication:
        active_faults_ |= FaultTmcCommunication;
        break;
    case Tmc5160Error::DisabledPinReadback:
    case Tmc5160Error::EnabledPinReadback:
        active_faults_ |= FaultTmcEnableReadback;
        break;
    case Tmc5160Error::ConfigurationReadback:
        active_faults_ |= FaultTmcConfiguration;
        break;
    case Tmc5160Error::CriticalDriverStatus:
        if ((snapshot.flags & TMC5160_CRITICAL_FAULT_MASK) == 0U) {
            active_faults_ |= FaultTmcCriticalStatus;
        }
        break;
    case Tmc5160Error::None:
        break;
    }

    if ((snapshot.flags & TMC5160_FAULT_UNDERVOLTAGE) != 0U) {
        active_faults_ |= FaultTmcUndervoltage;
    }
    if ((snapshot.flags & TMC5160_FAULT_OVERTEMP_WARNING) != 0U) {
        active_faults_ |= FaultTmcOvertemperatureWarning;
    }
    if ((snapshot.flags & TMC5160_FAULT_OVERTEMP) != 0U) {
        active_faults_ |= FaultTmcOvertemperature;
    }
    if ((snapshot.flags & (TMC5160_FAULT_SHORT_TO_GROUND | TMC5160_FAULT_SHORT_TO_SUPPLY)) != 0U) {
        active_faults_ |= FaultTmcShortCircuit;
    }
    const uint32_t detailed_driver_faults =
        TMC5160_FAULT_OVERTEMP |
        TMC5160_FAULT_SHORT_TO_GROUND |
        TMC5160_FAULT_SHORT_TO_SUPPLY;
    if (((snapshot.flags & TMC5160_FAULT_DRIVER_ERROR) != 0U) &&
        ((snapshot.flags & detailed_driver_faults) == 0U)) {
        active_faults_ |= FaultTmcCriticalStatus;
    }
}

void FaultManager::update_persistent_log(
    const uint32_t now_ms,
    const tmc5160_fault_snapshot& snapshot)
{
    const uint32_t persistent_faults = active_faults_ & kPersistentFaultMask;
    const uint32_t new_faults = persistent_faults & ~previous_persistent_faults_;
    previous_persistent_faults_ = persistent_faults;

    if ((new_faults != 0U) &&
        !fault_log_append(now_ms, new_faults, snapshot.gstat, snapshot.driver_status)) {
        eeprom_write_failed_ = true;
        active_faults_ |= FaultEepromWrite;
        latched_faults_ |= FaultEepromWrite;
    }
}

void FaultManager::update_level()
{
    if (((active_faults_ & kFaultLevelMask) != 0U) || mismatch_fault_latched_) {
        level_ = FaultLevel::Fault;
    } else if ((network_state_ == NetworkState::Offline) ||
               (controller_state_ == NetworkState::Offline) ||
               ((active_faults_ & (FaultEepromUnavailable | FaultEepromWrite)) != 0U)) {
        level_ = FaultLevel::Degraded;
    } else if (((latched_faults_ & (FaultPositionMismatch | FaultCommandTimeout)) != 0U) ||
               ((active_faults_ & FaultTmcOvertemperatureWarning) != 0U)) {
        level_ = FaultLevel::Warning;
    } else {
        level_ = FaultLevel::Nominal;
    }
}
