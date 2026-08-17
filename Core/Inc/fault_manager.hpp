#pragma once

#include <cstdint>

#include "fault_log.h"
#include "tmc5160_state.hpp"

enum Fault : uint32_t
{
    FaultNone = 0U,
    FaultMotorSlip = 1UL << 0U,
    FaultFusionOffsetExceeded = FaultMotorSlip,
    FaultTmcCommunication = 1UL << 1U,
    FaultTmcEnableReadback = 1UL << 2U,
    FaultTmcConfiguration = 1UL << 3U,
    FaultTmcUndervoltage = 1UL << 4U,
    FaultTmcOvertemperatureWarning = 1UL << 5U,
    FaultTmcOvertemperature = 1UL << 6U,
    FaultTmcShortCircuit = 1UL << 7U,
    FaultTmcCriticalStatus = 1UL << 8U,
    FaultEepromUnavailable = 1UL << 9U,
    FaultEepromWrite = 1UL << 10U,
    FaultCommandTimeout = 1UL << 11U,
};

enum class FaultLevel : int32_t
{
    Nominal = 0,
    Warning = 1,
    Degraded = 2,
    Fault = 3,
};

enum class NetworkState : int32_t
{
    Starting = 0,
    Online = 1,
    Offline = 2,
};

enum class StopReason : int32_t
{
    None = 0,
    NetworkOffline = 1,
    MotorSlip = 2,
    FusionOffsetExceeded = MotorSlip,
    TmcDriver = 3,
    ControllerOffline = 4,
    CommandTimeout = 5,
};

class FaultManager final
{
public:
    struct UpdateResult
    {
        bool stop_motion;
    };

    void initialize(uint32_t now_ms, uint8_t joint_id);
    void note_heartbeat(uint32_t now_ms, bool from_controller);
    void note_velocity_command(uint32_t now_ms, bool active);
    bool latch_motor_slip();
    UpdateResult update(
        uint32_t now_ms,
        bool fusion_offset_available,
        float fusion_offset_rad,
        float backlash_rad,
        Tmc5160Error tmc_error,
        const tmc5160_fault_snapshot& tmc_snapshot);

    bool acknowledge();
    bool motion_allowed() const;
    bool remote_motion_allowed() const;

    uint32_t active_faults() const;
    uint32_t latched_faults() const;
    FaultLevel level() const;
    NetworkState network_state() const;
    NetworkState controller_state() const;
    StopReason stop_reason() const;

    bool last_log_record(fault_log_record& record) const;

private:
    bool update_network(uint32_t now_ms);
    bool update_controller(uint32_t now_ms);
    bool update_command_watchdog(uint32_t now_ms);
    bool update_fusion_offset(
        uint32_t now_ms,
        bool available,
        float offset_rad,
        float backlash_rad);
    void update_tmc_faults(Tmc5160Error error, const tmc5160_fault_snapshot& snapshot);
    void update_persistent_log(uint32_t now_ms, const tmc5160_fault_snapshot& snapshot);
    void update_level();

    uint32_t active_faults_ = FaultNone;
    uint32_t latched_faults_ = FaultNone;
    uint32_t previous_persistent_faults_ = FaultNone;
    FaultLevel level_ = FaultLevel::Nominal;
    NetworkState network_state_ = NetworkState::Starting;
    NetworkState controller_state_ = NetworkState::Starting;
    StopReason stop_reason_ = StopReason::None;

    uint32_t startup_ms_ = 0U;
    uint32_t last_heartbeat_ms_ = 0U;
    uint32_t last_controller_heartbeat_ms_ = 0U;
    uint32_t last_velocity_command_ms_ = 0U;
    uint32_t fusion_offset_exceeded_since_ms_ = 0U;
    bool heartbeat_seen_ = false;
    bool controller_heartbeat_seen_ = false;
    bool velocity_command_active_ = false;
    bool fusion_offset_exceeded_ = false;
    bool motor_slip_fault_latched_ = false;
    bool eeprom_write_failed_ = false;
};
