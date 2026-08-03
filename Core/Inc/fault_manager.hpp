#pragma once

#include <cstdint>

#include "fault_log.h"
#include "tmc5160_state.hpp"

enum Fault : uint32_t
{
    FaultNone = 0U,
    FaultPositionMismatch = 1UL << 0U,
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
    PositionMismatch = 2,
    TmcDriver = 3,
};

class FaultManager final
{
public:
    struct UpdateResult
    {
        bool stop_motion;
    };

    void initialize(uint32_t now_ms, uint8_t joint_id);
    void note_heartbeat(uint32_t now_ms);
    UpdateResult update(
        uint32_t now_ms,
        bool has_output_encoder,
        float encoder_angle_rad,
        float tmc_angle_rad,
        Tmc5160Error tmc_error,
        const tmc5160_fault_snapshot& tmc_snapshot);

    bool acknowledge();
    bool motion_allowed() const;
    bool remote_motion_allowed() const;

    uint32_t active_faults() const;
    uint32_t latched_faults() const;
    FaultLevel level() const;
    NetworkState network_state() const;
    StopReason stop_reason() const;

    uint32_t log_count() const;
    bool last_log_record(fault_log_record& record) const;

private:
    bool update_network(uint32_t now_ms);
    bool update_position_mismatch(bool available, float encoder_angle_rad, float tmc_angle_rad);
    void update_tmc_faults(Tmc5160Error error, const tmc5160_fault_snapshot& snapshot);
    void update_persistent_log(uint32_t now_ms, const tmc5160_fault_snapshot& snapshot);
    void update_level();

    uint32_t active_faults_ = FaultNone;
    uint32_t latched_faults_ = FaultNone;
    uint32_t previous_persistent_faults_ = FaultNone;
    FaultLevel level_ = FaultLevel::Nominal;
    NetworkState network_state_ = NetworkState::Starting;
    StopReason stop_reason_ = StopReason::None;

    uint32_t startup_ms_ = 0U;
    uint32_t last_heartbeat_ms_ = 0U;
    uint8_t mismatch_warning_count_ = 0U;
    bool heartbeat_seen_ = false;
    bool mismatch_active_ = false;
    bool mismatch_fault_latched_ = false;
    bool eeprom_write_failed_ = false;
};
