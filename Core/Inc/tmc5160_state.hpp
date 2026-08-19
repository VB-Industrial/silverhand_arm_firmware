#pragma once

#include <cstdint>

extern "C" {
#include "tmc5160.h"
}

enum class Tmc5160State : int32_t
{
    Uninitialized = 0,
    Disabled = 1,
    Enabling = 2,
    Enabled = 3,
    Fault = 4,
};

enum class Tmc5160Error : int32_t
{
    None = 0,
    Communication = 1,
    DisabledPinReadback = 2,
    ConfigurationReadback = 3,
    EnabledPinReadback = 4,
    CriticalDriverStatus = 5,
};

class Tmc5160StateMachine final
{
public:
    bool initialize(uint8_t initial_current);
    bool enable();
    bool disable();
    bool is_enabled() const;
    void update(uint32_t now_ms);

    Tmc5160State state() const;
    Tmc5160Error error() const;
    const tmc5160_fault_snapshot& fault_snapshot() const;
    uint32_t health_read_failure_count() const;
    uint32_t enable_readback_mismatch_count() const;
    uint32_t critical_status_count() const;
    uint32_t initialize_count() const;
    uint32_t enable_count() const;
    uint32_t disable_count() const;

private:
    bool configure_and_enable();
    bool enable_preserving_configuration();
    bool refresh_health_snapshot();
    void enter_fault(Tmc5160Error error);

    Tmc5160State state_ = Tmc5160State::Uninitialized;
    Tmc5160Error error_ = Tmc5160Error::None;
    uint8_t initial_current_ = 0U;
    uint32_t last_status_update_ms_ = 0U;
    uint32_t last_enable_attempt_ms_ = 0U;
    uint32_t health_read_failure_count_ = 0U;
    uint32_t enable_readback_mismatch_count_ = 0U;
    uint32_t critical_status_count_ = 0U;
    uint32_t initialize_count_ = 0U;
    uint32_t enable_count_ = 0U;
    uint32_t disable_count_ = 0U;
    tmc5160_fault_snapshot fault_snapshot_{};
    uint8_t health_read_failure_streak_ = 0U;
    uint8_t enable_readback_mismatch_streak_ = 0U;
    uint8_t critical_status_streak_ = 0U;
    bool cold_initialized_ = false;
    bool driver_enabled_readback_ = false;
};
