#pragma once

#include <cstdint>

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
    bool is_enabled();
    void update(uint32_t now_ms);

    Tmc5160State state() const;
    Tmc5160Error error() const;

private:
    bool configure_while_disabled();
    bool verify_driver_enable_pin(bool expected_enabled) const;
    void enter_fault(Tmc5160Error error);

    Tmc5160State state_ = Tmc5160State::Uninitialized;
    Tmc5160Error error_ = Tmc5160Error::None;
    uint8_t initial_current_ = 0U;
    uint32_t last_status_update_ms_ = 0U;
};
