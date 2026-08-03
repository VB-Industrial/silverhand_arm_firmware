#include "tmc5160_state.hpp"

#include "main.h"

namespace
{
constexpr uint32_t kGstatClearMask = 0x7UL;
constexpr uint32_t kEnableStabilizationDelayMs = 100U;
constexpr uint32_t kDisableDecelerationTimeoutMs = 250U;
constexpr uint32_t kStatusUpdatePeriodMs = 100U;
}  // namespace

bool Tmc5160StateMachine::initialize(const uint8_t initial_current)
{
    initial_current_ = initial_current;
    state_ = Tmc5160State::Disabled;
    return enable();
}

bool Tmc5160StateMachine::enable()
{
    if (state_ == Tmc5160State::Enabled) {
        return is_enabled();
    }
    if ((state_ != Tmc5160State::Disabled) && (state_ != Tmc5160State::Fault)) {
        return false;
    }

    state_ = Tmc5160State::Enabling;
    fault_snapshot_ = {};
    if (!configure_while_disabled()) {
        enter_fault(error_);
        return false;
    }

    // Never resume a velocity command that existed before disable/reset.
    tmc5160_move(0);
    tmc5160_clear_gstat(kGstatClearMask);
    tmc5160_arm();
    HAL_Delay(kEnableStabilizationDelayMs);

    if (!refresh_health_snapshot()) {
        enter_fault(Tmc5160Error::Communication);
        return false;
    }
    if (!driver_enabled_readback_) {
        enter_fault(Tmc5160Error::EnabledPinReadback);
        return false;
    }
    if ((fault_snapshot_.flags & TMC5160_CRITICAL_FAULT_MASK) != 0U) {
        enter_fault(Tmc5160Error::CriticalDriverStatus);
        return false;
    }

    state_ = Tmc5160State::Enabled;
    error_ = Tmc5160Error::None;
    last_status_update_ms_ = HAL_GetTick();
    return true;
}

bool Tmc5160StateMachine::disable()
{
    if (state_ == Tmc5160State::Disabled) {
        bool enabled = false;
        if (!tmc5160_read_driver_enabled(&enabled)) {
            enter_fault(Tmc5160Error::Communication);
            return false;
        }
        if (!enabled) {
            return true;
        }
        enter_fault(Tmc5160Error::DisabledPinReadback);
        return false;
    }

    // Debug/service disable: stop first, then remove phase current.
    tmc5160_move(0);
    const uint32_t start_ms = HAL_GetTick();
    while ((tmc5160_velocity_read() != 0) &&
           ((HAL_GetTick() - start_ms) < kDisableDecelerationTimeoutMs)) {
        HAL_Delay(1U);
    }

    tmc5160_disarm();
    HAL_Delay(1U);
    bool enabled = false;
    if (!tmc5160_read_driver_enabled(&enabled)) {
        enter_fault(Tmc5160Error::Communication);
        return false;
    }
    if (enabled) {
        enter_fault(Tmc5160Error::DisabledPinReadback);
        return false;
    }

    state_ = Tmc5160State::Disabled;
    error_ = Tmc5160Error::None;
    return true;
}

bool Tmc5160StateMachine::is_enabled() const
{
    return state_ == Tmc5160State::Enabled;
}

void Tmc5160StateMachine::update(const uint32_t now_ms)
{
    if ((state_ != Tmc5160State::Enabled) ||
        ((now_ms - last_status_update_ms_) < kStatusUpdatePeriodMs)) {
        return;
    }
    last_status_update_ms_ = now_ms;

    if (!refresh_health_snapshot()) {
        enter_fault(Tmc5160Error::Communication);
    } else if (!driver_enabled_readback_) {
        enter_fault(Tmc5160Error::EnabledPinReadback);
    } else if ((fault_snapshot_.flags & TMC5160_CRITICAL_FAULT_MASK) != 0U) {
        enter_fault(Tmc5160Error::CriticalDriverStatus);
    }
}

Tmc5160State Tmc5160StateMachine::state() const
{
    return state_;
}

Tmc5160Error Tmc5160StateMachine::error() const
{
    return error_;
}

const tmc5160_fault_snapshot& Tmc5160StateMachine::fault_snapshot() const
{
    return fault_snapshot_;
}

bool Tmc5160StateMachine::configure_while_disabled()
{
    tmc5160_init(static_cast<int8_t>(initial_current_));

    bool enabled = false;
    if (!tmc5160_read_driver_enabled(&enabled)) {
        error_ = Tmc5160Error::Communication;
        return false;
    }
    if (enabled) {
        error_ = Tmc5160Error::DisabledPinReadback;
        return false;
    }
    if (!tmc5160_configuration_matches()) {
        error_ = Tmc5160Error::ConfigurationReadback;
        return false;
    }
    return true;
}

bool Tmc5160StateMachine::refresh_health_snapshot()
{
    tmc5160_health_snapshot snapshot{};
    const bool success = tmc5160_health_check(&snapshot);
    fault_snapshot_ = snapshot.faults;
    driver_enabled_readback_ = snapshot.driver_enabled;
    return success;
}

void Tmc5160StateMachine::enter_fault(const Tmc5160Error error)
{
    // Fault disable is immediate; do not rely on SPI to stop a possibly broken driver.
    tmc5160_disarm();
    state_ = Tmc5160State::Fault;
    error_ = error;
}
