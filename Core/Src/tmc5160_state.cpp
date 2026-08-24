#include "tmc5160_state.hpp"

#include "main.h"

namespace
{
constexpr uint32_t kGstatClearMask = 0x7UL;
constexpr uint32_t kDisableDecelerationTimeoutMs = 250U;
constexpr uint32_t kStatusUpdatePeriodMs = 100U;
constexpr uint32_t kEnableRetryPeriodMs = 500U;
constexpr uint8_t kHealthReadFailureThreshold = 3U;
constexpr uint8_t kEnableReadbackMismatchThreshold = 3U;
constexpr uint8_t kCriticalStatusThreshold = 2U;
}  // namespace

bool Tmc5160StateMachine::initialize(const uint8_t initial_hold_current,
                                     const uint8_t initial_run_current)
{
    initialize_count_++;
    initial_hold_current_ = initial_hold_current;
    initial_run_current_ = initial_run_current;
    health_read_failure_count_ = 0U;
    enable_readback_mismatch_count_ = 0U;
    critical_status_count_ = 0U;
    health_read_failure_streak_ = 0U;
    enable_readback_mismatch_streak_ = 0U;
    critical_status_streak_ = 0U;
    cold_initialized_ = false;
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
    last_enable_attempt_ms_ = HAL_GetTick();
    fault_snapshot_ = {};
    const bool enabled = cold_initialized_
        ? enable_preserving_configuration()
        : configure_and_enable();
    if (!enabled) {
        enter_fault(error_);
        return false;
    }

    tmc5160_clear_gstat(kGstatClearMask);

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
    enable_count_++;
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
    if (!tmc5160_stop(kDisableDecelerationTimeoutMs)) {
        enter_fault(Tmc5160Error::CriticalDriverStatus);
        return false;
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
    disable_count_++;
    error_ = Tmc5160Error::None;
    return true;
}

bool Tmc5160StateMachine::is_enabled() const
{
    return state_ == Tmc5160State::Enabled;
}

void Tmc5160StateMachine::update(const uint32_t now_ms)
{
    if (state_ == Tmc5160State::Fault) {
        if ((now_ms - last_enable_attempt_ms_) >= kEnableRetryPeriodMs) {
            enable();
        }
        return;
    }
    if ((state_ != Tmc5160State::Enabled) ||
        ((now_ms - last_status_update_ms_) < kStatusUpdatePeriodMs)) {
        return;
    }
    last_status_update_ms_ = now_ms;

    if (!refresh_health_snapshot()) {
        ++health_read_failure_count_;
        enable_readback_mismatch_streak_ = 0U;
        critical_status_streak_ = 0U;
        if (++health_read_failure_streak_ >= kHealthReadFailureThreshold) {
            // A failed health poll does not prove that the power stage is
            // unsafe. Keep phase current applied and expose the communication
            // error diagnostically; a later valid poll clears the active error.
            error_ = Tmc5160Error::Communication;
        }
        return;
    }

    health_read_failure_streak_ = 0U;
    if (error_ == Tmc5160Error::Communication) {
        error_ = Tmc5160Error::None;
    }
    if (!driver_enabled_readback_) {
        ++enable_readback_mismatch_count_;
        critical_status_streak_ = 0U;
        if (++enable_readback_mismatch_streak_ >= kEnableReadbackMismatchThreshold) {
            enter_fault(Tmc5160Error::EnabledPinReadback);
        }
        return;
    }

    enable_readback_mismatch_streak_ = 0U;
    if ((fault_snapshot_.flags & TMC5160_CRITICAL_FAULT_MASK) != 0U) {
        ++critical_status_count_;
        if (++critical_status_streak_ >= kCriticalStatusThreshold) {
            enter_fault(Tmc5160Error::CriticalDriverStatus);
        }
    } else {
        critical_status_streak_ = 0U;
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

uint32_t Tmc5160StateMachine::health_read_failure_count() const
{
    return health_read_failure_count_;
}

uint32_t Tmc5160StateMachine::enable_readback_mismatch_count() const
{
    return enable_readback_mismatch_count_;
}

uint32_t Tmc5160StateMachine::critical_status_count() const
{
    return critical_status_count_;
}

uint32_t Tmc5160StateMachine::initialize_count() const { return initialize_count_; }
uint32_t Tmc5160StateMachine::enable_count() const { return enable_count_; }
uint32_t Tmc5160StateMachine::disable_count() const { return disable_count_; }

bool Tmc5160StateMachine::configure_and_enable()
{
    if (!tmc5160_init(initial_hold_current_, initial_run_current_)) {
        error_ = Tmc5160Error::Communication;
        return false;
    }
    cold_initialized_ = true;

    bool enabled = false;
    if (!tmc5160_read_driver_enabled(&enabled)) {
        error_ = Tmc5160Error::Communication;
        return false;
    }
    if (!enabled) {
        error_ = Tmc5160Error::EnabledPinReadback;
        return false;
    }
    if (!tmc5160_configuration_matches()) {
        error_ = Tmc5160Error::ConfigurationReadback;
        return false;
    }
    return true;
}

bool Tmc5160StateMachine::enable_preserving_configuration()
{
    // Runtime recovery must not replay cold-start register writes. The TMC5160
    // retains its configuration while powered, and rewriting chopper/current/
    // ramp registers can create a mechanical current transient.
    tmc5160_health_snapshot snapshot{};
    if (!tmc5160_health_check(&snapshot)) {
        error_ = Tmc5160Error::Communication;
        return false;
    }
    fault_snapshot_ = snapshot.faults;
    if ((snapshot.faults.flags & TMC5160_CRITICAL_FAULT_MASK) != 0U) {
        error_ = Tmc5160Error::CriticalDriverStatus;
        return false;
    }
    if (!tmc5160_runtime_configuration_matches()) {
        error_ = Tmc5160Error::ConfigurationReadback;
        return false;
    }

    tmc5160_arm();
    HAL_Delay(1U);
    bool enabled = false;
    if (!tmc5160_read_driver_enabled(&enabled)) {
        error_ = Tmc5160Error::Communication;
        return false;
    }
    if (!enabled) {
        error_ = Tmc5160Error::EnabledPinReadback;
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
