#pragma once

#include <array>
#include <cstdint>

extern "C" {
#include "encoder_calibration_storage.h"
}

enum class EncoderCalibrationState : int32_t {
    Idle = 0,
    WaitLimitA = 1,
    WaitLimitB = 2,
    Ready = 3,
    MoveToA = 4,
    SettleAtA = 5,
    SweepToB = 6,
    Processing = 7,
    Saving = 8,
    Complete = 9,
    Failed = 10,
    Aborted = 11,
    AutoSeekLimitA = 12,
    AutoBackoffA = 13,
    AutoSeekLimitB = 14,
    SettleAtB = 15,
    ReverseSweepToA = 16,
    MoveToRockStart = 17,
    RockSettle = 18,
    RockSweep = 19,
    MoveToMiddle = 20,
    SettleAtMiddle = 21,
};

enum class EncoderCalibrationError : int32_t {
    None = 0,
    EncoderUnavailable = 1,
    InvalidManualSpan = 2,
    MotionTimeout = 3,
    WrongDirection = 4,
    InvalidTable = 5,
    Storage = 6,
    Driver = 7,
};

class EncoderCalibration final {
public:
    struct Action {
        bool command_velocity = false;
        int32_t velocity_steps = 0;
        bool zero_tmc_position = false;
        bool disable_driver = false;
    };

    void initialize(uint8_t joint_id, bool encoder_inverted, uint16_t raw);
    bool begin_auto(uint32_t now_ms, bool encoder_available, uint16_t raw);
    bool begin_backlash(uint32_t now_ms, bool encoder_available, uint16_t raw);
    bool calibrate_zero(uint16_t raw);
    void fail(EncoderCalibrationError error);
    Action update(uint32_t now_ms, bool encoder_available, uint16_t raw, int32_t tmc_position_steps);

    EncoderCalibrationState state() const;
    EncoderCalibrationError error() const;
    int32_t progress_percent() const;
    bool blocks_normal_control() const;
    const encoder_calibration_data& data() const;
    bool has_stored_data() const;
    bool calibrated_position_ticks(uint16_t raw, int32_t& ticks_from_zero) const;
    bool calibrated_limit_ticks(
        int32_t& hard_a_ticks,
        int32_t& soft_a_ticks,
        int32_t& soft_b_ticks,
        int32_t& hard_b_ticks) const;
    bool zero_valid() const;
    uint16_t zero_raw() const;

private:
    int32_t observe_raw(uint16_t raw);
    int32_t velocity_toward(int32_t target) const;
    bool reached(int32_t target, int32_t previous, int32_t current) const;
    int32_t point_target(uint16_t index) const;
    int32_t corrected_position_ticks(int32_t position_ticks) const;
    bool process_table();
    bool prepare_backlash_rock(int32_t center_ticks);
    bool locate_stored_position(uint16_t raw, int32_t& position_ticks);
    bool locate_position_in_stored_span(uint16_t raw, bool safe_only, int32_t& position_ticks) const;
    bool record_backlash_sample(int32_t position_ticks, int32_t tmc_position_steps);
    bool finalize_backlash();
    bool auto_stall_detected(uint32_t now_ms);
    bool prepare_automatic_span(uint16_t raw);

    static constexpr int32_t kCalibrationVelocitySteps = 20000;
    static constexpr uint32_t kSettleTimeMs = 500U;
    static constexpr uint32_t kMotionTimeoutMs = 360000U;
    static constexpr uint32_t kAutoStallTimeMs = 300U;
    static constexpr uint32_t kAutoStartupGraceMs = 500U;
    static constexpr int32_t kAutoStallMotionTicks = 6;
    static constexpr uint8_t kBacklashRockCycles = 4U;
    static constexpr uint8_t kBacklashDiscardedSamples = 2U;
    static constexpr uint8_t kBacklashSampleCount = 2U * kBacklashRockCycles;
    static constexpr int32_t kMinimumBacklashRockHalfRangeTicks = 8;
    // One twenty-fourth of an encoder revolution is 15 degrees.
    static constexpr int32_t kBacklashRockHalfRangeTicks = 16384 / 24;

    EncoderCalibrationState state_ = EncoderCalibrationState::Idle;
    EncoderCalibrationError error_ = EncoderCalibrationError::None;
    uint8_t joint_id_ = 0U;
    bool encoder_inverted_ = false;
    bool has_stored_data_ = false;
    uint16_t previous_raw_ = 0U;
    int32_t position_ticks_ = 0;
    int32_t previous_position_ticks_ = 0;
    int32_t safe_start_ticks_ = 0;
    int32_t safe_end_ticks_ = 0;
    uint32_t state_started_ms_ = 0U;
    uint16_t next_point_ = 0U;
    uint32_t stall_anchor_ms_ = 0U;
    int32_t stall_anchor_ticks_ = 0;
    int32_t rock_middle_ticks_ = 0;
    int32_t rock_low_ticks_ = 0;
    int32_t rock_high_ticks_ = 0;
    int32_t rock_target_ticks_ = 0;
    int32_t rock_start_position_ticks_ = 0;
    int32_t rock_start_tmc_steps_ = 0;
    uint8_t backlash_sample_count_ = 0U;
    bool rock_at_high_ = false;
    std::array<int32_t, ENCODER_CALIBRATION_MAX_POINTS> tmc_samples_{};
    std::array<int32_t, ENCODER_CALIBRATION_MAX_POINTS> reverse_tmc_samples_{};
    std::array<int32_t, kBacklashSampleCount> backlash_samples_{};
    encoder_calibration_data data_{};
};
