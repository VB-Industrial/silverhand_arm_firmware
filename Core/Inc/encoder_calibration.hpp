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
    bool begin(bool encoder_available, uint16_t raw);
    bool begin_auto(uint32_t now_ms, bool encoder_available, uint16_t raw);
    bool calibrate_zero(uint16_t raw);
    bool advance(uint32_t now_ms, bool encoder_available, uint16_t raw);
    void abort();
    void fail(EncoderCalibrationError error);
    Action update(uint32_t now_ms, bool encoder_available, uint16_t raw, int32_t tmc_position_steps);

    EncoderCalibrationState state() const;
    EncoderCalibrationError error() const;
    int32_t progress_percent() const;
    bool blocks_normal_control() const;
    const encoder_calibration_data& data() const;
    bool has_stored_data() const;
    int32_t manual_total_travel() const;
    bool zero_valid() const;
    uint16_t zero_raw() const;

private:
    int32_t observe_raw(uint16_t raw);
    int32_t velocity_toward(int32_t target) const;
    bool reached(int32_t target, int32_t previous, int32_t current) const;
    int32_t point_target(uint16_t index) const;
    bool process_table();
    bool auto_stall_detected(uint32_t now_ms);
    bool prepare_automatic_span(uint16_t raw);

    static constexpr int32_t kCalibrationVelocitySteps = 20000;
    static constexpr uint32_t kSettleTimeMs = 500U;
    static constexpr uint32_t kMotionTimeoutMs = 360000U;
    static constexpr uint32_t kAutoStallTimeMs = 300U;
    static constexpr uint32_t kAutoStartupGraceMs = 500U;
    static constexpr int32_t kAutoStallMotionTicks = 6;

    EncoderCalibrationState state_ = EncoderCalibrationState::Idle;
    EncoderCalibrationError error_ = EncoderCalibrationError::None;
    uint8_t joint_id_ = 0U;
    bool encoder_inverted_ = false;
    bool has_stored_data_ = false;
    uint16_t previous_raw_ = 0U;
    int32_t position_ticks_ = 0;
    int32_t previous_position_ticks_ = 0;
    int32_t manual_total_travel_ = 0;
    int32_t safe_start_ticks_ = 0;
    int32_t safe_end_ticks_ = 0;
    uint32_t state_started_ms_ = 0U;
    uint16_t next_point_ = 0U;
    uint32_t stall_anchor_ms_ = 0U;
    int32_t stall_anchor_ticks_ = 0;
    std::array<int32_t, ENCODER_CALIBRATION_MAX_POINTS> tmc_samples_{};
    encoder_calibration_data data_{};
};
