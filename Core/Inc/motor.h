#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "fault_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_UPDATE_PERIOD_MS 10U

typedef struct motor_encoder_diagnostics {
    uint16_t raw_frame;
    uint32_t transfer_count;
    uint32_t error_count;
    int32_t last_hal_status;
    int32_t raw_unwrapped_min;
    int32_t raw_unwrapped_max;
    int32_t raw_unwrapped_span;
    int32_t maximum_frame_delta;
    bool last_read_ok;
    bool has_valid_angle;
} motor_encoder_diagnostics;

typedef enum motor_hybrid_state {
    MOTOR_HYBRID_STATE_UNKNOWN = 0,
    MOTOR_HYBRID_STATE_TAKEUP_POSITIVE = 1,
    MOTOR_HYBRID_STATE_LOCKED_POSITIVE = 2,
    MOTOR_HYBRID_STATE_TAKEUP_NEGATIVE = 3,
    MOTOR_HYBRID_STATE_LOCKED_NEGATIVE = 4,
    MOTOR_HYBRID_STATE_MOTION_MISMATCH = 5,
    MOTOR_HYBRID_STATE_MANUAL = 6,
} motor_hybrid_state;

typedef struct motor_fusion_diagnostics {
    float encoder_angle_rad;
    float tmc_angle_rad;
    float offset_rad;
    float fused_angle_rad;
    float encoder_error_rad;
    float backlash_rad;
    float innovation_rad;
    float applied_correction_rad;
    float slip_window_residual_rad;
    uint32_t rejected_spike_count;
    uint32_t persistent_residual_count;
    float takeup_tmc_travel_rad;
    float takeup_encoder_travel_rad;
    float encoder_weight;
    motor_hybrid_state hybrid_state;
    bool slip_candidate;
    bool slip_latched;
    bool calibrated_encoder;
} motor_fusion_diagnostics;

typedef enum motor_servo_state {
    MOTOR_SERVO_STATE_INACTIVE = 0,
    MOTOR_SERVO_STATE_TRACKING = 1,
    MOTOR_SERVO_STATE_SETTLING = 2,
    MOTOR_SERVO_STATE_AT_TARGET = 3,
} motor_servo_state;

typedef struct motor_servo_diagnostics {
    float target_position_rad;
    float position_error_rad;
    float command_velocity_rad_s;
    uint32_t command_age_ms;
    motor_servo_state state;
} motor_servo_diagnostics;

typedef struct motor_limit_diagnostics {
    float hard_lower_rad;
    float soft_lower_rad;
    float soft_upper_rad;
    float hard_upper_rad;
    float current_position_rad;
    float minimum_velocity_rad_s;
    float maximum_velocity_rad_s;
    bool active;
} motor_limit_diagnostics;

typedef struct motor_driver_diagnostics {
    uint32_t health_read_failure_count;
    uint32_t enable_readback_mismatch_count;
    uint32_t critical_status_count;
} motor_driver_diagnostics;

typedef struct motor_extended_diagnostics {
    float physical_lower_rad;
    float physical_upper_rad;
    int32_t corrected_encoder_ticks;
    int32_t target_steps;
    uint32_t driver_initialize_count;
    uint32_t driver_enable_count;
    uint32_t driver_disable_count;
    bool localized;
    bool position_reached;
} motor_extended_diagnostics;

typedef enum motor_startup_recovery_state {
    MOTOR_STARTUP_RECOVERY_CHECKING = 0,
    MOTOR_STARTUP_RECOVERY_IN_RANGE = 1,
    MOTOR_STARTUP_RECOVERY_TO_LOWER_SOFT = 2,
    MOTOR_STARTUP_RECOVERY_TO_UPPER_SOFT = 3,
    MOTOR_STARTUP_RECOVERY_COMPLETE = 4,
    MOTOR_STARTUP_RECOVERY_UNLOCALIZED = 5,
    MOTOR_STARTUP_RECOVERY_FAILED = 6,
} motor_startup_recovery_state;

typedef enum motor_control_mode {
    MOTOR_CONTROL_MODE_HOLD = 0,
    MOTOR_CONTROL_MODE_SERVO = 1,
    MOTOR_CONTROL_MODE_DIRECT = 2,
    MOTOR_CONTROL_MODE_CALIBRATION = 3,
    MOTOR_CONTROL_MODE_TMC_POSITION = 4,
} motor_control_mode;

void motor_init(void);
void motor_update(uint32_t now_ms);

bool motor_command(float position_rad, float velocity_rad_s, float acceleration_rad_s2);
bool motor_move(int32_t velocity_command);
bool motor_set_tmc_position_steps(int32_t target_position_steps);
bool motor_move_calibration(int32_t velocity_command);
bool motor_move_radians_per_second(float velocity_rad_s);
bool motor_set_position_radians(float target_position_rad);
bool motor_arm(bool armed);
bool motor_driver_enabled(void);
int32_t motor_driver_state(void);
int32_t motor_driver_error(void);
int32_t motor_control_mode_get(void);
void motor_note_heartbeat(uint32_t now_ms, uint8_t source_node_id);

int32_t motor_position_steps(void);
int32_t motor_velocity_steps(void);
uint16_t motor_encoder_raw(void);
bool motor_encoder_get_diagnostics(motor_encoder_diagnostics* diagnostics);
bool motor_fusion_get_diagnostics(motor_fusion_diagnostics* diagnostics);
bool motor_servo_get_diagnostics(motor_servo_diagnostics* diagnostics);
bool motor_limit_get_diagnostics(motor_limit_diagnostics* diagnostics);
bool motor_driver_get_diagnostics(motor_driver_diagnostics* diagnostics);
bool motor_extended_get_diagnostics(motor_extended_diagnostics* diagnostics);
int32_t motor_startup_recovery_state_get(void);
float motor_startup_recovery_target_get(void);

bool motor_auto_calibration_start(void);
bool motor_manual_calibration_command(int32_t command);
bool motor_backlash_calibration_start(void);
bool motor_zero_calibrate(void);
int32_t motor_calibration_state(void);
int32_t motor_calibration_error(void);
int32_t motor_calibration_failure_state(void);
int32_t motor_calibration_progress(void);
void motor_calibration_data(int32_t* values, uint8_t capacity, uint8_t* count);

float motor_fused_angle_manipulator(void);
float motor_fused_velocity_manipulator(void);

bool motor_ack_fail(void);
int32_t motor_fail_level(void);
bool motor_slip_latched(void);
uint32_t motor_fault_active(void);
uint32_t motor_fault_latched(void);
int32_t motor_network_state(void);
int32_t motor_controller_state(void);
int32_t motor_stop_reason(void);
bool motor_fault_log_last(fault_log_record* record);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */
