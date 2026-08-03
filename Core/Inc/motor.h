#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "fault_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motor_encoder_diagnostics {
    uint16_t raw_frame;
    uint32_t transfer_count;
    uint32_t error_count;
    int32_t last_hal_status;
    bool last_read_ok;
    bool has_valid_angle;
} motor_encoder_diagnostics;

typedef struct motor_fusion_diagnostics {
    float encoder_angle_rad;
    float tmc_angle_rad;
    float offset_rad;
    float fused_angle_rad;
    float encoder_error_rad;
    float backlash_rad;
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

typedef enum motor_control_mode {
    MOTOR_CONTROL_MODE_HOLD = 0,
    MOTOR_CONTROL_MODE_SERVO = 1,
    MOTOR_CONTROL_MODE_DIRECT = 2,
    MOTOR_CONTROL_MODE_CALIBRATION = 3,
} motor_control_mode;

void motor_init(void);
void motor_update(uint32_t now_ms);

bool motor_command(float position_rad, float velocity_rad_s, float acceleration_rad_s2);
bool motor_move(int32_t velocity_command);
bool motor_move_radians_per_second(float velocity_rad_s);
void motor_set_position_steps(int32_t target_position_steps);
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

bool motor_calibration_command(int32_t command);
bool motor_auto_calibration_start(void);
bool motor_backlash_calibration_start(void);
bool motor_zero_calibrate(void);
bool motor_calibration_next(void);
int32_t motor_calibration_state(void);
int32_t motor_calibration_error(void);
int32_t motor_calibration_progress(void);
void motor_calibration_result(int32_t* values, uint8_t capacity, uint8_t* count);

float motor_fused_angle_manipulator(void);
float motor_fused_velocity_manipulator(void);

bool motor_ack_fail(void);
int32_t motor_fail_level(void);
uint32_t motor_fault_active(void);
uint32_t motor_fault_latched(void);
int32_t motor_network_state(void);
int32_t motor_controller_state(void);
int32_t motor_stop_reason(void);
uint32_t motor_fault_log_count(void);
bool motor_fault_log_last(fault_log_record* record);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */
