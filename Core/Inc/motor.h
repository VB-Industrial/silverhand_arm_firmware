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

void motor_init(void);
void motor_update(uint32_t now_ms);

void motor_command(float position_rad, float velocity_rad_s, float acceleration_rad_s2);
void motor_move(int32_t velocity_command);
void motor_set_position_steps(int32_t target_position_steps);
bool motor_arm(bool armed);
bool motor_driver_enabled(void);
int32_t motor_driver_state(void);
int32_t motor_driver_error(void);
void motor_note_heartbeat(uint32_t now_ms, uint8_t source_node_id);

int32_t motor_position_steps(void);
int32_t motor_velocity_steps(void);
uint16_t motor_encoder_raw(void);
bool motor_encoder_get_diagnostics(motor_encoder_diagnostics* diagnostics);

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
