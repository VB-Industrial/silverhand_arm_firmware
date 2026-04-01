#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor_update(uint32_t now_ms);

void motor_command(float position_rad, float velocity_rad_s, float acceleration_rad_s2);
void motor_move(int32_t velocity_command);
void motor_set_position_steps(int32_t target_position_steps);
void motor_arm(bool armed);

int32_t motor_position_steps(void);
int32_t motor_velocity_steps(void);
uint16_t motor_encoder_raw(void);

float motor_fused_angle_manipulator(void);
float motor_fused_velocity_manipulator(void);

bool motor_ack_fail(void);
int32_t motor_fail_level(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */
