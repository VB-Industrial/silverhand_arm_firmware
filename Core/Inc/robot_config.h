#pragma once
#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static const uint8_t SR_JOINT_INDEX = 1U;

typedef struct robot_joint_profile {
    uint8_t joint_index;
    uint8_t node_id;
    uint16_t js_pub_port_id;
    uint16_t js_sub_port_id;
    uint16_t agent_js_sub_port_id;
    uint16_t heartbeat_subject_id;
    uint8_t motor_type;
    float max_effort;
    int8_t max_irun_scaler;
    int8_t init_irun;
    int8_t direction;
    uint32_t joint_full_steps;
    float motor_gear_ratio;
    float joint_gear_ratio;
    uint32_t default_zero_enc;
    float angle_encoder_weight;
    float angle_tmc_weight;
    float velocity_tmc_weight;
    float velocity_encoder_weight;
    float velocity_encoder_lpf_alpha;
    bool enable_cyphal;
    bool has_output_encoder;
    int8_t output_encoder_inverted;
} robot_joint_profile;

static const robot_joint_profile kRobotJointProfiles[] = {
    {1U, 1U, 1111U, 1121U, 1001U, 7509U, 17U, 3.9F, 12, 5, 1, 7680000U, 50.0F, 3.0F, 4785U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, true, 0},
    {2U, 2U, 1112U, 1122U, 1001U, 7509U, 23U, 10.2F, 31, 10, 1, 5120000U, 50.0F, 2.0F, 9871U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, true, 0},
    {3U, 3U, 1113U, 1123U, 1001U, 7509U, 17U, 3.9F, 12, 5, 1, 5120000U, 50.0F, 2.0F, 6769U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, true, 0},
    {4U, 4U, 1114U, 1124U, 1001U, 7509U, 14U, 0.5F, 8, 3, 1, 2458010U, 19.203208F, 2.5F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, false, 0},
    {5U, 5U, 1115U, 1125U, 1001U, 7509U, 14U, 0.5F, 8, 3, 1, 2458010U, 19.203208F, 2.5F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, false, 0},
    {6U, 6U, 1116U, 1126U, 1001U, 7509U, 14U, 0.5F, 8, 3, 1, 983204U, 19.203208F, 1.0F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, true, false, 0},
};
static const robot_joint_profile* const kRobotJointProfile = &kRobotJointProfiles[SR_JOINT_INDEX - 1U];

#ifdef __cplusplus
}
#endif

#endif /* INC_ROBOT_CONFIG_H_ */
