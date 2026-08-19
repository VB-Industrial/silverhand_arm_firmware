#pragma once
#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static const uint8_t SR_JOINT_INDEX = 5U;

// Cyphal uavcan.node.GetInfo software version. Increment these manually before
// flashing a new firmware series: 2.<commit> with the uncommitted trial in
// software_vcs_revision_id.
static const uint8_t SR_FIRMWARE_VERSION_MAJOR = 2U;
static const uint8_t SR_FIRMWARE_VERSION_MINOR = 47U;
static const uint64_t SR_FIRMWARE_VERSION_TRIAL = 9U;

// Fusion offset remains available through pos_get for tuning and logging, but it
// must not stop motion until the slip detector has been validated on hardware.
static const bool SR_ENABLE_FUSION_OFFSET_FAULT = false;

typedef struct robot_joint_profile {
    uint8_t joint_index;
    uint8_t node_id;
    uint8_t controller_node_id;
    uint16_t servo_command_port_id;
    uint16_t direct_command_port_id;
    uint16_t feedback_port_id;
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
    float maximum_servo_velocity_rad_s;
    float maximum_direct_velocity_rad_s;
    bool enable_cyphal;
    bool has_output_encoder;
    int8_t output_encoder_inverted;
    float logical_hard_lower_rad;
    float logical_hard_upper_rad;
    float soft_limit_margin_rad;
} robot_joint_profile;

/*
 * Cyphal subject map:
 *
 *   controller node 100 -> joint nodes 21..26
 *     1121..1126: reg.udral.physics.kinematics.rotation.Planar.0.1
 *                 SERVO position/velocity/acceleration command
 *     1131..1136: uavcan.si.unit.angular_velocity.Scalar.1.0
 *                 DIRECT joint velocity command in rad/s
 *
 *   joint nodes 21..26 -> controller/network
 *     1001:       reg.udral.physics.kinematics.rotation.Planar.0.1
 *                 shared position/velocity feedback subject; distinguish
 *                 joints by the transfer source node-ID
 */
static const robot_joint_profile kRobotJointProfiles[] = {
    {1U, 21U, 100U, 1121U, 1131U, 1001U, 7509U, 17U, 3.9F, 12, 4, 1, 7680000U, 50.0F, 4.0F, 4785U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 1, -2.984743118F, 2.986660719F, 0.076699019F},
    {2U, 22U, 100U, 1122U, 1132U, 1001U, 7509U, 23U, 10.2F, 31, 6, -1, 5120000U, 50.0F, 1.0F, 9871U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 0, -3.425762653F, 0.021859227F, 0.068645716F},
    {3U, 23U, 100U, 1123U, 1133U, 1001U, 7509U, 17U, 3.9F, 12, 4, -1, 5120000U, 50.0F, 1.0F, 6769U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 0, -0.007669904F, 5.105088234F, 0.076699019F},
    {4U, 24U, 100U, 1124U, 1134U, 1001U, 7509U, 14U, 0.3F, 5, 2, 1, 2458010U, 19.203208F, 2.5F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 1, -2.444015026F, 3.231714010F, 0.076699019F},
    {5U, 25U, 100U, 1125U, 1135U, 1001U, 7509U, 14U, 0.3F, 5, 2, -1, 2458010U, 19.203208F, 2.5F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 0, -2.418704271F, 2.488116980F, 0.076699019F},
    {6U, 26U, 100U, 1126U, 1136U, 1001U, 7509U, 14U, 0.3F, 5, 2, 1, 2458010U, 19.203208F, 2.5F, 0U, 0.8F, 0.2F, 0.95F, 0.05F, 0.1F, 0.1F, 0.12F, true, true, 1, -2.503456831F, 3.180325747F, 0.076699019F},
};
static const robot_joint_profile* const kRobotJointProfile = &kRobotJointProfiles[SR_JOINT_INDEX - 1U];

#ifdef __cplusplus
}
#endif

#endif /* INC_ROBOT_CONFIG_H_ */
