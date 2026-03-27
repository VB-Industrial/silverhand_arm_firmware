#pragma once

namespace silver::arm_fw {

struct JointProfile {
    unsigned joint_index;
    float gear_ratio;
    float upper_limit_rad;
    float lower_limit_rad;
    bool motor_encoder_inverted;
    bool output_encoder_inverted;
};

constexpr JointProfile kExampleJointProfile{
    .joint_index = 0U,
    .gear_ratio = 1.0F,
    .upper_limit_rad = 3.14F,
    .lower_limit_rad = -3.14F,
    .motor_encoder_inverted = false,
    .output_encoder_inverted = false,
};

}  // namespace silver::arm_fw
