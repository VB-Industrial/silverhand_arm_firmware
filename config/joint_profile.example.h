#pragma once

namespace silver::arm_fw {

struct JointProfile {
    unsigned joint_index;
    float gear_ratio;
    float upper_limit_rad;
    float lower_limit_rad;
    int8_t output_encoder_inverted;
};

constexpr JointProfile kExampleJointProfile{
    .joint_index = 0U,
    .gear_ratio = 1.0F,
    .upper_limit_rad = 3.14F,
    .lower_limit_rad = -3.14F,
    .output_encoder_inverted = 0,
};

}  // namespace silver::arm_fw
