#include <cstdint>

namespace silver::arm_fw {

struct RuntimeConfig {
    bool cyphal_enabled{true};
    bool motor_control_enabled{true};
};

static RuntimeConfig make_runtime_config() {
    RuntimeConfig cfg{};
#if !defined(SR_ARM_ENABLE_CYPHAL)
    cfg.cyphal_enabled = false;
#endif
#if !defined(SR_ARM_ENABLE_MOTOR_CONTROL)
    cfg.motor_control_enabled = false;
#endif
    return cfg;
}

int app_main() {
    const RuntimeConfig runtime = make_runtime_config();
    (void) runtime;

    for (;;) {
    }
}

}  // namespace silver::arm_fw

int main() {
    return silver::arm_fw::app_main();
}
