#include "main.h"
#include "robot_config.h"

#include <algorithm>
#include <memory>

#include "cyphal/cyphal.h"
#include "cyphal/node/node_info_handler.h"
#include "cyphal/node/registers_handler.hpp"
#include "cyphal/providers/G4CAN.h"
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/subscriptions/subscription.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/scalar/Integer32_1_0.h"
#include "uavcan/si/unit/angular_velocity/Scalar_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>

#include <uavcan/node/GetInfo_1_0.h>

extern "C" {
#include "communications.h"
#include "motor.h"
#include "system_watchdog.h"
#include "utility.h"

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(DirectVelocityMsg, uavcan_si_unit_angular_velocity_Scalar_1_0)

std::byte buffer[sizeof(CyphalInterface) + sizeof(G4CAN) + sizeof(SystemAllocator)];
std::shared_ptr<CyphalInterface> interface;


void error_handler() { Error_Handler(); }
uint64_t micros_64() { return HAL_GetTick() * 1000; }
UtilityConfig utilities(micros_64, error_handler);

class HBeatReader: public AbstractSubscription<HBeat> {
public:
    HBeatReader(InterfacePtr interface): AbstractSubscription<HBeat>(interface,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const uavcan_node_Heartbeat_1_0& hbeat, CanardRxTransfer* transfer) override {
        UNUSED(hbeat);
        if (transfer != nullptr) {
            motor_note_heartbeat(HAL_GetTick(), transfer->metadata.remote_node_id);
        }
    }
};


HBeatReader* h_reader;

class JSReader: public AbstractSubscription<JS_msg> {
public:
	JSReader(InterfacePtr interface): AbstractSubscription<JS_msg>(interface,
		kRobotJointProfile->servo_command_port_id
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_in, CanardRxTransfer* transfer) override
    {
        if ((transfer == nullptr) ||
            (transfer->metadata.remote_node_id != kRobotJointProfile->controller_node_id)) {
            return;
        }
        motor_command(
            js_in.angular_position.radian,
            js_in.angular_velocity.radian_per_second,
            js_in.angular_acceleration.radian_per_second_per_second);
    }
};

JSReader* js_reader;
class DirectVelocityReader: public AbstractSubscription<DirectVelocityMsg> {
public:
    DirectVelocityReader(InterfacePtr interface): AbstractSubscription<DirectVelocityMsg>(
        interface,
        kRobotJointProfile->direct_command_port_id
    ) {};

    void handler(
        const uavcan_si_unit_angular_velocity_Scalar_1_0& command,
        CanardRxTransfer* transfer) override
    {
        if ((transfer == nullptr) ||
            (transfer->metadata.remote_node_id != kRobotJointProfile->controller_node_id)) {
            return;
        }
        motor_move_radians_per_second(command.radian_per_second);
    }
};

DirectVelocityReader* direct_velocity_reader;
NodeInfoReader* nireader;
static constexpr size_t NUMBER_OF_REGISTERS = 12;

RegistersHandler<NUMBER_OF_REGISTERS>* registers_handler;

static bool try_get_register_int32(const uavcan_register_Value_1_0& value, int32_t& out)
{
    switch (value._tag_) {
    case 5:
        if (value.integer32.value.count > 0) {
            out = value.integer32.value.elements[0];
            return true;
        }
        break;
    case 6:
        if (value.integer16.value.count > 0) {
            out = value.integer16.value.elements[0];
            return true;
        }
        break;
    case 7:
        if (value.integer8.value.count > 0) {
            out = value.integer8.value.elements[0];
            return true; 
        }
        break;
    case 9:
        if (value.natural32.value.count > 0) {
            out = static_cast<int32_t>(value.natural32.value.elements[0]);
            return true;
        }
        break;
    case 10:
        if (value.natural16.value.count > 0) {
            out = value.natural16.value.elements[0];
            return true;
        }
        break;
    case 11:
        if (value.natural8.value.count > 0) {
            out = value.natural8.value.elements[0];
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

static bool try_get_register_real32(const uavcan_register_Value_1_0& value, float& out)
{
    switch (value._tag_) {
    case 12:
        if (value.real64.value.count > 0) {
            out = static_cast<float>(value.real64.value.elements[0]);
            return true;
        }
        break;
    case 13:
        if (value.real32.value.count > 0) {
            out = value.real32.value.elements[0];
            return true;
        }
        break;
    case 14:
        if (value.real16.value.count > 0) {
            out = value.real16.value.elements[0];
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}

static void set_register_int32(uavcan_register_Value_1_0& value, const int32_t data)
{
    value._tag_ = 5;
    value.integer32.value.elements[0] = data;
    value.integer32.value.count = 1;
}

static void set_register_int32_array(
    uavcan_register_Value_1_0& value,
    const int32_t* data,
    const size_t count)
{
    value._tag_ = 5;
    const size_t output_count = std::min(count, sizeof(value.integer32.value.elements) / sizeof(int32_t));
    for (size_t index = 0U; index < output_count; ++index) {
        value.integer32.value.elements[index] = data[index];
    }
    value.integer32.value.count = output_count;
}

static void set_register_real32(uavcan_register_Value_1_0& value, const float data)
{
    value._tag_ = 13;
    value.real32.value.elements[0] = data;
    value.real32.value.count = 1;
}

static void set_register_real32_array(
    uavcan_register_Value_1_0& value,
    const float* data,
    const size_t count)
{
    value._tag_ = 13;
    const size_t output_count = std::min(count, sizeof(value.real32.value.elements) / sizeof(float));
    for (size_t index = 0U; index < output_count; ++index) {
        value.real32.value.elements[index] = data[index];
    }
    value.real32.value.count = output_count;
}

void move_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t velocity_command = motor_velocity_steps();
    if (try_get_register_int32(v_in, velocity_command)) {
        motor_move(velocity_command);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
    set_register_int32(v_out, motor_velocity_steps());
    response.persistent = true;
    response._mutable = true;
}

void tmc_position_set_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t target_position_steps = motor_position_steps();
    if (try_get_register_int32(v_in, target_position_steps)) {
        motor_set_tmc_position_steps(target_position_steps);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
    set_register_int32(v_out, motor_position_steps());
    response.persistent = false;
    response._mutable = true;
}

void pos_set_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    float target_position_rad = motor_fused_angle_manipulator();
    if (try_get_register_real32(v_in, target_position_rad)) {
        motor_set_position_radians(target_position_rad);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }

    set_register_real32(v_out, motor_fused_angle_manipulator());
    response.persistent = false;
    response._mutable = true;
}

void pos_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    motor_fusion_diagnostics fusion{};
    motor_encoder_diagnostics encoder{};
    motor_limit_diagnostics limits{};
    motor_fusion_get_diagnostics(&fusion);
    motor_encoder_get_diagnostics(&encoder);
    motor_limit_get_diagnostics(&limits);
    const float values[] = {
        static_cast<float>(motor_encoder_raw()),
        static_cast<float>(motor_position_steps()),
        fusion.encoder_angle_rad,
        fusion.tmc_angle_rad,
        fusion.offset_rad,
        fusion.fused_angle_rad,
        motor_fused_velocity_manipulator(),
        fusion.encoder_error_rad,
        fusion.backlash_rad,
        static_cast<float>(motor_control_mode_get()),
        fusion.calibrated_encoder ? 1.0F : 0.0F,
        limits.hard_lower_rad,
        limits.soft_lower_rad,
        limits.soft_upper_rad,
        limits.hard_upper_rad,
        limits.minimum_velocity_rad_s,
        limits.maximum_velocity_rad_s,
        static_cast<float>(motor_calibration_state()),
        static_cast<float>(motor_calibration_progress()),
        fusion.innovation_rad,
        fusion.applied_correction_rad,
        fusion.slip_window_residual_rad,
        static_cast<float>(fusion.rejected_spike_count),
        static_cast<float>(fusion.persistent_residual_count),
        fusion.slip_candidate ? 1.0F : 0.0F,
        static_cast<float>(encoder.raw_unwrapped_min),
        static_cast<float>(encoder.raw_unwrapped_max),
        static_cast<float>(encoder.raw_unwrapped_span),
        static_cast<float>(encoder.maximum_frame_delta),
        static_cast<float>(fusion.hybrid_state),
        fusion.takeup_tmc_travel_rad,
        fusion.takeup_encoder_travel_rad,
        fusion.encoder_weight,
        fusion.slip_latched ? 1.0F : 0.0F,
        static_cast<float>(motor_startup_recovery_state_get()),
        motor_startup_recovery_target_get(),
    };
    set_register_real32_array(v_out, values, std::size(values));
    response.persistent = false;
    response._mutable = false;
}

void errors_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    motor_encoder_diagnostics encoder{};
    motor_driver_diagnostics driver{};
    fault_log_record log{};
    motor_encoder_get_diagnostics(&encoder);
    motor_driver_get_diagnostics(&driver);
    motor_fault_log_last(&log);
    const int32_t values[] = {
        static_cast<int32_t>(motor_fault_active()),
        static_cast<int32_t>(motor_fault_latched()),
        motor_fail_level(),
        motor_stop_reason(),
        motor_network_state(),
        motor_controller_state(),
        motor_driver_state(),
        motor_driver_error(),
        static_cast<int32_t>(system_watchdog_reset_reason()),
        encoder.last_read_ok ? 1 : 0,
        encoder.last_hal_status,
        static_cast<int32_t>(encoder.transfer_count),
        static_cast<int32_t>(encoder.error_count),
        static_cast<int32_t>(encoder.raw_frame),
        encoder.has_valid_angle ? 1 : 0,
        motor_calibration_state(),
        motor_calibration_error(),
        static_cast<int32_t>(log.sequence),
        static_cast<int32_t>(log.uptime_ms),
        static_cast<int32_t>(log.fault_mask),
        static_cast<int32_t>(log.tmc_gstat),
        static_cast<int32_t>(log.tmc_drv_status),
        static_cast<int32_t>(driver.health_read_failure_count),
        static_cast<int32_t>(driver.enable_readback_mismatch_count),
        static_cast<int32_t>(driver.critical_status_count),
    };
    set_register_int32_array(v_out, values, std::size(values));
    response.persistent = false;
    response._mutable = false;
}

void arm_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t arm_command = motor_driver_enabled() ? 1 : 0;
    if (try_get_register_int32(v_in, arm_command)) {
        motor_arm(arm_command != 0);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
    motor_fusion_diagnostics fusion{};
    motor_fusion_get_diagnostics(&fusion);
    const int32_t values[] = {
        motor_driver_enabled() ? 1 : 0,
        static_cast<int32_t>(fusion.hybrid_state),
        static_cast<int32_t>(fusion.encoder_weight * 1000.0F),
        fusion.slip_candidate ? 1 : 0,
        fusion.slip_latched ? 1 : 0,
    };
    set_register_int32_array(v_out, values, std::size(values));
    response.persistent = true;
    response._mutable = true;
}

void fail_ack_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t ack_command = 0;
    if (try_get_register_int32(v_in, ack_command) && (ack_command == 1)) {
        motor_ack_fail();
    }
    set_register_int32(v_out, motor_fail_level());
    response.persistent = true;
    response._mutable = true;
}

void auto_calibration_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    motor_auto_calibration_start();
    set_register_int32(v_out, motor_calibration_state());
    response.persistent = false;
    response._mutable = false;
}

void manual_calibration_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t command = 0;
    int32_t result = 0;
    if (try_get_register_int32(v_in, command)) {
        if ((command >= 1) && (command <= 5) &&
            motor_manual_calibration_command(command)) {
            result = command + 1;
        }
    }
    set_register_int32(v_out, result);
    response.persistent = false;
    response._mutable = true;
}

void calibration_move_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t velocity_command = 0;
    int32_t result = 0;
    if (try_get_register_int32(v_in, velocity_command) &&
        motor_move_calibration(velocity_command)) {
        result = velocity_command;
    }
    set_register_int32(v_out, result);
    response.persistent = false;
    response._mutable = true;
}

void calibration_data_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t values[8]{};
    uint8_t count = 0U;
    motor_calibration_data(values, static_cast<uint8_t>(std::size(values)), &count);
    set_register_int32_array(v_out, values, count);
    response.persistent = false;
    response._mutable = false;
}

void zero_calibration_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    set_register_int32(v_out, motor_zero_calibrate() ? 1 : 0);
    response.persistent = false;
    response._mutable = false;
}

void backlash_calibration_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    motor_backlash_calibration_start();
    set_register_int32(v_out, motor_calibration_state());
    response.persistent = false;
    response._mutable = false;
}

void send_JS(void) {             //float* pos, float* vel, float* eff
	static CanardTransferID int_transfer_id = 0;
	reg_udral_physics_kinematics_rotation_Planar_0_1 js_msg =
	{
			.angular_position = motor_fused_angle_manipulator(),
			.angular_velocity = motor_fused_velocity_manipulator(),
			.angular_acceleration = 0.0F
	};
    interface->send_msg<JS_msg>(
		&js_msg,
		kRobotJointProfile->feedback_port_id,
		&int_transfer_id
	);
}

void heartbeat() {
	static CanardTransferID hbeat_transfer_id = 0;
	static uint32_t uptime = 0;
    const int32_t fault_level = motor_fail_level();
    const uint8_t health = (fault_level <= 0)
        ? uavcan_node_Health_1_0_NOMINAL
        : ((fault_level == 1)
            ? uavcan_node_Health_1_0_ADVISORY
            : ((fault_level == 2)
                ? uavcan_node_Health_1_0_CAUTION
                : uavcan_node_Health_1_0_WARNING));
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {
        .uptime = uptime,
        .health = {health},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = static_cast<uint8_t>(motor_fault_active() & 0xFFU)
    };
    interface->send_msg<HBeat>(
		&heartbeat_msg,
		uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
		&hbeat_transfer_id
	);
    uptime += 1;
}

void setup_cyphal(FDCAN_HandleTypeDef* handler) {
	interface = std::shared_ptr<CyphalInterface>(
		         // memory location, node_id, fdcan handler, messages memory pool, utils ref
		CyphalInterface::create_bss<G4CAN, SystemAllocator>(buffer, kRobotJointProfile->node_id, handler, 400, utilities)
	);
    h_reader = new HBeatReader(interface);
	js_reader = new JSReader(interface);
	direct_velocity_reader = new DirectVelocityReader(interface);
	registers_handler = new RegistersHandler<NUMBER_OF_REGISTERS>(
        {
            RegisterDefinition{"man_cal", manual_calibration_handler},
            RegisterDefinition{"move_cal", calibration_move_handler},
            RegisterDefinition{"cal_data", calibration_data_handler},
            RegisterDefinition{"pos_set", pos_set_handler},
            RegisterDefinition{"pos_get", pos_get_handler},
            RegisterDefinition{"fail_ack", fail_ack_handler},
            RegisterDefinition{"errors", errors_handler},
            RegisterDefinition{"move", move_handler},
            RegisterDefinition{"tmc_pos_set", tmc_position_set_handler},
            RegisterDefinition{"zero_set", zero_calibration_handler},
            RegisterDefinition{"luft_cal", backlash_calibration_handler},
            RegisterDefinition{"arm", arm_handler},
        },
        interface
    );
	nireader = new NodeInfoReader(
        interface,
        "joint_" + std::to_string(kRobotJointProfile->joint_index),
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{0, 1},
        0
    );
}

void cyphal_loop() {
    interface->loop();
}

void cyphal_can_starter(FDCAN_HandleTypeDef* hfdcan)
{

	CanardFilter cyphal_filter_for_node_id = canardMakeFilterForServices(kRobotJointProfile->node_id);
	CanardFilter cyphal_filter_for_JS = canardMakeFilterForSubject(kRobotJointProfile->servo_command_port_id);
	CanardFilter cyphal_filter_for_direct = canardMakeFilterForSubject(kRobotJointProfile->direct_command_port_id);
	CanardFilter cyphal_filter_for_HB = canardMakeFilterForSubject(kRobotJointProfile->heartbeat_subject_id);//robot_joint_state_sub_port_id()
	static FDCAN_FilterTypeDef sFilterConfig;
	static FDCAN_FilterTypeDef directFilterConfig;
	static FDCAN_FilterTypeDef hbFilterConfig;
	static FDCAN_FilterTypeDef niFilterConfig;

	niFilterConfig.IdType = FDCAN_EXTENDED_ID;
	niFilterConfig.FilterIndex = 0;
	niFilterConfig.FilterType = FDCAN_FILTER_MASK;
	niFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	niFilterConfig.FilterID1 =  cyphal_filter_for_node_id.extended_can_id;
	niFilterConfig.FilterID2 =  cyphal_filter_for_node_id.extended_mask;

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 =  cyphal_filter_for_JS.extended_can_id;
	sFilterConfig.FilterID2 =  cyphal_filter_for_JS.extended_mask;

	directFilterConfig.IdType = FDCAN_EXTENDED_ID;
	directFilterConfig.FilterIndex = 2;
	directFilterConfig.FilterType = FDCAN_FILTER_MASK;
	directFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	directFilterConfig.FilterID1 = cyphal_filter_for_direct.extended_can_id;
	directFilterConfig.FilterID2 = cyphal_filter_for_direct.extended_mask;

	hbFilterConfig.IdType = FDCAN_EXTENDED_ID;
	hbFilterConfig.FilterIndex = 3;
	hbFilterConfig.FilterType = FDCAN_FILTER_MASK;
	hbFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	hbFilterConfig.FilterID1 =  cyphal_filter_for_HB.extended_can_id;
	hbFilterConfig.FilterID2 =  cyphal_filter_for_HB.extended_mask;



	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT) != HAL_OK)
	{
	  Error_Handler();
	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &niFilterConfig) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &directFilterConfig) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_FDCAN_ConfigFilter(hfdcan, &hbFilterConfig) != HAL_OK) {
	  Error_Handler();
	}

	if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, 5, 0) != HAL_OK) {
	  Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) {
	  Error_Handler();
	}

	HAL_FDCAN_Start(hfdcan);
}
}
