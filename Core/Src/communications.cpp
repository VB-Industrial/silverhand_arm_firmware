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
static constexpr size_t NUMBER_OF_REGISTERS = 29;

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
    set_register_int32(v_out, motor_position_steps());
    response.persistent = true;
    response._mutable = true;
}

void enc_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    set_register_int32(v_out, motor_encoder_raw());
    response.persistent = true;
    response._mutable = true;
}

void enc_status_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    motor_encoder_diagnostics diagnostics{};
    motor_encoder_get_diagnostics(&diagnostics);
    const int32_t values[] = {
        diagnostics.last_read_ok ? 1 : 0,
        diagnostics.last_hal_status,
        static_cast<int32_t>(diagnostics.transfer_count),
        static_cast<int32_t>(diagnostics.error_count),
        static_cast<int32_t>(diagnostics.raw_frame),
        diagnostics.has_valid_angle ? 1 : 0,
    };
    set_register_int32_array(v_out, values, sizeof(values) / sizeof(values[0]));
    response.persistent = false;
    response._mutable = false;
}

void fus_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    set_register_real32(v_out, motor_fused_angle_manipulator());
    response.persistent = true;
    response._mutable = true;
}

void fusion_diag_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    motor_fusion_diagnostics diagnostics{};
    motor_fusion_get_diagnostics(&diagnostics);
    const float values[] = {
        diagnostics.encoder_angle_rad,
        diagnostics.tmc_angle_rad,
        diagnostics.offset_rad,
        diagnostics.fused_angle_rad,
        diagnostics.encoder_error_rad,
        diagnostics.backlash_rad,
        diagnostics.calibrated_encoder ? 1.0F : 0.0F,
    };
    set_register_real32_array(v_out, values, std::size(values));
    response.persistent = false;
    response._mutable = false;
}

void servo_diag_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    motor_servo_diagnostics diagnostics{};
    motor_servo_get_diagnostics(&diagnostics);
    const float values[] = {
        static_cast<float>(diagnostics.state),
        diagnostics.target_position_rad,
        diagnostics.position_error_rad,
        diagnostics.command_velocity_rad_s,
        static_cast<float>(diagnostics.command_age_ms),
    };
    set_register_real32_array(v_out, values, std::size(values));
    response.persistent = false;
    response._mutable = false;
}

void reset_reason_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    set_register_int32(v_out, static_cast<int32_t>(system_watchdog_reset_reason()));
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
    set_register_int32(v_out, motor_driver_enabled() ? 1 : 0);
    response.persistent = true;
    response._mutable = true;
}

void tmc_state_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_driver_state());
    response.persistent = false;
    response._mutable = false;
}

void tmc_error_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_driver_error());
    response.persistent = false;
    response._mutable = false;
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

void fault_active_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, static_cast<int32_t>(motor_fault_active()));
    response.persistent = false;
    response._mutable = false;
}

void fault_latched_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, static_cast<int32_t>(motor_fault_latched()));
    response.persistent = false;
    response._mutable = false;
}

void fault_level_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_fail_level());
    response.persistent = false;
    response._mutable = false;
}

void stop_reason_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_stop_reason());
    response.persistent = false;
    response._mutable = false;
}

void network_state_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_network_state());
    response.persistent = false;
    response._mutable = false;
}

void controller_state_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_controller_state());
    response.persistent = false;
    response._mutable = false;
}

void control_mode_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, motor_control_mode_get());
    response.persistent = false;
    response._mutable = false;
}

void calibration_command_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t command = 0;
    if (try_get_register_int32(v_in, command)) {
        motor_calibration_command(command);
    }
    set_register_int32(v_out, motor_calibration_state());
    response.persistent = false;
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

void calibration_next_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t command = 0;
    if (try_get_register_int32(v_in, command) && (command == 1)) {
        motor_calibration_next();
    }
    set_register_int32(v_out, motor_calibration_state());
    response.persistent = false;
    response._mutable = true;
}

void calibration_state_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    const int32_t values[] = {
        motor_calibration_state(),
        motor_calibration_error(),
        motor_calibration_progress(),
    };
    set_register_int32_array(v_out, values, std::size(values));
    response.persistent = false;
    response._mutable = false;
}

void calibration_result_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response)
{
    int32_t values[13]{};
    uint8_t count = 0U;
    motor_calibration_result(values, static_cast<uint8_t>(std::size(values)), &count);
    set_register_int32_array(v_out, values, count);
    response.persistent = false;
    response._mutable = false;
}

void fault_log_count_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    set_register_int32(v_out, static_cast<int32_t>(motor_fault_log_count()));
    response.persistent = false;
    response._mutable = false;
}

void fault_log_last_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    fault_log_record record{};
    motor_fault_log_last(&record);
    const int32_t values[] = {
        static_cast<int32_t>(record.sequence),
        static_cast<int32_t>(record.uptime_ms),
        static_cast<int32_t>(record.fault_mask),
        static_cast<int32_t>(record.tmc_gstat),
        static_cast<int32_t>(record.tmc_drv_status),
    };
    set_register_int32_array(v_out, values, sizeof(values) / sizeof(values[0]));
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
            RegisterDefinition{"move", move_handler},
            RegisterDefinition{"pos_set", pos_set_handler},
            RegisterDefinition{"pos_get", pos_get_handler},
            RegisterDefinition{"enc_get", enc_get_handler},
            RegisterDefinition{"enc_status", enc_status_handler},
            RegisterDefinition{"arm", arm_handler},
            RegisterDefinition{"tmc_state", tmc_state_handler},
            RegisterDefinition{"tmc_error", tmc_error_handler},
            RegisterDefinition{"fus_get", fus_get_handler},
            RegisterDefinition{"fusion_diag", fusion_diag_handler},
            RegisterDefinition{"servo_diag", servo_diag_handler},
            RegisterDefinition{"reset_reason", reset_reason_handler},
            RegisterDefinition{"fail_ack", fail_ack_handler},
            RegisterDefinition{"fault_active", fault_active_handler},
            RegisterDefinition{"fault_latched", fault_latched_handler},
            RegisterDefinition{"fault_level", fault_level_handler},
            RegisterDefinition{"stop_reason", stop_reason_handler},
            RegisterDefinition{"network_state", network_state_handler},
            RegisterDefinition{"controller_state", controller_state_handler},
            RegisterDefinition{"control_mode", control_mode_handler},
            RegisterDefinition{"cal_cmd", calibration_command_handler},
            RegisterDefinition{"auto_cal", auto_calibration_handler},
            RegisterDefinition{"luft_cal", backlash_calibration_handler},
            RegisterDefinition{"zero_cal", zero_calibration_handler},
            RegisterDefinition{"cal_next", calibration_next_handler},
            RegisterDefinition{"cal_state", calibration_state_handler},
            RegisterDefinition{"cal_result", calibration_result_handler},
            RegisterDefinition{"fault_log_count", fault_log_count_handler},
            RegisterDefinition{"fault_log_last", fault_log_last_handler},
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
