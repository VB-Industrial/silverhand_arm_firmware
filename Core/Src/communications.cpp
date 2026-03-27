#include "main.h"
#include "robot_config.h"

#include <memory>

#include "cyphal/cyphal.h"
#include "cyphal/node/node_info_handler.h"
#include "cyphal/node/registers_handler.hpp"
#include "cyphal/providers/G4CAN.h"
#include "cyphal/allocators/sys/sys_allocator.h"
#include "cyphal/subscriptions/subscription.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/scalar/Integer32_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>

#include <uavcan/node/GetInfo_1_0.h>

extern "C" {
#include "as50xx.h"
#include "utility.h"
#include "communications.h"

extern uint16_t enc_angle;

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)

static uint32_t zero_enc_runtime = 0;
static uint16_t prev_enc_angle = 0;
static uint32_t prev_fusion_ts_ms = 0;
static float enc_velocity_lpf_rad_s = 0.0F;
static float fused_angle_rad = 0.0F;
static float fused_velocity_rad_s = 0.0F;
static float startup_tmc_angle_offset_rad = 0.0F;

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
        UNUSED(transfer);
    }
};


HBeatReader* h_reader;

class JSReader: public AbstractSubscription<JS_msg> {
public:
	JSReader(InterfacePtr interface): AbstractSubscription<JS_msg>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
		kRobotJointProfile->js_sub_port_id
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_in, CanardRxTransfer* transfer) override
    {
        UNUSED(transfer);
    	const float velocity = js_in.angular_velocity.radian_per_second;
    	const float position = js_in.angular_position.radian;
    	const float acceleration = js_in.angular_acceleration.radian_per_second_per_second;

    		if(acceleration != 0.0F)
    		{
    			//tmc5160_velocity(rad_to_steps(velocity, kRobotJointProfile->joint_full_steps));
    			tmc5160_move(rad_to_steps(acceleration, kRobotJointProfile->joint_full_steps));
    		}
    		else if((fabs(velocity) < 0.0001F) && (velocity != 0.0F))
    		{
    			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    	    	tmc5160_velocity(rad_to_steps(20000, kRobotJointProfile->joint_full_steps));
    	    	tmc5160_position(rad_to_steps(position, kRobotJointProfile->joint_full_steps));
    		}
    		else if(velocity == 0.0F)
    		{
    			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    			tmc5160_acceleration(kRobotJointProfile->joint_full_steps); //FULL THROTTLE!!!
    	    	tmc5160_velocity(rad_to_steps(kRobotJointProfile->joint_full_steps, kRobotJointProfile->joint_full_steps)); //FULL PULL!
    	    	tmc5160_position(rad_to_steps(position, kRobotJointProfile->joint_full_steps));
    		}
    		else
    		{
    			tmc5160_move(rad_to_steps(velocity, kRobotJointProfile->joint_full_steps));
    		}
    }
};

JSReader* js_reader;
NodeInfoReader* nireader;
static constexpr size_t NUMBER_OF_REGISTERS = 6;

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

static void set_register_int32(uavcan_register_Value_1_0& value, const int32_t data)
{
    value._tag_ = 5;
    value.integer32.value.elements[0] = data;
    value.integer32.value.count = 1;
}

static void set_register_real32(uavcan_register_Value_1_0& value, const float data)
{
    value._tag_ = 13;
    value.real32.value.elements[0] = data;
    value.real32.value.count = 1;
}

static float encoder_fused_angle_radians(void)
{
    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;

    int32_t delta_ticks = static_cast<int32_t>(enc_angle) - static_cast<int32_t>(zero_enc_runtime);
    if (delta_ticks > kHalfTurnTicks) {
        delta_ticks -= kEncoderTicksPerTurn;
    } else if (delta_ticks < -kHalfTurnTicks) {
        delta_ticks += kEncoderTicksPerTurn;
    }

    const float radians_per_tick = (2.0F * static_cast<float>(M_PI)) / static_cast<float>(kEncoderTicksPerTurn);
    return static_cast<float>(kRobotJointProfile->direction) * static_cast<float>(delta_ticks) * radians_per_tick;
}

static float tmc_angle_radians(void)
{
    return steps_to_rads(tmc5160_position_read(), kRobotJointProfile->joint_full_steps);
}

static float tmc_corrected_angle_radians(void)
{
    return tmc_angle_radians() + startup_tmc_angle_offset_rad;
}

static float tmc_velocity_radians_per_second(void)
{
    return steps_to_rads(tmc5160_velocity_read(), kRobotJointProfile->joint_full_steps);
}

static void update_fusion_state(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const float tmc_angle = tmc_corrected_angle_radians();
    const float tmc_velocity = tmc_velocity_radians_per_second();

    if (!kRobotJointProfile->has_output_encoder) {
        fused_angle_rad = tmc_angle;
        fused_velocity_rad_s = tmc_velocity;
        prev_enc_angle = enc_angle;
        prev_fusion_ts_ms = now_ms;
        enc_velocity_lpf_rad_s = 0.0F;
        return;
    }

    const float encoder_angle = encoder_fused_angle_radians();
    fused_angle_rad = (kRobotJointProfile->angle_encoder_weight * encoder_angle) +
                      (kRobotJointProfile->angle_tmc_weight * tmc_angle);

    if (prev_fusion_ts_ms == 0U) {
        prev_enc_angle = enc_angle;
        prev_fusion_ts_ms = now_ms;
        fused_velocity_rad_s = tmc_velocity;
        return;
    }

    const uint32_t dt_ms = now_ms - prev_fusion_ts_ms;
    if (dt_ms == 0U) {
        fused_velocity_rad_s = tmc_velocity;
        return;
    }

    constexpr int32_t kEncoderTicksPerTurn = _ENCODER_READMASK + 1;
    constexpr int32_t kHalfTurnTicks = kEncoderTicksPerTurn / 2;
    int32_t delta_ticks = static_cast<int32_t>(enc_angle) - static_cast<int32_t>(prev_enc_angle);
    if (delta_ticks > kHalfTurnTicks) {
        delta_ticks -= kEncoderTicksPerTurn;
    } else if (delta_ticks < -kHalfTurnTicks) {
        delta_ticks += kEncoderTicksPerTurn;
    }

    const float dt_s = static_cast<float>(dt_ms) / 1000.0F;
    const float radians_per_tick = (2.0F * static_cast<float>(M_PI)) / static_cast<float>(kEncoderTicksPerTurn);
    const float enc_velocity_raw = (static_cast<float>(kRobotJointProfile->direction) * static_cast<float>(delta_ticks) * radians_per_tick) / dt_s;
    const float lpf_alpha = kRobotJointProfile->velocity_encoder_lpf_alpha;
    enc_velocity_lpf_rad_s = (lpf_alpha * enc_velocity_raw) + ((1.0F - lpf_alpha) * enc_velocity_lpf_rad_s);

    fused_velocity_rad_s = (kRobotJointProfile->velocity_tmc_weight * tmc_velocity) +
                           (kRobotJointProfile->velocity_encoder_weight * enc_velocity_lpf_rad_s);

    prev_enc_angle = enc_angle;
    prev_fusion_ts_ms = now_ms;
}

void fusion_startup_sync(void)
{
    startup_tmc_angle_offset_rad = 0.0F;
    if (kRobotJointProfile->has_output_encoder) {
        startup_tmc_angle_offset_rad = encoder_fused_angle_radians() - tmc_angle_radians();
    }

    prev_enc_angle = enc_angle;
    prev_fusion_ts_ms = HAL_GetTick();
    enc_velocity_lpf_rad_s = 0.0F;
    fused_angle_rad = tmc_corrected_angle_radians();
    fused_velocity_rad_s = 0.0F;
}

void move_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t velocity_command = tmc5160_velocity_read();
    if (try_get_register_int32(v_in, velocity_command)) {
        tmc5160_arm();
        tmc5160_move(velocity_command);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
    set_register_int32(v_out, tmc5160_velocity_read());
    response.persistent = true;
    response._mutable = true;
}

void pos_set_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t target_position = 0;
    if (try_get_register_int32(v_in, target_position)) {
        tmc5160_set_default_vel();
        tmc5160_position(target_position);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }

    set_register_int32(v_out, tmc5160_position_read());
    response.persistent = true;
    response._mutable = true;
}

void pos_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    set_register_int32(v_out, tmc5160_position_read());
    response.persistent = true;
    response._mutable = true;
}

void enc_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    set_register_int32(v_out, enc_angle);
    response.persistent = true;
    response._mutable = true;
}

void fus_get_handler(
    const uavcan_register_Value_1_0&,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    set_register_real32(v_out, fused_angle_rad);
    response.persistent = true;
    response._mutable = true;
}

void arm_handler(
    const uavcan_register_Value_1_0& v_in,
    uavcan_register_Value_1_0& v_out,
    RegisterAccessResponse::Type& response
) {
    int32_t arm_command = 0;
    if (try_get_register_int32(v_in, arm_command)) {
        if (arm_command != 0) {
            tmc5160_arm();
        } else {
            tmc5160_disarm();
        }
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
    set_register_int32(v_out, arm_command);
    response.persistent = true;
    response._mutable = true;
}

void send_JS(void) {             //float* pos, float* vel, float* eff
	static CanardTransferID int_transfer_id = 0;
    update_fusion_state();
	reg_udral_physics_kinematics_rotation_Planar_0_1 js_msg =
	{
			.angular_position = fused_angle_rad,
			.angular_velocity = fused_velocity_rad_s,
			.angular_acceleration = 0.0F
	};
    interface->send_msg<JS_msg>(
		&js_msg,
		kRobotJointProfile->agent_js_sub_port_id,
		&int_transfer_id
	);
}

void heartbeat() {
	static CanardTransferID hbeat_transfer_id = 0;
	static uint32_t uptime = 0;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    uavcan_node_Heartbeat_1_0 heartbeat_msg = {
        .uptime = uptime,
        .health = {uavcan_node_Health_1_0_NOMINAL},
        .mode = {uavcan_node_Mode_1_0_OPERATIONAL},
        .vendor_specific_status_code = 0
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
    zero_enc_runtime = kRobotJointProfile->default_zero_enc;
    fusion_startup_sync();
	registers_handler = new RegistersHandler<NUMBER_OF_REGISTERS>(
        {
            RegisterDefinition{"move", move_handler},
            RegisterDefinition{"pos_set", pos_set_handler},
            RegisterDefinition{"pos_get", pos_get_handler},
            RegisterDefinition{"enc_get", enc_get_handler},
            RegisterDefinition{"arm", arm_handler},
            RegisterDefinition{"fus_get", fus_get_handler},
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
	CanardFilter cyphal_filter_for_JS = canardMakeFilterForSubject(kRobotJointProfile->js_sub_port_id);//robot_joint_state_sub_port_id() //1121
	CanardFilter cyphal_filter_for_HB = canardMakeFilterForSubject(kRobotJointProfile->heartbeat_subject_id);//robot_joint_state_sub_port_id()
	static FDCAN_FilterTypeDef sFilterConfig;
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

	hbFilterConfig.IdType = FDCAN_EXTENDED_ID;
	hbFilterConfig.FilterIndex = 2;
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
