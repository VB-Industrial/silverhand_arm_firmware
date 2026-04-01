/*
 * tmc5160.c
 *
 *  Created on: Aug 11, 2023
 *      Author: VR
 */


/*

#define NEMA14_MAX_IRUN_SCALER 10
#define NEMA17_MAX_IRUN_SCALER 12
#define NEMA23_MAX_IRUN_SCALER 31

//TODO to research global scaler adjust for different types of motor to achieve full range of current control. For now just limits the IRUN multiplier.
#if MOTOR == NEMA14
	#define MOTOR_MAX_IRUN_SCALER NEMA14_MAX_IRUN_SCALER
#elif MOTOR == NEMA17
	#define MOTOR_MAX_IRUN_SCALER NEMA17_MAX_IRUN_SCALER
#elif MOTOR == NEMA23
	#define MOTOR_MAX_IRUN_SCALER NEMA23_MAX_IRUN_SCALER
#endif

//MAX HOLDING TORQUE * gear ratio / 5 (which is empirical decrease for moving torque)
#define NEMA14_MAX_TORQUE 0.5
#define NEMA17_MAX_TORQUE 3.9
#define NEMA23_MAX_TORQUE 10.2

#if MOTOR == NEMA14
	#define MOTOR_MAX_TORQUE NEMA14_MAX_TORQUE
#elif MOTOR == NEMA17
	#define MOTOR_MAX_TORQUE NEMA17_MAX_TORQUE
#elif MOTOR == NEMA23
	#define MOTOR_MAX_TORQUE NEMA23_MAX_TORQUE
#endif

// 200 (fullsteps) * 256 (microsteps) * Gear ratio
#define NEMA14_FULLSTEPS    983204
#define NEMA17_FULLSTEPS	2560000
#define NEMA23_FULLSTEPS    2560000

#if MOTOR == NEMA14
	#define MOTOR_FULLSTEP NEMA14_FULLSTEPS
#elif MOTOR == NEMA17
	#define MOTOR_FULLSTEP NEMA17_FULLSTEPS
#elif MOTOR == NEMA23
	#define MOTOR_FULLSTEP NEMA23_FULLSTEPS
#endif

// Gear ratio
#define NEMA14_GR 19 //TODO possible to correct ratio to 19.38/187 for more precise velocity calculation
#define NEMA17_GR 50
#define NEMA23_GR 50

#if MOTOR == NEMA14
	#define MOTOR_GR NEMA14_GR
#elif MOTOR == NEMA17
	#define MOTOR_GR NEMA17_GR
#elif MOTOR == NEMA23
	#define MOTOR_GR NEMA23_GR
#endif

*/


#include "tmc5160.h"
#include <stdlib.h>


#if (USE_FREERTOS == 1)
#include "cmsis_os.h"
#define tmc5160_delay(x)   osDelay(x)
#else
#define tmc5160_delay(x)   HAL_Delay(x)
#endif

static void tmc5160_write_reg32(const uint8_t reg_addr, const uint32_t value)
{
	uint8_t WData[5] = {0};
	WData[0] = reg_addr;
	WData[1] = (value & 0xFF000000U) >> 24;
	WData[2] = (value & 0x00FF0000U) >> 16;
	WData[3] = (value & 0x0000FF00U) >> 8;
	WData[4] = (value & 0x000000FFU);
	tmc5160_write(WData);
}

static void tmc5160_configure_pins_for_spi_mode(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);   // DRV_ENN: keep driver disabled during strap setup
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // SPI_MODE ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // SD_MODE OFF, internal ramp generator ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // DIR
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // STEP
}

static void tmc5160_enable_driver_after_strap(void)
{
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(100);
}

static void tmc5160_set_rampmode_position(void)
{
	tmc5160_write_reg32(TMC5160_REG_RAMPMODE, 0x00000000U);
}

static void tmc5160_set_rampmode_velocity_positive(void)
{
	tmc5160_write_reg32(TMC5160_REG_RAMPMODE, 0x00000001U);
}

static void tmc5160_set_rampmode_velocity_negative(void)
{
	tmc5160_write_reg32(TMC5160_REG_RAMPMODE, 0x00000002U);
}

static void tmc5160_set_rampmode_hold(void)
{
	tmc5160_write_reg32(TMC5160_REG_RAMPMODE, 0x00000003U);
}

static void tmc5160_set_xactual(const int32_t position)
{
	tmc5160_write_reg32(TMC5160_REG_XACTUAL, (uint32_t) position);
}

static void tmc5160_set_vstart(const uint32_t velocity)
{
	tmc5160_write_reg32(TMC5160_REG_VSTART, velocity);
}

static void tmc5160_set_a1(const uint32_t acceleration)
{
	tmc5160_write_reg32(TMC5160_REG_A1, acceleration);
}

static void tmc5160_set_v1(const uint32_t velocity)
{
	tmc5160_write_reg32(TMC5160_REG_V1, velocity);
}

static void tmc5160_set_amax(const uint32_t acceleration)
{
	tmc5160_write_reg32(TMC5160_REG_AMAX, acceleration);
}

static void tmc5160_set_vmax(const uint32_t velocity)
{
	tmc5160_write_reg32(TMC5160_REG_VMAX, velocity);
}

static void tmc5160_set_dmax(const uint32_t deceleration)
{
	tmc5160_write_reg32(TMC5160_REG_DMAX, deceleration);
}

static void tmc5160_set_d1(const uint32_t deceleration)
{
	tmc5160_write_reg32(TMC5160_REG_D1, deceleration);
}

static void tmc5160_set_vstop(const uint32_t velocity)
{
	tmc5160_write_reg32(TMC5160_REG_VSTOP, velocity);
}

static void tmc5160_set_xtarget(const int32_t position)
{
	tmc5160_write_reg32(TMC5160_REG_XTARGET, (uint32_t) position);
}

static void tmc5160_set_chopconf_spreadcycle_default(void)
{
	tmc5160_write_reg32(TMC5160_REG_CHOPCONF, 0x000000C3U);
}

static void tmc5160_set_current_levels(const uint8_t ihold, const uint8_t irun, const uint8_t ihold_delay)
{
	uint32_t value = 0U;
	value |= ((uint32_t) ihold_delay) << 16;
	value |= ((uint32_t) irun) << 8;
	value |= (uint32_t) ihold;
	tmc5160_write_reg32(TMC5160_REG_IHOLD_IRUN, value);
}

static void tmc5160_set_tpowerdown(const uint8_t delay)
{
	tmc5160_write_reg32(TMC5160_REG_TPOWERDOWN, (uint32_t) delay);
}

static void tmc5160_set_pwmconf(const uint32_t value)
{
	tmc5160_write_reg32(TMC5160_REG_PWMCONF, value);
}

static void tmc5160_set_gconf(const uint32_t value)
{
	tmc5160_write_reg32(TMC5160_REG_GCONF, value);
}

static void tmc5160_enable_stealthchop_default(void)
{
	tmc5160_set_gconf(0x00000004U);
}

static void tmc5160_set_tpwm_thrs(const uint32_t value)
{
	tmc5160_write_reg32(TMC5160_REG_TPWM_THRS, value);
}

void tmc5160_position(int32_t position)
{
	tmc5160_set_rampmode_position();
	tmc5160_set_xtarget(position);
}

void tmc5160_move(int32_t vel)
{

	int32_t vel_to_go;
	vel_to_go = (int32_t)(vel*1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet

	if (vel_to_go < 0) //select positive or negative mode depending on vel sign
	{
		  tmc5160_set_rampmode_velocity_negative();
	}
	else
	{
		  tmc5160_set_rampmode_velocity_positive();
	}
	vel_to_go = abs(vel_to_go);

	int32_t v1;
	v1 = (int32_t)(vel_to_go*0.1);

	tmc5160_set_v1((uint32_t) v1);
	tmc5160_set_vmax((uint32_t) vel_to_go);
}

void tmc5160_apply_default_motion_profile()
{
	// Apply an intentionally aggressive default accel/decel profile.
	tmc5160_set_vstart(0x0003FFFFU);
	tmc5160_set_a1(0x0000FFFFU);
	tmc5160_set_amax(0x0000FFFFU);
	tmc5160_set_dmax(0x0000FFFFU);
	tmc5160_set_d1(0x0000FFFFU);
	tmc5160_set_vstop(0x0003FFFFU);
}

void tmc5160_velocity(int32_t vel)
{

	int32_t vel_to_go;
	vel_to_go = (int32_t)(vel*1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
	vel_to_go = abs(vel_to_go);

	int32_t V1;
	V1 = (int32_t)(vel_to_go*0.1);

	tmc5160_set_v1((uint32_t) V1);
	tmc5160_set_vmax((uint32_t) vel_to_go);

}

void tmc5160_effort(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun)
{
	uint8_t IRUN = 0;
	uint8_t IHOLD = 0;

	IRUN = tmc5160_torque_to_current(effort, max_effort, max_irun_scaler, init_irun);
	IHOLD = IRUN >> 1;

	tmc5160_set_current_levels(IHOLD, IRUN, 0U);
}


void tmc5160_acceleration(uint32_t acc)
{
	uint8_t Tk = 65; //time constant. Acceleration equals steps/Tk see ref on p. 81 of datasheet
	acc = (uint32_t)(acc / Tk);

	tmc5160_set_vstart(0x0000000FU);
	tmc5160_set_a1(acc);
	tmc5160_set_amax(acc);
	tmc5160_set_dmax(acc);
	tmc5160_set_d1(acc);
	tmc5160_set_vstop(0x0000000FU);
}


void tmc5160_write(uint8_t* data)
{
	HAL_GPIO_WritePin(_STEPPER_MOTOR_DRIVER_NSS_GPIO, _STEPPER_MOTOR_DRIVER_NSS_PIN, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_Transmit(&_STEPPER_MOTOR_DRIVER_SPI, data, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(_STEPPER_MOTOR_DRIVER_NSS_GPIO, _STEPPER_MOTOR_DRIVER_NSS_PIN, GPIO_PIN_SET); //CS HIGH
}


void tmc5160_read(uint8_t* WData, uint8_t* RData)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_TransmitReceive(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
	nop();
	nop();
	nop();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS LOW
	HAL_SPI_TransmitReceive(&_STEPPER_MOTOR_DRIVER_SPI, WData, RData, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS HIGH
}


int32_t tmc5160_position_read()
{
	return tmc5160_read_reg(TMC5160_REG_XACTUAL_READ);
}

int32_t tmc5160_velocity_read()
{
	uint8_t WData[5] = {0};
	uint8_t RData[5] = {0};
	WData[0] = TMC5160_REG_VACTUAL;
	tmc5160_read(WData, RData);

	int32_t response = 0;

    response |= (RData[1] & 0xFF);
    response <<= 8;
    response |= (RData[2] & 0xFF);
    response <<= 8;
    response |= (RData[3] & 0xFF);
    response <<= 8;
    response |= (RData[4] & 0xFF);

    int32_t rv = 0;
    rv = sign_extend_bits_to_32(response, 24);

	return (int32_t)(rv / 1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
}

int32_t tmc5160_read_reg(uint8_t reg_addr)
{
	uint8_t WData[5] = {0};
	uint8_t RData[5] = {0};
	WData[0] = reg_addr & 0x7F;
	tmc5160_read(WData, RData);

	int32_t response = 0;
	response |= RData[1];
	response <<= 8;
	response |= RData[2];
	response <<= 8;
	response |= RData[3];
	response <<= 8;
	response |= RData[4];

	return response;
}

void tmc5160_init(int8_t init_irun)
{
	tmc5160_configure_pins_for_spi_mode();
	tmc5160_enable_driver_after_strap();

	tmc5160_set_chopconf_spreadcycle_default();
	tmc5160_set_current_levels((uint8_t) init_irun, (uint8_t) init_irun, 0U);
	tmc5160_set_tpowerdown(0x0AU);
	tmc5160_set_pwmconf(0xC40D001EU);
	tmc5160_enable_stealthchop_default();
	tmc5160_set_tpwm_thrs(0x000000C8U);

	tmc5160_set_zero();

	tmc5160_set_xactual(0);
	tmc5160_set_xtarget(0);
	tmc5160_set_rampmode_position();

	tmc5160_apply_default_motion_profile();

	HAL_Delay(100);
}


void tmc5160_set_motor_direction(int8_t dir)
{
	if(dir <= 0)
	{
	  tmc5160_set_gconf(0x00000014U);
	}
	else
	{
	  tmc5160_set_gconf(0x00000004U);
	}
}

void tmc5160_set_zero()
{
	tmc5160_set_rampmode_hold();
	tmc5160_set_xactual(0);
}

void tmc5160_disarm()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //DRV SLEEP 0 for power on, 1 for power off
}

void tmc5160_arm()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); //DRV SLEEP 0 for power on, 1 for power off
}

void tmc5160_stop()
{
	uint32_t pos = 0;

	tmc5160_set_vstart(0U);
	tmc5160_set_vmax(0U);

	pos = tmc5160_position_read();
	tmc5160_position((int32_t) pos);
}


uint8_t tmc5160_torque_to_current(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun)
{
	uint8_t IRUN = 0;
	effort = (effort > max_effort) ? max_effort : effort; //TODO use clamp no ref function from utility
	IRUN = (effort / max_effort) * max_irun_scaler;
	IRUN = (IRUN < init_irun) ? init_irun : IRUN;
	return IRUN;
}
