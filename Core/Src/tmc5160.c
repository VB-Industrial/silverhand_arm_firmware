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

void tmc5160_position(int32_t position)
{
	uint8_t WData[5] = {0};

	WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; //SPI send: 0xA000000000; // RAMPMODE = 1 (position move)
	  tmc5160_write(WData);

	WData[0] = 0xAD; //moving register
	WData[1] = (position & 0xFF000000) >> 24; //position in steps
	WData[2] = (position & 0x00FF0000) >> 16;
	WData[3] = (position & 0x0000FF00) >> 8;
	WData[4] = (position & 0x000000FF);
	tmc5160_write(WData);
}

void tmc5160_move(int32_t vel)
{

	int32_t vel_to_go;
	vel_to_go = (int32_t)(vel*1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet

	uint8_t WData[5] = {0};
	if (vel_to_go < 0) //select positive or negative mode depending on vel sign
	{
		  WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x02; //SPI send: 0xA000000001; // RAMPMODE = 1 (positive velocity move)
		  tmc5160_write(WData);
	}
	else
	{
		  WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x01; //SPI send: 0xA000000001; // RAMPMODE = 2 (negative velocity move)
		  tmc5160_write(WData);
	}
	vel_to_go = abs(vel_to_go);

	int32_t v1;
	v1 = (int32_t)(vel_to_go*0.1);


	//Acceleration threshold velocity V1
	WData[0] = 0xA5; //V1 speed register
	WData[1] = (v1 & 0xFF000000) >> 24;
	WData[2] = (v1 & 0x00FF0000) >> 16;
	WData[3] = (v1 & 0x0000FF00) >> 8;
	WData[4] = (v1 & 0x000000FF);
	tmc5160_write(WData);

	//sending VMAX
	WData[0] = 0xA7; //VMAX speed register
	WData[1] = (vel_to_go & 0xFF000000) >> 24;
	WData[2] = (vel_to_go & 0x00FF0000) >> 16;
	WData[3] = (vel_to_go & 0x0000FF00) >> 8;
	WData[4] = (vel_to_go & 0x000000FF);
	tmc5160_write(WData);
}

void tmc5160_apply_default_motion_profile()
{
	uint8_t WData[5] = {0};

	// Apply an intentionally aggressive default accel/decel profile.
	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x03; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);

	WData[0] = 0xA4; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);

	WData[0] = 0xA6; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);

	WData[0] = 0xA8; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);

	WData[0] = 0xAA; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);

	WData[0] = 0xAB; WData[1] = 0x00; WData[2] = 0x03; WData[3] = 0xFF; WData[4] = 0xFF;
	tmc5160_write(WData);
}

void tmc5160_velocity(int32_t vel)
{

	int32_t vel_to_go;
	vel_to_go = (int32_t)(vel*1.3981013); //1.3981.. is the time ratio according to "Microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP" see ref on p. 81 of datasheet
	vel_to_go = abs(vel_to_go);

	int32_t V1;
	V1 = (int32_t)(vel_to_go*0.1);

	//Acceleration threshold velocity V1
	uint8_t WData[5] = {0};

	WData[0] = 0xA5; //V1 speed register
	WData[1] = (V1 & 0xFF000000) >> 24;
	WData[2] = (V1 & 0x00FF0000) >> 16;
	WData[3] = (V1 & 0x0000FF00) >> 8;
	WData[4] = (V1 & 0x000000FF);
	tmc5160_write(WData);

	//VMAX
	WData[0] = 0xA7; //VMAX speed register
	WData[1] = (vel_to_go & 0xFF000000) >> 24;
	WData[2] = (vel_to_go & 0x00FF0000) >> 16;
	WData[3] = (vel_to_go & 0x0000FF00) >> 8;
	WData[4] = (vel_to_go & 0x000000FF);
	tmc5160_write(WData);

}

void tmc5160_effort(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun)
{
	uint8_t IRUN = 0;
	uint8_t IHOLD = 0;
	uint8_t WData[5] = {0};

	IRUN = tmc5160_torque_to_current(effort, max_effort, max_irun_scaler, init_irun);
	IHOLD = IRUN >> 1;

	WData[0] = 0x90;
	WData[1] = 0x00;
	WData[2] = 0x00; // IHOLDDELAY=0 TODO?
	WData[3] = IRUN; // IRUN
	WData[4] = IHOLD; // IHOLD

	tmc5160_write(WData);
}


void tmc5160_acceleration(uint32_t acc)
{
	uint8_t WData[5] = {0};
	uint8_t Tk = 65; //time constant. Acceleration equals steps/Tk see ref on p. 81 of datasheet
	acc = (uint32_t)(acc / Tk);

	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0F; // Start acceleration = 15 (Near start)
	tmc5160_write(WData);

	// A1 First acceleration
	WData[0] = 0xA4;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// AMAX Acceleration above V1
	WData[0] = 0xA6;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// DMAX Deceleration above V1
	WData[0] = 0xA8;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	// D1 Deceleration below V1
	WData[0] = 0xAA;
	WData[3] = (acc & 0x0000FF00) >> 8;
	WData[4] = (acc & 0x000000FF);
	tmc5160_write(WData);

	WData[0] = 0xAB; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0F; // VSTOP = 15 Stop velocity (Near to zero)
	tmc5160_write(WData);
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
	return tmc5160_read_reg(0x21);
}

int32_t tmc5160_velocity_read()
{
	uint8_t WData[5] = {0};
	uint8_t RData[5] = {0};
	WData[0] = 0x22; //VACTUAL register address
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
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // DRV_ENN: keep driver disabled during strap setup
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // SPI_MODE ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // SD_MODE OFF, internal ramp generator ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS HIGH
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // DIR
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // STEP
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // enable driver after mode pins are valid
	HAL_Delay(100);

	uint8_t WData[5] = {0};

	WData[0] = 0xEC; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (SpreadCycle)
	tmc5160_write(WData);

	WData[0] = 0x90; WData[1] = 0x00; WData[2] = 0x00; WData[3] = init_irun; WData[4] = init_irun; //  IHOLDDELAY=0,  IRUN=10/31,  IHOLD=02/31
	tmc5160_write(WData);

	WData[0] = 0x91; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x0A; // TPOWERDOWN=10: Delay before power down in stand still
	tmc5160_write(WData);

	//0xC40C001E default for 0x70 reg
	WData[0] = 0xF0; WData[1] = 0xC4; WData[2] = 0x0D; WData[3] = 0x00; WData[4] = 0x1E; // PWM_CONF PWM_FREQ 35kHz TODO
	tmc5160_write(WData);

	WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x04; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	tmc5160_write(WData);

	WData[0] = 0x93; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0xC8; // TPWM_THRS=200 yields a switching velocity about 35000 = ca. 30RPM
	tmc5160_write(WData);

	// WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; //SPI send: 0xA000000000; // RAMPMODE = 0 (Target position move)
	// tmc5160_write(WData);

	tmc5160_apply_default_motion_profile();

	HAL_Delay(100);
}


void tmc5160_set_motor_direction(int8_t dir)
{
	if(dir <= 0)
	{
	  uint8_t WData[5] = {0};
	  WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x14; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	  tmc5160_write(WData);
	}
	else
	{
	  uint8_t WData[5] = {0};
	  WData[0] = 0x80; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x04; // EN_PWM_MODE=1 enables StealthChop (with default PWMCONF)
	  tmc5160_write(WData);
	}
}

void tmc5160_set_zero()
{
	uint8_t WData[5] = {0};
	WData[0] = 0xA0; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x03; // RAMPMODE = 3 (HOLD mode)
	tmc5160_write(WData);

	WData[0] = 0xA1; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; // Set zero
	tmc5160_write(WData);
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
	uint8_t WData[5] = {0};
	uint32_t pos = 0;

	WData[0] = 0xA3; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00; // Start acceleration = 10 (Near start)
	tmc5160_write(WData);

	//VMAX
	WData[0] = 0xA7; WData[1] = 0x00; WData[2] = 0x00; WData[3] = 0x00; WData[4] = 0x00;
	tmc5160_write(WData);

	pos = tmc5160_position_read();
	tmc5160_position(pos);
}


uint8_t tmc5160_torque_to_current(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun)
{
	uint8_t IRUN = 0;
	effort = (effort > max_effort) ? max_effort : effort; //TODO use clamp no ref function from utility
	IRUN = (effort / max_effort) * max_irun_scaler;
	IRUN = (IRUN < init_irun) ? init_irun : IRUN;
	return IRUN;
}
