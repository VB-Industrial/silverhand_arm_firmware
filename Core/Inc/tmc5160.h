/*
 * tmc5160.h
 *
 *  Created on: Aug 11, 2023
 *      Author: VR
 */

#ifndef INC_TMC5160_H_
#define INC_TMC5160_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "utility.h"

#include "spi.h"


#define		_STEPPER_MOTOR_DRIVER_SPI			hspi1
#define		_STEPPER_MOTOR_DRIVER_USE_FREERTOS		0 /* FreeRTOS by default */
#define     _STEPPER_MOTOR_DRIVER_NSS_GPIO GPIOA
#define     _STEPPER_MOTOR_DRIVER_NSS_PIN GPIO_PIN_4
#define nop() asm volatile("nop")

#define BYTE(value, n)    (((value) >> ((n) << 3)) & 0xFF)

#define TMC5160_REG_GCONF        0x80U
#define TMC5160_REG_GSTAT        0x01U
#define TMC5160_REG_IOIN         0x04U
#define TMC5160_REG_TSTEP        0x12U
#define TMC5160_REG_IHOLD_IRUN   0x90U
#define TMC5160_REG_TPOWERDOWN   0x91U
#define TMC5160_REG_TPWM_THRS    0x93U
#define TMC5160_REG_RAMP_STAT    0x35U
#define TMC5160_REG_RAMPMODE     0xA0U
#define TMC5160_REG_XACTUAL      0xA1U
#define TMC5160_REG_VACTUAL      0x22U
#define TMC5160_REG_VSTART       0xA3U
#define TMC5160_REG_A1           0xA4U
#define TMC5160_REG_V1           0xA5U
#define TMC5160_REG_AMAX         0xA6U
#define TMC5160_REG_VMAX         0xA7U
#define TMC5160_REG_DMAX         0xA8U
#define TMC5160_REG_D1           0xAAU
#define TMC5160_REG_VSTOP        0xABU
#define TMC5160_REG_XTARGET      0xADU
#define TMC5160_REG_XACTUAL_READ 0x21U
#define TMC5160_REG_DRV_STATUS   0x6FU
#define TMC5160_REG_CHOPCONF     0xECU
#define TMC5160_REG_PWMCONF      0xF0U

/**
  * @brief  Read driver register into given var
  * @param  given variable by ref
  * @param  timeout Timeout duration
  * @retval bool status
  */

/*Init
 *
 *
 *
 *
 */
void tmc5160_init(int8_t init_irun);

/*Position in ticks
 *
 *
 *
 *
 *
 */
void tmc5160_position(int32_t position);

void tmc5160_move(int32_t vel);

void tmc5160_acceleration(uint32_t acc);

int32_t tmc5160_position_read();

void tmc5160_velocity(int32_t vel);

void tmc5160_apply_default_motion_profile();

int32_t tmc5160_velocity_read();

int32_t tmc5160_read_reg(uint8_t reg_addr);

void tmc5160_effort(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun);

void tmc5160_set_motor_direction(int8_t);

void tmc5160_set_zero();

void tmc5160_arm();

void tmc5160_disarm();

void tmc5160_stop();

void tmc5160_write(uint8_t* data);

void tmc5160_read(uint8_t* WData, uint8_t* RData);

uint8_t tmc5160_torque_to_current(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun);

#endif /* INC_TMC5160_H_ */
