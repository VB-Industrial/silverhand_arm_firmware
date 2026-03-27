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

#define DEFAULT_VELOCITY_IN_STEPS 20000



#define BYTE(value, n)    (((value) >> ((n) << 3)) & 0xFF)

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
void tmc5160_init(int8_t init_irun, int8_t direction);

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

void tmc5160_set_default_vel();

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
