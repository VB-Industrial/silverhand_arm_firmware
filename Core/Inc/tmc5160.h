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

#define TMC5160_FAULT_UNDERVOLTAGE       (1UL << 0U)
#define TMC5160_FAULT_OVERTEMP_WARNING   (1UL << 1U)
#define TMC5160_FAULT_OVERTEMP           (1UL << 2U)
#define TMC5160_FAULT_SHORT_TO_GROUND    (1UL << 3U)
#define TMC5160_FAULT_SHORT_TO_SUPPLY    (1UL << 4U)
#define TMC5160_FAULT_DRIVER_ERROR       (1UL << 5U)

#define TMC5160_CRITICAL_FAULT_MASK \
    (TMC5160_FAULT_UNDERVOLTAGE | TMC5160_FAULT_OVERTEMP | \
     TMC5160_FAULT_SHORT_TO_GROUND | TMC5160_FAULT_SHORT_TO_SUPPLY | \
     TMC5160_FAULT_DRIVER_ERROR)

typedef struct tmc5160_fault_snapshot {
    uint32_t flags;
    uint32_t gstat;
    uint32_t driver_status;
} tmc5160_fault_snapshot;

typedef struct tmc5160_health_snapshot {
    tmc5160_fault_snapshot faults;
    bool driver_enabled;
} tmc5160_health_snapshot;

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
bool tmc5160_init(int8_t init_irun);

/*Position in ticks
 *
 *
 *
 *
 *
 */
void tmc5160_position(int32_t position, int32_t velocity_steps_per_second);

void tmc5160_move(int32_t vel);

void tmc5160_acceleration(uint32_t acc);

int32_t tmc5160_position_read();

void tmc5160_velocity(int32_t vel);

void tmc5160_apply_default_motion_profile();

int32_t tmc5160_velocity_read();

bool tmc5160_position_reached(void);

int32_t tmc5160_read_reg(uint8_t reg_addr);
uint32_t tmc5160_current_configuration(void);

void tmc5160_clear_gstat(uint32_t flags);

bool tmc5160_read_driver_enabled(bool* enabled);

bool tmc5160_configuration_matches(void);
bool tmc5160_runtime_configuration_matches(void);

bool tmc5160_health_check(tmc5160_health_snapshot* snapshot);

void tmc5160_effort(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun);

void tmc5160_set_run_current(uint8_t irun);

void tmc5160_set_motor_direction(int8_t);

void tmc5160_set_zero();

void tmc5160_arm();

void tmc5160_disarm();

bool tmc5160_stop(uint32_t timeout_ms);

void tmc5160_write(uint8_t* data);

void tmc5160_read(uint8_t* WData, uint8_t* RData);

uint8_t tmc5160_torque_to_current(double effort, float max_effort, int8_t max_irun_scaler, int8_t init_irun);

#endif /* INC_TMC5160_H_ */
