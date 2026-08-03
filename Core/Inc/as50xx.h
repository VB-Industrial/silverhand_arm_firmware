/*
 * as50xx.h
 *
 *  Created on: Jul 31, 2023
 *      Author: VR
 */

#ifndef INC_AS50XX_H_
#define INC_AS50XX_H_


#include "stm32g4xx_hal.h"
#include "spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define		_ENCODER_SPI			hspi3
#define     _ENCODER_NSS_GPIO GPIOA
#define     _ENCODER_NSS_PIN GPIO_PIN_15
#define 	_ENCODER_READMASK 0x3fff

typedef struct {
	uint16_t raw_frame;
	uint32_t transfer_count;
	uint32_t error_count;
	HAL_StatusTypeDef last_hal_status;
	bool last_read_ok;
	bool has_valid_angle;
} as50_diagnostics_t;


/**
  * @brief  Read encoder data into given var
  * @param  given variable by ref
  * @param  timeout Timeout duration
  * @retval bool status
*/
bool as50_readAngle(uint16_t * data, uint32_t timeout);

/**
  * @brief  Copy the passive SPI diagnostics collected by as50_readAngle
  */
void as50_getDiagnostics(as50_diagnostics_t * diagnostics);


/**
  * @brief  Set encoder zero angle to current angle
  * @param  timeout Timeout duration
  * @retval bool status
  */
void as50_setZero(uint32_t timeout);

void as50_write(uint16_t address, uint16_t data);


#endif /* INC_AS50XX_H_ */
