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
#include <stdint.h>
#include <stddef.h>

#define		_ENCODER_SPI			hspi3
#define     _ENCODER_NSS_GPIO GPIOA
#define     _ENCODER_NSS_PIN GPIO_PIN_15
#define 	_ENCODER_READMASK 0x3fff


/**
  * @brief  Read encoder data into given var
  * @param  given variable by ref
  * @param  timeout Timeout duration
  * @retval bool status
*/
void as50_readAngle(uint16_t * data, uint32_t timeout);


/**
  * @brief  Set encoder zero angle to current angle
  * @param  timeout Timeout duration
  * @retval bool status
  */
void as50_setZero(uint32_t timeout);

void as50_write(uint16_t address, uint16_t data);


#endif /* INC_AS50XX_H_ */
