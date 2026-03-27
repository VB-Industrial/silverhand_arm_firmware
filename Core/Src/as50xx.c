/*
 * AS50xx.c
 *
 *  Created on: Jul 31, 2023
 *      Author: VR
 */

#include "as50xx.h"

#include <stdint.h>
#include <stddef.h>

#if (_ENCODER_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define as50_delay(x)   osDelay(x)
#else
#define as50_delay(x)   HAL_Delay(x)
#endif

uint16_t read_angle_register = 0x3FFF; //angle read register
uint16_t set_zero_register_M = 0x0016; //angle set zero register MOST
uint16_t set_zero_register_L = 0x0017; //angle set zero register LEAST
uint16_t spiR;

/*
uint16_t parity(uint16_t x){};
bool as50_readAngle(uint16_t * data, uint32_t timeout){};
bool as50_setZero(uint32_t timeout){};
bool as50_write(uint16_t address, uint16_t data){};
*/

uint16_t parity(uint16_t x)
{
	uint16_t parity = 0;

	while(x != 0)
	{
		parity ^= x;
		x >>= 1;
	}

	return (parity & 0x1);
}

void as50_readAngle(uint16_t * data, uint32_t timeout)
{

	  HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_RESET);
	  if (HAL_SPI_TransmitReceive(&_ENCODER_SPI, (uint8_t*)&read_angle_register, (uint8_t*)data, 1, timeout) == HAL_OK)
	  {
		  HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_SET);
		  *data &= _ENCODER_READMASK;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_SET);
	  }
}


void as50_setZero(uint32_t timeout)
{
	  UNUSED(timeout);
	  uint16_t angle_to_set_as_zero;
	  as50_readAngle(&angle_to_set_as_zero, 100);
	  as50_write(set_zero_register_M, ((angle_to_set_as_zero >> 6) & 0x00FF));
	  as50_write(set_zero_register_L, (angle_to_set_as_zero  & 0x003F));
}


void as50_write(uint16_t address, uint16_t data)
{
	if (parity(address & 0x3FFF) == 1) address = address | 0x8000; // set parity bit

	HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_RESET);

	as50_delay(1);

	if (HAL_SPI_Transmit(&_ENCODER_SPI, (uint8_t*) &address, 2, 100) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_SET);

	as50_delay(1);

	if (parity(data & 0x3FFF) == 1) data = data | 0x8000; // set parity bit

	HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_RESET);

	as50_delay(1);

	if (HAL_SPI_Transmit(&_ENCODER_SPI, (uint8_t*) &data, 2, 100) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_SET);
}

