/*
 * AS50xx.c
 *
 *  Created on: Jul 31, 2023
 *      Author: VR
 */

#include "as50xx.h"

#include <stdint.h>
#include <stddef.h>
#include <limits.h>

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

static as50_diagnostics_t diagnostics = {
	.raw_frame = 0U,
	.transfer_count = 0U,
	.error_count = 0U,
	.last_hal_status = HAL_ERROR,
	.last_read_ok = false,
	.has_valid_angle = false,
};

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

bool as50_readAngle(uint16_t * data, uint32_t timeout)
{
	  uint16_t raw_frame = 0U;

	  if (data == NULL)
	  {
		  return false;
	  }

	  if (diagnostics.transfer_count < UINT32_MAX)
	  {
		  ++diagnostics.transfer_count;
	  }

	  /*
	   * This single-frame exchange is intentionally kept as proven on the
	   * physical T-encoder. AS50xx replies are pipelined; do not change the
	   * command sequence without verifying it on the hardware bench.
	   */
	  HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_RESET);
	  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
		  &_ENCODER_SPI,
		  (uint8_t*)&read_angle_register,
		  (uint8_t*)&raw_frame,
		  1,
		  timeout);
	  HAL_GPIO_WritePin(_ENCODER_NSS_GPIO, _ENCODER_NSS_PIN, GPIO_PIN_SET);

	  diagnostics.last_hal_status = status;
	  diagnostics.last_read_ok = (status == HAL_OK);
	  if (status != HAL_OK)
	  {
		  if (diagnostics.error_count < UINT32_MAX)
		  {
			  ++diagnostics.error_count;
		  }
		  return false;
	  }

	  diagnostics.raw_frame = raw_frame;
	  diagnostics.has_valid_angle = true;
	  *data = raw_frame & _ENCODER_READMASK;
	  return true;
}

void as50_getDiagnostics(as50_diagnostics_t * diagnostics_out)
{
	if (diagnostics_out != NULL)
	{
		*diagnostics_out = diagnostics;
	}
}

void as50_setZero(uint32_t timeout)
{
	  uint16_t angle_to_set_as_zero = 0U;
	  if (!as50_readAngle(&angle_to_set_as_zero, timeout))
	  {
		  return;
	  }
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

