/*
 * at24_eeprom.h
 *
 *  Created on: Jul 28, 2023
 *      Author: VR
 */

#ifndef INC_AT24_EEPROM_H_
#define INC_AT24_EEPROM_H_


/********************************* Configuration Start *********************************/
#define		_EEPROM_SIZE_KBIT		64       /* 64K (8 192 x 8) */
#define		_EEPROM_I2C			hi2c4
#define		_EEPROM_USE_FREERTOS		1  /* FreeRTOS by default */
#define		_EEPROM_ADDRESS			0xA0
#define		_EEPROM_USE_WP_PIN		0
#define		_EEPROM_USE_IWDG		0

/* The AT24CXX has a hardware data protection scheme that allows the
user to write protect the whole memory when the WP pin is at VCC. */

#if (_EEPROM_USE_WP_PIN == 1)
#define		_EEPROM_WP_GPIO			WP_GPIO_Port
#define		_EEPROM_WP_PIN			WP_Pin
#endif

/* For watchdog timer refresh, required during erase operation */
#if (_EEPROM_USE_IWDG == 1)
#define		_EEPROM_IWDG			hiwdg
#endif

/********************************* Configuration End *********************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/**
  * @brief  Checks if memory device is ready for communication.
  * @param  none
  * @retval bool
  */
int at24_isConnected(void);

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration (optional)
  * @retval bool status
  */
int at24_write(uint16_t, uint8_t *, uint8_t , uint32_t);

/**
  * @brief  Read an amount of data in blocking mode to a specific memory address
  * @param  address Internal memory address
  * @param  data Pointer to data buffer
  * @param  len Amount of data to be sent
  * @param  timeout Timeout duration (optional)
  * @retval bool status
  */
int at24_read(uint16_t , uint8_t *, uint8_t , uint32_t);

/**
  * @brief  Erase memory.
  * @note   This requires time in seconds
  * @param  none
  * @retval bool status
  */
int at24_eraseChip(void);

#ifdef __cplusplus
}
#endif

#endif

/* EOF */
