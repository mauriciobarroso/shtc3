/**
  ******************************************************************************
  * @file           : shtc3.h
  * @author         : Mauricio Barroso Benavides
  * @date           : Jul 18, 2023
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHTC3_H_
#define SHTC3_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
#define ESP32_TARGET
#endif

#ifdef ESP32_TARGET
#include "driver/i2c_master.h"
#else
#include "main.h"
#endif /* ESP32_TARGET */

/* Exported Macros -----------------------------------------------------------*/
#define SHTC3_I2C_ADDR						0x70
#define SHTC3_I2C_BUFFER_LEN_MAX			8

#define SHTC3_CMD_READ_ID					0xEFC8 /* command: read ID register */
#define SHTC3_CMD_SOFT_RESET				0x805D /* soft reset */
#define SHTC3_CMD_SLEEP						0xB098 /* sleep */
#define SHTC3_CMD_WAKEUP					0x3517 /* wakeup */

#define SHTC3_CMD_MEAS_T_RH_POLLING_NM		0x7866 /* meas. read T first, clock stretching disabled in normal mode */
#define SHTC3_CMD_MEAS_T_RH_CLOCKSTR_NM		0x7CA2 /* meas. read T first, clock stretching enabled in normal mode */
#define SHTC3_CMD_MEAS_RH_T_POLLING_NM		0x58E0 /* meas. read RH first, clock stretching disabled in normal mode */
#define SHTC3_CMD_MEAS_RH_T_CLOCKSTR_NM		0x5C24 /* meas. read RH first, clock stretching enabled in normal mode */
#define SHTC3_CMD_MEAS_T_RH_POLLING_LPM		0x609C /* meas. read T first, clock stretching disabled in low power mode */
#define SHTC3_CMD_MEAS_T_RH_CLOCKSTR_LPM	0x6458 /* meas. read T first, clock stretching enabled in low power mode */
#define SHTC3_CMD_MEAS_RH_T_POLLING_LPM		0x401A /* meas. read RH first, clock stretching disabled in low power mode */
#define SHTC3_CMD_MEAS_RH_T_CLOCKSTR_LPM	0x44DE /* meas. read RH first, clock stretching enabled in low power mode */

/* Exported typedef ----------------------------------------------------------*/
typedef struct {
#ifdef ESP32_TARGET
	i2c_master_dev_handle_t handle;
#else
	uint8_t addr;
	I2C_HandleTypeDef *handle;
#endif /* ESP32_TARGET */
} shtc3_i2c_t;

typedef struct {
	shtc3_i2c_t i2c_dev;
} shtc3_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 *
 * @param me         : Pointer to a shtc3_t instance
 * @param i2c_handle : Pointer to a structure with the data to initialize the
 * 					   I2C device
 * @param dev_addr   : I2C device address
 *
 * @return ESP_OK on success
 */
int shtc3_init(shtc3_t *const me, void *i2c_handle, uint8_t dev_addr);

/**
 * @brief Function to get the device ID
 *
 * @param me : Pointer to a shtc3_t instance
 * @param id : Pointer to SHTC3 ID
 *
 * @return ESP_OK on success
 */
int shtc3_get_id(shtc3_t *const me, uint16_t *id);

/**
 * @brief Function to get the temperature (°C) and humidity (%)
 *
 * @param me   : Pointer to a shtc3_t instance
 * @param temp : Pointer to floating point value, where the calculated
 *               temperature value will be stored
 * @param hum  : Pointer to floating point value, where the calculated
 *               humidity value will be stored
 *
 * @return ESP_OK on success
 */
int shtc3_get_temp_and_hum(shtc3_t *const me, float *temp, float *hum);

/**
 * @brief Function to get the temperature (°C) and humidity (%). This function
 *        polls every 1 ms until measumente is ready
 *
 * @param me   : Pointer to a shtc3_t instance
 * @param temp : Pointer to floating point value, where the calculated
 *               temperature value will be stored
 * @param hum  : Pointer to floating point value, where the calculated
 *               humidity value will be stored
 *
 * @return ESP_OK on success
 */
int shtc3_get_temp_and_hum_polling(shtc3_t *const me, float *temp, float *hum);

/**
 * @brief Function to put the device in sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_sleep(shtc3_t *const me);

/**
 * @brief Function to wakeup the device from sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_wakeup(shtc3_t *const me);

/**
 * @brief Function to perfrom a software reset of the device
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
int shtc3_soft_reset(shtc3_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* SHTC3_H_ */

/***************************** END OF FILE ************************************/
