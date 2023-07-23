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

#include "i2c_bus.h"

/* Exported Macros -----------------------------------------------------------*/
#define SHTC3_ADDR								0x70

#define SHTC3_READ_ID							0xEFC8 /* command: read ID register */
#define SHTC3_SOFT_RESET					0x805D /* soft reset */
#define SHTC3_SLEEP								0xB098 /* sleep */
#define SHTC3_WAKEUP							0x3517 /* wakeup */
#define SHTC3_MEAS_T_RH_POLLING		0x7866 /* meas. read T first, clock stretching disabled */
#define SHTC3_MEAS_T_RH_CLOCKSTR	0x7CA2 /* meas. read T first, clock stretching enabled */
#define SHTC3_MEAS_RH_T_POLLING		0x58E0 /* meas. read RH first, clock stretching disabled */
#define SHTC3_MEAS_RH_T_CLOCKSTR	0x5C24 /* meas. read RH first, clock stretching enabled */

#define SHTC3_CRC_POLYNOMIAL			0x131 /* P(x) = x^8 + x^5 + x^4 + 1 = 100110001 */
/* Exported typedef ----------------------------------------------------------*/
typedef struct {
	i2c_bus_dev_t *i2c_dev;
	uint16_t id;
} shtc3_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 *
 * @param me       : Pointer to a shtc3_t instance
 * @param i2c_bus  : Pointer to a structure with the data to initialize the
 * 								   I2C device
 * @param dev_addr : I2C device address
 * @param read     : Pointer to I2C read function
 * @param write    : Pointer to I2C write function
 *
 * @return ESP_OK on success
 */
esp_err_t shtc3_init(shtc3_t *const me, i2c_bus_t *i2c_bus, uint8_t dev_addr,
		i2c_bus_read_t read, i2c_bus_write_t write);

/**
 * @brief Function to get the device ID
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t shtc3_get_id(shtc3_t *const me);

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
esp_err_t shtc3_get_temp_and_hum(shtc3_t *const me, float *temp, float *hum);

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
esp_err_t shtc3_get_temp_and_hum_polling(shtc3_t *const me, float *temp, float *hum);

/**
 * @brief Function to put the device in sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t shtc3_sleep(shtc3_t *const me);

/**
 * @brief Function to wakeup the device from sleep mode
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t shtc3_wakeup(shtc3_t *const me);

/**
 * @brief Function to perfrom a software reset of the device
 *
 * @param me : Pointer to a shtc3_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t shtc3_soft_reset(shtc3_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* SHTC3_H_ */

/***************************** END OF FILE ************************************/
