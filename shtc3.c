/**
  ******************************************************************************
  * @file           : shtc3.c
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

/* Includes ------------------------------------------------------------------*/
#include "shtc3.h"
#include "esp_err.h"
#include "esp_log.h"

/* Private macros ------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "shtc3";

/* Private function prototypes -----------------------------------------------*/
static uint8_t *get_buffer(uint16_t data);
static int8_t i2c_read(uint16_t reg_addr, uint8_t *reg_data,
		uint32_t data_len, void *intf);
static int8_t i2c_write(uint16_t reg_addr, const uint8_t *reg_data,
		uint32_t data_len, void *intf);
static bool check_crc(uint8_t data[], uint8_t, uint8_t bytes_num, uint8_t checksum);
static float calc_temp(uint16_t raw_temp);
static float calc_hum(uint16_t raw_hum);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 */
esp_err_t shtc3_init(shtc3_t *const me, i2c_bus_t *i2c_bus, uint8_t dev_addr,
		i2c_bus_read_t read, i2c_bus_write_t write) {
	/* Print initializing message */
	ESP_LOGI(TAG, "Initializing SHTC3 instance...");

	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Add device to bus */
	ret = i2c_bus_add_dev(i2c_bus, dev_addr, "shtc3", NULL, NULL);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add device");
		return ret;
	}

	/**/
	me->i2c_dev = &i2c_bus->devs.dev[i2c_bus->devs.num - 1]; /* todo: write function to get the dev from name */

	/* Print successful initialization message */
	ESP_LOGI(TAG, "SHTC3 instance initialized successfully");

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to get the device ID
 */
esp_err_t shtc3_get_id(shtc3_t *const me) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to get the temperature (°C) and humidity (%)
 */
esp_err_t shtc3_get_temp_and_hum(shtc3_t *const me, float *temp, float *hum) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to get the temperature (°C) and humidity (%). This function
 *        polls every 1 ms until measumente is ready
 */
esp_err_t shtc3_get_temp_and_hum_polling(shtc3_t *const me, float *temp, float *hum) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to put the device in sleep mode
 */
esp_err_t shtc3_sleep(shtc3_t *const me) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to wakeup the device from sleep mode
 */
esp_err_t shtc3_wakeup(shtc3_t *const me) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perfrom a software reset of the device
 */
esp_err_t shtc3_soft_reset(shtc3_t *const me) {
	/* Variable to return */
	esp_err_t ret = ESP_OK;

	/* Return ESP_OK */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
static bool check_crc(uint8_t data[], uint8_t, uint8_t bytes_num, uint8_t checksum) {
	uint8_t crc = 0xFF;

	/* Calculates 8-bit checksum with given polynomial */
	for (uint8_t i = 0; i < bytes_num; i++) {
		crc ^= data[i];

		for (uint8_t j = 8; j > 0; --j) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ SHTC3_CRC_POLYNOMIAL;
			}
			else {
				crc <<= 1;
			}
		}
	}

	/* Return false if the calculated checksum is different of the given checksum */
	if (crc != checksum) {
		return false;
	}

	/* Return true for no errors */
	return true;
}

static float calc_temp(uint16_t raw_temp) {
	return 175 * (float)raw_temp / 65536.0f - 45.0f; /* T = -45 + 175 * raw_temp / 2^16 */
}

static float calc_hum(uint16_t raw_hum) {
  return 100 * (float)raw_hum / 65536.0f; /* RH = raw_hum / 2^16 * 100 */
}

static int8_t i2c_read(uint16_t reg_addr, uint8_t *reg_data,
		uint32_t data_len, void *intf) {

	uint8_t reg[2] = {(reg_addr & 0xFF00 ) >> 8, reg_addr & 0x00FF};

	i2c_bus_dev_t *dev = (i2c_bus_dev_t *)intf;

	return dev->read(reg, 2, reg_data, data_len, intf);
}

static int8_t i2c_write(uint16_t reg_addr, const uint8_t *reg_data,
		uint32_t data_len, void *intf) {
	i2c_bus_dev_t *dev = (i2c_bus_dev_t *)intf;

	uint8_t reg[2] = {(uint8_t)((reg_addr & 0xFF00 ) >> 8),
			(uint8_t)(reg_addr & 0x00FF)};

	return dev->write(reg, 2, reg_data, data_len, intf);
}

/***************************** END OF FILE ************************************/
