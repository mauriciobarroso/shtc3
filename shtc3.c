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

#ifdef ESP32_TARGET
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#endif /* ESP32_TARGET */

/* Private macros ------------------------------------------------------------*/
#define NOP()			asm volatile ("nop")
#define CRC8_POLYNOMIAL	0x31
#define CRC8_INIT 		0xFF
#define CRC8_LEN 		1

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "shtc3";

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 *
 * @param data : Pointer to the data to be read from addr
 * @param data_len : Length of the data transfer
 * @param intf     : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t shtc3_reg_read(uint8_t *data, uint32_t data_len, void *intf);

/**
 * @brief Function that implements the default I2C write transaction
 *
 * @param data : Data (command) to be written to device
 * @param intf : Pointer to the interface descriptor
 *
 * @return 0 if successful, non-zero otherwise
 */
static int8_t shtc3_reg_write(uint16_t data, void *intf);

/**
 * @brief Function that implements a mili seconds delay
 *
 * @param time_ms: Time in us to delay
 */
static void delay_ms(uint32_t time_ms);

/**
 * @brief Function that generates a CRC byte for a given data
 *
 * @param data  :
 * @param count :
 *
 * @return CRC byte
 */
static uint8_t generate_crc(const uint8_t *data, uint16_t count);

/**
 * @brief Function that checks the CRC for the received data
 *
 * @param data     :
 * @param count    :
 * @param checksum :
 *
 * @return False on failure or True on success
 */
static bool check_crc(const uint8_t *data, uint16_t count, uint8_t checksum);

static float calc_temp(uint16_t raw_temp);
static float calc_hum(uint16_t raw_hum);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a SHTC3 instance
 */
int shtc3_init(shtc3_t *const me, void *i2c_handle, uint8_t dev_addr)
{
	/* Variable to return error code */
	int ret = 0;

#ifdef ESP32_TARGET
	/* Add device to I2C bus */
	i2c_device_config_t i2c_dev_conf = {
			.scl_speed_hz = 400000,
			.device_address = dev_addr
	};

	if (i2c_master_bus_add_device((i2c_master_bus_handle_t)i2c_handle,
			&i2c_dev_conf, &me->i2c_dev) != 0) {
		ESP_LOGE(TAG, "Failed to add device to I2C bus");
		return ret;
	}
#else
	me->i2c_dev.i2c_handle = (I2C_HandleTypeDef *)i2c_handle;
	me->i2c_dev.dev_addr = SHTC3_I2C_ADDR;
#endif /* ESP32_TARGET */

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to get the device ID
 */
int shtc3_get_id(shtc3_t *const me, uint16_t *id)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_READ_ID, me->i2c_dev);

	uint8_t data[3] = {0};
	shtc3_reg_read(data, 3, me->i2c_dev);

	/* Check data received CRC */
	if (!check_crc(data, 2, data[2])) {
		return -1;
	}

	*id = data[0] << 8 | data[1];

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to get the temperature (°C) and humidity (%)
 */
int shtc3_get_temp_and_hum(shtc3_t *const me, float *temp, float *hum)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_wakeup(me);

	shtc3_reg_write(SHTC3_CMD_MEAS_T_RH_CLOCKSTR_NM, me->i2c_dev);

	delay_ms(300);

	uint8_t data[6] = {0};
	shtc3_reg_read(data, 6, me->i2c_dev);

	/* Check data received CRC */
	if (!check_crc(&data[0], 2, data[2])) {
		return -1;
	}

	if (!check_crc(&data[3], 2, data[5])) {
		return -1;
	}

	*temp = calc_temp((uint16_t)((data[0] << 8) | (data[1])));
	*hum = calc_hum((uint16_t)((data[3] << 8) | (data[4])));

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to get the temperature (°C) and humidity (%). This function
 *        polls every 1 ms until measumente is ready
 */
int shtc3_get_temp_and_hum_polling(shtc3_t *const me, float *temp, float *hum)
{
	/* Variable to return error code */
	int ret = 0;

	/* todo: write */

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to put the device in sleep mode
 */
int shtc3_sleep(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_SLEEP, me->i2c_dev);

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to wakeup the device from sleep mode
 */
int shtc3_wakeup(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_WAKEUP, me->i2c_dev);

	delay_ms(1);

	/* Return 0 */
	return ret;
}

/**
 * @brief Function to perfrom a software reset of the device
 */
int shtc3_soft_reset(shtc3_t *const me)
{
	/* Variable to return error code */
	int ret = 0;

	shtc3_reg_write(SHTC3_CMD_SOFT_RESET, me->i2c_dev);

	/* Return 0 */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 */
static int8_t shtc3_reg_read(uint8_t *data, uint32_t data_len, void *intf)
{
#ifdef ESP32_TARGET
	i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf;

	if (i2c_master_receive(i2c_dev, data, data_len, -1)
			!= 0) {
		return -1;
	}
#else
	i2c_stm32_dev_t *i2c_dev = (i2c_stm32_dev_t *)intf;

	if (HAL_I2C_Master_Receive(i2c_dev->i2c_handle,
			(i2c_dev->dev_addr << 1) | 0x01, data, data_len, 100) > 0) {
		return -1;
	}
#endif /* ESP32_TARGET */

	return 0;
}

/**
 * @brief Function that implements the default I2C write transaction
 */
static int8_t shtc3_reg_write(uint16_t data, void *intf)
{
	uint8_t buffer[SHTC3_I2C_BUFFER_LEN_MAX] = {
			(uint8_t)((data >> 8) & 0xFF),
			(uint8_t)(data & 0xFF)
	};

	/* Transmit buffer */
#ifdef ESP32_TARGET
	i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)intf;

	if (i2c_master_transmit(i2c_dev, buffer, 2, -1)
			!= 0) {
		return -1;
	}
#else
	i2c_stm32_dev_t *i2c_dev = (i2c_stm32_dev_t *)intf;

	if (HAL_I2C_Master_Transmit(i2c_dev->i2c_handle, i2c_dev->dev_addr << 1,
			buffer, 2, 100)) {
		return -1;
	}
#endif /* ESP32_TARGET */
	return 0;
}

/**
 * @brief Function that implements a mili seconds delay
 */
static void delay_ms(uint32_t time_ms)
{
#ifdef ESP32_TARGET
	uint64_t m = (uint64_t)esp_timer_get_time();

	uint32_t period_us = time_ms * 1000;
	if (period_us) {
		uint64_t e = (m + period_us);

		if (m > e) { /* overflow */
			while ((uint64_t)esp_timer_get_time() > e) {
				NOP();
			}
		}

		while ((uint64_t)esp_timer_get_time() < e) {
			NOP();
		}
	}
#else
  HAL_Delay(time_ms);
#endif /* ESP32_TARGET */
}

/**
 * @brief Function that generates a CRC byte for a given data
 */
static uint8_t generate_crc(const uint8_t *data, uint16_t count) {
  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  /* calculates 8-Bit checksum with given polynomial */
  for (current_byte = 0; current_byte < count; ++current_byte) {
  	crc ^= (data[current_byte]);

  	for (crc_bit = 8; crc_bit > 0; --crc_bit) {
  		if (crc & 0x80) {
  			crc = (crc << 1) ^ CRC8_POLYNOMIAL;
  		}
  		else {
  			crc = (crc << 1);
  		}
  	}
  }
  return crc;
}

/**
 * @brief Function that checks the CRC for the received data
 */
static bool check_crc(const uint8_t *data, uint16_t count, uint8_t checksum) {
	if (generate_crc(data, count) != checksum) {
		return false;
	}

	return true;
}

static float calc_temp(uint16_t raw_temp)
{
	return 175 * (float)raw_temp / 65536.0f - 45.0f;
}

static float calc_hum(uint16_t raw_hum)
{
	return 100 * (float)raw_hum / 65536.0f;
}
/***************************** END OF FILE ************************************/
