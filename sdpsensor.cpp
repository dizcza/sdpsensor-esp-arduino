/*
 * sdpsensor.c
 *
 *  Created on: Sep 7, 2021
 *      Author: Danylo Ulianych
 */

#include <HardwareSerial.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sdpsensor.h"

#define SPD31_500_PID  0x03010100
#define SDP32_125_PID  0x03010200
#define SDP800_500_PID 0x03020100
#define SDP810_500_PID 0x03020A00
#define SDP801_500_PID 0x03020400
#define SDP811_500_PID 0x03020D00
#define SDP800_125_PID 0x03020200
#define SDP810_125_PID 0x03020B00

#define SDPSENSOR_I2C_CMD_LEN       2
#define SDPSENSOR_TEMPERATURE_SCALE (200.0f)
#define I2C_NO_TIMEOUT              0  // non-blocking

const char *TAG_SDPSENSOR = "sdpsensor";

/*
 * See http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
 */
uint8_t SDPSensor::computeCRC(uint8_t* data)
{
    const uint8_t generator = 0x31;  /* Polynomial */
    uint8_t crc = 0xFF; /* Initial value from the datasheet */

    for (int i = 0; i < 2; i++) {  /* two-bytes input data */
        crc ^= data[i];  /* XOR-in the next input byte */
        for (int j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t) ((crc << 1) ^ generator);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


SDPSensor::SDPSensor(uint8_t i2c_addr, i2c_port_t i2c_port) {
    this->i2c_addr = i2c_addr;
    this->i2c_port = i2c_port;
    this->pressureScale = 0;  // will be set in the init()
    this->maxSuccessiveFailsCount = 2;
    this->failsCount = 0;
}


void SDPSensor::initI2C(int pinSDA, int pinSCL) {
  int intr_flag_disable = 0;

  /* I2C master doesn't need buffer */
  size_t i2c_master_rx_buf_disable = 0;
  size_t i2c_master_tx_buf_disable = 0;

  i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pinSDA,
        .scl_io_num = pinSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
  conf.master.clk_speed = 400000;  /*!< I2C master clock frequency */

  ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));

	ESP_ERROR_CHECK(
			i2c_driver_install(i2c_port, conf.mode,
					i2c_master_rx_buf_disable, i2c_master_tx_buf_disable,
					intr_flag_disable));
	ESP_LOGI(TAG_SDPSENSOR, "I2C%d line initialized", i2c_port);
}


void SDPSensor::initSensor() {
	SDPSensor::reset();  // stop continuous mode

	// commands to read product id
	uint8_t cmd0[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x7C };
	uint8_t cmd1[SDPSENSOR_I2C_CMD_LEN] = { 0xE1, 0x02 };

	// command to trigger a one-time measurement
	uint8_t cmd_measure[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x2F };

	uint8_t read_buffer[18] = { 0 };

	const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
	ESP_ERROR_CHECK(
			i2c_master_write_to_device(i2c_port, i2c_addr, cmd0, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long));
	ESP_ERROR_CHECK(
			i2c_master_write_to_device(i2c_port, i2c_addr, cmd1, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long));

	/*
	 Read product id and serial number.
	 Data Format:
	 | Byte  | 0 | 1 | 2 | 3 | 4 | 5 | 6...18 |
	 | Value |  pid1 |CRC|  pid2 |CRC| serial |
	 */
	ESP_ERROR_CHECK(
			i2c_master_read_from_device(i2c_port, i2c_addr, read_buffer, 18,
					ticks_to_wait_long));

	const uint32_t pid = (read_buffer[0] << 24) | (read_buffer[1] << 16)
			| (read_buffer[3] << 8) | (read_buffer[4] << 0);

    uint32_t model_number, range_pa;
	switch (pid & 0xFFFFFF00) {
	case SPD31_500_PID:
		model_number = 31;
		range_pa = 500;
		break;
	case SDP32_125_PID:
		model_number = 32;
		range_pa = 125;
		break;
	case SDP800_500_PID:
		model_number = 800;
		range_pa = 500;
		break;
	case SDP810_500_PID:
		model_number = 810;
		range_pa = 500;
		break;
	case SDP801_500_PID:
		model_number = 801;
		range_pa = 500;
		break;
	case SDP811_500_PID:
		model_number = 811;
		range_pa = 500;
		break;
	case SDP800_125_PID:
		model_number = 800;
		range_pa = 125;
		break;
	case SDP810_125_PID:
		model_number = 810;
		range_pa = 125;
		break;
	}

	ESP_LOGI(TAG_SDPSENSOR, "Initialized SDP%d %dPa sensor (PID=0x%08X)", model_number, range_pa, pid);

	ESP_ERROR_CHECK(
			i2c_master_write_to_device(i2c_port, i2c_addr, cmd_measure, SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long));

	vTaskDelay(pdMS_TO_TICKS(90));  // theoretically 45 ms

	ESP_ERROR_CHECK(
			i2c_master_read_from_device(i2c_port, i2c_addr, read_buffer, 9, ticks_to_wait_long));

	this->pressureScale = ((int16_t) read_buffer[6]) << 8 | read_buffer[7];

	ESP_LOGI(TAG_SDPSENSOR, "SDP%d pressure scale: %d", model_number, this->pressureScale);
}

uint16_t SDPSensor::getPressureScale() {
	return pressureScale;
}

esp_err_t SDPSensor::startContinuous() {
	uint8_t cmd[SDPSENSOR_I2C_CMD_LEN] = { 0x36, 0x1E };
	const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
	esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd,
			SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
	ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::startContinuous %s",
			esp_err_to_name(err));
	// wait for sensor to start continuously making measurements
	vTaskDelay(pdMS_TO_TICKS(20));
	return err;
}

esp_err_t SDPSensor::stopContinuous() {
	uint8_t cmd[SDPSENSOR_I2C_CMD_LEN] = { 0x3F, 0xF9 };
	const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
	esp_err_t err = i2c_master_write_to_device(i2c_port, i2c_addr, cmd,
			SDPSENSOR_I2C_CMD_LEN, ticks_to_wait_long);
	ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::stopContinuous %s",
			esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(20));
	return err;
}

esp_err_t SDPSensor::reset() {
	uint8_t cmd[1] = { 0x06 };
	const TickType_t ticks_to_wait_long = pdMS_TO_TICKS(100);
	esp_err_t err = i2c_master_write_to_device(i2c_port, 0x00, cmd,
			1, ticks_to_wait_long);
	ESP_LOGI(TAG_SDPSENSOR, "SDPSensor::reset %s", esp_err_to_name(err));
	vTaskDelay(pdMS_TO_TICKS(20));
	return err;
}

void SDPSensor::watchdogSetParams(uint32_t maxSuccessiveFailsCount) {
    this->maxSuccessiveFailsCount = maxSuccessiveFailsCount;
}


void SDPSensor::watchdogCheck(esp_err_t status) {
	if (status == ESP_OK || status == ESP_ERR_INVALID_CRC) {
		// invalid CRC does not mean that the sensor is not functioning
		failsCount = 0;
	} else {
		failsCount++;
	}

	if (failsCount >= maxSuccessiveFailsCount) {
		ESP_LOGW(TAG_SDPSENSOR, "SDPWatchdog barked. Resetting...");
		// flush i2c buffers before resetting
		i2c_reset_tx_fifo(i2c_port);
		i2c_reset_rx_fifo(i2c_port);

		esp_err_t err = ESP_FAIL;
		do {
			err = SDPSensor::reset();
			if (err == ESP_OK) {
				err = SDPSensor::startContinuous();
			}
		} while (err != ESP_OK);
	}
}


esp_err_t SDPSensor::readContinuousRaw(int16_t *diffPressureRaw) {
	uint8_t data[3] = { 0 };
	esp_err_t err = i2c_master_read_from_device(i2c_port, i2c_addr, data, 3,
			I2C_NO_TIMEOUT);
	if (err == ESP_OK) {
		*diffPressureRaw = ((int16_t) data[0]) << 8 | data[1];
		if (data[2] != computeCRC(data)) {
			err = ESP_ERR_INVALID_CRC;
		}
	}
	return err;
}


esp_err_t SDPSensor::readContinuousRawTemperature(int16_t *diffPressureRaw, float *temperature) {
	uint8_t data[6] = { 0 };

	/*
	 Data Format:
	 | Byte  |  0  |  1  |  2  |  3  |  4  |  5  |
	 | Value | pressure  | CRC |temperature| CRC |
	 */
	esp_err_t err = i2c_master_read_from_device(i2c_port, i2c_addr, data, 6, I2C_NO_TIMEOUT);

	if (err == ESP_OK) {
		if (diffPressureRaw != NULL) {
			*diffPressureRaw = ((int16_t) data[0]) << 8 | data[1];
		}
		if (temperature != NULL) {
			int16_t temp_raw = ((int16_t) data[3]) << 8 | data[4];
			*temperature = temp_raw / SDPSENSOR_TEMPERATURE_SCALE;
		}
		if (( data[2] != computeCRC(data) ) || ( data[5] != computeCRC(&data[3]) )) {
			err = ESP_ERR_INVALID_CRC;
		}
	}

	return err;
}
