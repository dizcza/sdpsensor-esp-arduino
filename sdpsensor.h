/*
 * sdpsensor.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Danylo Ulianych
 */

#ifndef SDPSENSOR_H_
#define SDPSENSOR_H_

#include "driver/i2c.h"


class SDPSensor {
    private:
        uint8_t i2c_addr;                    /* I2C address */
        i2c_port_t i2c_port;                 /* I2C master port */
        uint16_t pressureScale;              /* Diff pressure scale */
        uint32_t failsCount;                 /* SDP successive failed reads count */
        uint32_t maxSuccessiveFailsCount;    /* SDP max successive failed reads before SW reset */
        uint8_t computeCRC(uint8_t *data);   /* Compute CRC from data */
    public:

        /**
         * Constructor
         *
         * @param i2c_addr - I2C address.
         * @param i2c_port - I2C port.
         *                   ESP boards have two I2C peripherals.
         *                   Defaults to 0.
         */
        SDPSensor(uint8_t i2c_addr, i2c_port_t i2c_port = 0);


        /**
         * Initialize I2C. Same as `Wire.begin(SDA, SCL)`.
         *
         * @param pinSDA - SDA GPIO
         * @param pinSCL - SCL GPIO
         */
        void initI2C(int pinSDA, int pinSCL);


        /**
         * Initialize SDP sensor.
         *
         * Firstly, it resets the sensor prior to executing any I2C commands.
         * Then it reads and saves the sensor serial number and diff pressure scale.
         */
        void initSensor();


        /**
         * Return the diff pressure scale, saved in the `initSensor()` call.
         *
         * @returns diff pressure scale
         */
        uint16_t getPressureScale();


        /**
         * Start the sensor in the continuous mode.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t startContinuous();


        /**
         * Stop the sensor continuous mode.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t stopContinuous();


        /**
         * Reset the SDP sensor.
         *
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t reset();


        /**
         * Set watchdog params to be used in the `watchdogCheck()` function.
         * Has no effect if the user does not call the `watchdogCheck()` function.
         *
         * @param maxSuccessiveFailsCount - the maximum successive failed reads
         *                                  until the reset command is issued.
         */
        void watchdogSetParams(uint32_t maxSuccessiveFailsCount);


        /**
         * Check the error code status of the last read measurement.
         * If enough errors occurred, a software reset is issued.
         *
         * Usage:
         *   esp_err_t err = sdp.readContinuousRaw(&diff_pressure);
         *   sdp.watchdogCheck(err);
         *
         * @param status - the error code of the last measurement
         */
        void watchdogCheck(esp_err_t status);


        /**
         * Read the raw differential pressure value and save the result
         * in `diffPressureRaw`. To convert it to the real value in Pa,
         * one should divide it by the pressure scale (see
         * `getPressureScale()` function).
         *
         * This call is non-blocking (zero I2C timeout).
         *
         * @param diffPressureRaw - a pointer to save the result
         * @returns the error code (defined in esp_err.h):
         *    ESP_OK               - success
         *    ESP_FAIL             - failure
         *    ESP_ERR_TIMEOUT      - timed out
         *    ESP_ERR_INVALID_CRC  - CRC mismatch
         */
        esp_err_t readContinuousRaw(int16_t *diffPressureRaw);


        /**
         * Read the raw differential pressure value AND the temperature.
         * 
         * @param diffPressureRaw - a pointer to save the diff pressure
         * @param temperature - a pointer to save the temperature in Celsius
         * @returns the error code (defined in esp_err.h)
         */
        esp_err_t readContinuousRawTemperature(int16_t *diffPressureRaw, float *temperature);
};

#endif /* SDPSENSOR_H_ */