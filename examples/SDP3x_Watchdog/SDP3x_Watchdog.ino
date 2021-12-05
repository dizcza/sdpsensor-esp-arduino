/**
 * An example to use an SDP3x sensor with an ESP32 board
 *   with a watchdog that barks each time three errors are
 *   encountered in a row.
 * 
 * Software setup:
 *   1) install the arduino-esp library
 *   2) select your ESP board in "Tools" -> "Board"
 *   3) select the right port in "Tools" -> "Port"
 *   4) turn on the logs by setting "Tools" -> "Core Debug Level" -> "Info"
 * Hardware setup:
 *   1) connect GPIO SDA to pin 19
 *   2) connect GPIO SCL to pin 23
 * or change the code appropriately.
 */

#include <Wire.h>
#include "sdpsensor.h"


/**
 * An SDP Sensor instance.
 * If you don't know the I2C address of a sensor,
 *   scan all addresses with an I2C scanner first.
 */
SDPSensor sdp(0x21);

/* Failed reads count */
uint32_t failsCount = 0;

/* Max SDP successive failed reads before a SW reset */
const uint32_t maxSuccessiveFailsCount = 1;

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
bool watchdogCheck(esp_err_t status) {
    if (status == ESP_OK) {
        failsCount = 0;
    } else {
        failsCount++;
    }

    if (failsCount >= maxSuccessiveFailsCount) {
        Serial.println("SDPWatchdog barked. Resetting...");
        esp_err_t err = ESP_FAIL;
        do {
            err = sdp.reset();  // sdp.stopContinuous() is a safer version
            if (err == ESP_OK) {
                err = sdp.startContinuous();
            }
        } while (err != ESP_OK);
        return true;
    }

    return false;
}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  Wire.begin(19, 23);

  // try until succeeds
  while (sdp.stopContinuous() != ESP_OK);  // sdp.reset() is also possible
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);
}


void loop() {
  int16_t pressure;
  esp_err_t err;

  do {
    // read the sensor without delay until the watchdog starts barking ...
    err = sdp.readDiffPressure(&pressure);
  } while (!watchdogCheck(err));

  // wait & repeat
  delay(3000);
}
