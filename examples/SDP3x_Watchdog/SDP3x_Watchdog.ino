/**
 * An example to use an SDP3x sensor with an ESP32 board
 *   with a watchdog that barks each time three errors are
 *   encountered in a row.
 * 
 * Prior setup:
 *   1) install the arduino-esp library
 *   2) select your ESP board in "Tools" -> "Board"
 *   3) select the right port in "Tools" -> "Port"
 *   4) turn on the logs by setting "Tools" -> "Core Debug Level" -> "Info"
 */
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
const uint32_t maxSuccessiveFailsCount = 3;

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
void watchdogCheck(esp_err_t status) {
    if (status == ESP_OK) {
        failsCount = 0;
    } else {
        failsCount++;
    }

    if (failsCount >= maxSuccessiveFailsCount) {
        ESP_LOGW("watchdog", "SDPWatchdog barked. Resetting...");
        esp_err_t err = ESP_FAIL;
        do {
            err = sdp.reset();  // sdp.stopContinuous() is a safer version
            if (err == ESP_OK) {
                err = sdp.startContinuous();
            }
        } while (err != ESP_OK);
    }

}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  sdp.initI2C(19, 23);  // same as Wire.begin(SDA, SCL)

  // try until succeeds
  while (sdp.stopContinuous() != ESP_OK);  // sdp.reset() is also possible
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);
}


void loop() {
  delay(1000);
  int16_t pressure;
  esp_err_t err = sdp.readDiffPressure(&pressure);
  ESP_LOGI("sdp", "raw pressure: %d, err code: %s", pressure, esp_err_to_name(err));

  /*
   *  Optionally, check the last result code.
   *  If enough errors occur, a software reset is issued.
   */
  watchdogCheck(err);
}
