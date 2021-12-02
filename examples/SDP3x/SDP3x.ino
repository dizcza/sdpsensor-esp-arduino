/**
 * An example to use an SDP3x sensor with an ESP32 board.
 * 
 * Prior setup:
 *   1) install the arduino-esp library
 *   2) select your ESP board in "Tools" -> "Board"
 *   3) select the right port in "Tools" -> "Port"
 *   4) turn on the logs by setting "Tools" -> "Core Debug Level" -> "Info"
 */
#include "sdpsensor.h"


/* An SDP sensor instance.
 * If you don't know the I2C address of a sensor,
 *   scan all addresses with an I2C scanner first.
 */
SDPSensor sdp(0x21);


/* Try until succeeds. */
void initSensorUntilSuccess() {
  while (sdp.stopContinuous() != ESP_OK);  // sdp.reset() is also possible
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);
}


/* Try until the first error occurs. */
void initSensorSingleTrial() {
  sdp.stopContinuous();  // sdp.reset() is also possible
  sdp.begin();
  sdp.startContinuous();
}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  sdp.initI2C(19, 23);  // same as Wire.begin(SDA, SCL)

  // choose how you want to initialize the sensor
  initSensorSingleTrial();
  //initSensorUntilSuccess();
}


void loop() {
  delay(1000);
  int16_t pressure;
  esp_err_t err = sdp.readDiffPressure(&pressure);
  if (err == ESP_OK) {
    // success; process the 'pressure' here
  }
  ESP_LOGI("sdp", "raw pressure: %d, err code: %s", pressure, esp_err_to_name(err));
}
