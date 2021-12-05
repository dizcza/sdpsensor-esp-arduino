/**
 * An example to use an SDP3x sensor with an ESP32 board.
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
#include "sdpsensor.h"
#include <Wire.h>


/* An SDP sensor instance.
 * If you don't know the I2C address of a sensor,
 *   scan all addresses with an I2C scanner first.
 */
SDPSensor sdp(0x21);


/* Try until succeeds. */
bool initSensorUntilSuccess() {
  while (sdp.stopContinuous() != ESP_OK);  // sdp.reset() is also possible
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);
  return true;
}


/* Try until the first error occurs. */
bool initSensorSingleTrial() {
  sdp.stopContinuous();  // sdp.reset() is also possible
  sdp.begin();
  esp_err_t err = sdp.startContinuous();
  return err == ESP_OK;
}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  Wire.begin(19, 23);  // sdp.initI2C() is also possible
  Wire.setClock(400000);  // (optionally) set I2C frequency to high

  // choose how you want to initialize the sensor
  bool success = initSensorSingleTrial();
  //bool success = initSensorUntilSuccess();
  if (success) {
    uint32_t modelNumber;
    uint32_t rangePa;
    sdp.getInfo(&modelNumber, &rangePa, NULL, NULL);

    Serial.println("Initialized an SDP sensor:");
    Serial.print("  model number: ");
    Serial.println(modelNumber);
    Serial.print("  measurement range (in Pa): ");
    Serial.println(rangePa);
  } else {
    Serial.println("Failed to initialize an SDP sensor.");
  }
}


void loop() {
  char message[256];
  delay(1000);
  int16_t pressure;
  esp_err_t err = sdp.readDiffPressure(&pressure);
  if (err == ESP_OK) {
    // success; process the 'pressure' here
    float pressurePa = (float) pressure / sdp.getPressureScale();
    sprintf(message, "Pressure %d / %d = %.5f Pa", pressure, sdp.getPressureScale(), pressurePa);
    Serial.println(message);
  } else {
    Serial.print("FAILED: ");
    Serial.println(esp_err_to_name(err));
  }
}
