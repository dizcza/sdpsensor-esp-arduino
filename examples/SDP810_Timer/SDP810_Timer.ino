/**
 * An example to read an SDP sensor at a fixed rate.
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdpsensor.h"

#define SAMPLE_PERIOD_US  10000  // in microseconds

static TaskHandle_t read_sensor_task_handle;

SDPSensor sdp(0x25);


static void sdp_timer_callback(void* arg)
{
  xTaskNotifyGive(read_sensor_task_handle);
}



static void sdptask_read_sensor() {
  esp_err_t err;
  int16_t diff_pressure;

  while (1) {
    // wait for an interrupt
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    err = sdp.readDiffPressure(&diff_pressure);

    if (err == ESP_OK) {
      // send to a receiver here
    }
    ESP_LOGI("main", "err %s val %d", esp_err_to_name(err), diff_pressure);

    // Note that you should never print here any messages because
    // printing to serial is VERY slow. It's done for demonstration
    // purposes here.
    Serial.print("Raw diff pressure: ");
    Serial.println(diff_pressure);
  }
}


bool startTimer() {
  esp_timer_handle_t sdp_timer;
  const esp_timer_create_args_t sdp_timer_args = {
          .callback = &sdp_timer_callback,
          .name = "sdp_timer"
  };
  if (esp_timer_create(&sdp_timer_args, &sdp_timer) != ESP_OK) {
    Serial.println("Failed to create a timer");
    return false;
  }
  if (esp_timer_start_periodic(sdp_timer, SAMPLE_PERIOD_US) != ESP_OK) {
    Serial.println("Failed to start a periodic timer");
    return false;
  }
  Serial.println("The periodic timer is started");
  return true;
}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  Wire.begin(19, 23);
  Wire.setClock(400000);  // allow higher sampling rates

  // you should be already familiar with this
  while (sdp.stopContinuous() != ESP_OK);
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);

  // Start a task on the second core with the priority '1'.
  // The task must be started before the timer.
  xTaskCreatePinnedToCore((TaskFunction_t) sdptask_read_sensor, "sdp_read", 4096, NULL, 1, &read_sensor_task_handle, APP_CPU_NUM);

  if (!startTimer()) {
    // failed; stop the task
    vTaskDelete(read_sensor_task_handle);
    return;
  }
}


void loop() {
  // sdptask_read_sensor task should be running
}
