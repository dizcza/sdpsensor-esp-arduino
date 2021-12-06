/**
 * An example to use SDP3x sensor interrupts.
 * 
 * Software setup:
 *   1) install the arduino-esp library
 *   2) select your ESP board in "Tools" -> "Board"
 *   3) select the right port in "Tools" -> "Port"
 *   4) turn on the logs by setting "Tools" -> "Core Debug Level" -> "Info"
 * Hardware setup:
 *   1) connect GPIO SDA to pin 19
 *   2) connect GPIO SCL to pin 23
 *   3) connect GPIO IRQ to pin 4 (!)
 * or change the code appropriately.
 */

#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdpsensor.h"

#define GPIO_IRQ_PIN  4  // SDP3x IRQ pin connected to your board

static TaskHandle_t read_sensor_task_handle;

SDPSensor sdp(0x21);


static void IRAM_ATTR sdpsensor_irq_handler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(read_sensor_task_handle, &xHigherPriorityTaskWoken);

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
   should be performed to ensure the interrupt returns directly to the highest
   priority task.  The macro used for this purpose is dependent on the port in
   use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    // Note that you should never print here any messages because
    // printing to serial is VERY slow. It's done for demonstration
    // purposes here.
    Serial.print("Raw diff pressure: ");
    Serial.println(diff_pressure);
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000); // let serial console settle
  Wire.begin(19, 23);
  Wire.setClock(400000); // allow high speed

  // you should be already familiar with this
  while (sdp.stopContinuous() != ESP_OK);
  while (sdp.begin() != ESP_OK);
  while (sdp.startContinuous() != ESP_OK);

  uint32_t modelNumber;
  sdp.getInfo(&modelNumber, NULL, NULL, NULL);
  if (!(modelNumber == 31 || modelNumber == 32)) {
    Serial.println("WARNING! Your SDP sensor does NOT support interrupts!");
  }

  if (sdp.attachIRQHandler(GPIO_IRQ_PIN, sdpsensor_irq_handler) != ESP_OK) {
    Serial.println("Failed to hook an interrupt handler. Check the wiring.");
    return;
  }
  
  // start a task on the second core with the priority '1'
  xTaskCreatePinnedToCore((TaskFunction_t) sdptask_read_sensor, "sdp_read", 4096, NULL, 1, &read_sensor_task_handle, APP_CPU_NUM);

  // allow the sensor to run the first measurement
  xTaskNotifyGive(read_sensor_task_handle);
}


void loop() {
  // sdptask_read_sensor task is running
}
