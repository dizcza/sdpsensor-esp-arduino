# arduino-esp library for Sensirion SDP3x and SDP8xx series

ESP-IDF & Arduino friendly implementation of communication with Sensirion differential pressure sensors.

Compared to a general-purpose arduino library [SDP3x-Arduino](https://github.com/DataDrake/SDP3x-Arduino), it's:

1. a bit faster (158 vs 170 us). Benchmarks are measured for the 400kHz I2C clock frequency (`Wire.setClock()`).
2. not thread-safe. For thread-safe operations, use Arduino's Wire lib.
2. it has support for interrupt handlers (available for SDP3x sensors only). With interrupts, you can achieve reliable sensor updates at ~2100 Hz frequency. See the [SDP3x\_Interrupts](./examples/SDP3x_Interrupts/SDP3x_Interrupts.ino) demo.
2. ESP-IDF friendly: run this code either in Arduino or in your favourite ESP-IDF framework. For pure C implementation, refer to [`sdpsensor.c`](https://github.com/dizcza/esp32-sdpsensor/blob/master/main/sdpsensor.c).
3. DataDrake's implementation has a bug in the reset function. It's fixed here.
4. Each function returns a descriptive error code defined in [`esp_err.h`](https://github.com/espressif/esp-idf/blob/master/components/esp_common/include/esp_err.h). To check for success, call `if (err == ESP_OK) ...`.

## I2C address

Typical I2C sensor addresses:

* SDP31 & SDP32: `0x21`, `0x22`, `0x23`
* SDP8x0: `0x25`
* SDP8x1: `0x26`

## Usage

See [examples](./examples).

Serial output:

```
[  1021][I][sdpsensor.cpp:95] initI2C(): I2C0 line initialized
[  1021][I][sdpsensor.cpp:232] stopContinuous(): SDPSensor::stopContinuous ESP_OK
[  1043][I][sdpsensor.cpp:182] begin(): Initialized SDP31 500Pa sensor (PID=0x03010188)
[  1134][I][sdpsensor.cpp:200] begin(): SDP31 pressure scale: 60
[  1134][I][sdpsensor.cpp:219] startContinuous(): SDPSensor::startContinuous ESP_OK

[  2154][I][sdpsensor-esp-arduino.ino:32] loop(): raw pressure: 0, err code: ESP_OK
...
```

### Resetting the sensor

If the sensor was working in a continuous mode, prior to executing any I2C commands, it ought to be reset by calling either the `stopContinuous()` or `reset()` function. Note, however, that the `reset` commands are received by *all* I2C peripherals connected to the same I2C line (port), because it employs the `0x00` address, which is the I2C general call address.


## Capillary filter

To operate as a differential pressure sensor it must have a mechanical capillary filter attached to one of the ports. A low-pass frequency filter adapts to slowly changing atmospheric pressure, allowing mid- & high-frequency bands of differential pressure to be read from the other open port. Without the filter attached, the difference in pressure will be negligible.

