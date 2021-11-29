# arduino-esp library for Sensirion SDP3x and SDP8xx series

This repository is tailored specifically for ESP boards and achieves **3X** faster performance.

To read the first three bytes (two for diff pressure and one CRC), it takes only 158 us compared to
450 us of a general-purpose arduino library [SDP3x-Arduino](https://github.com/DataDrake/SDP3x-Arduino).

## Usage

See [sdpsensor-esp-arduino.ino](./sdpsensor-esp-arduino.ino).

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

