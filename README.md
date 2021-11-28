# arduino-esp library for Sensirion SDP3x and SDP8xx series

This repository is tailored specifically for ESP boards and achieves **3X** faster performance.

To read the first three bytes (two for diff pressure and one CRC), it takes only 158 us compared to
450 us of a general-purpose arduino library [SDP3x-Arduino](https://github.com/DataDrake/SDP3x-Arduino).

## Usage

See [sdpsensor-esp-arduino.ino](./sdpsensor-esp-arduino.ino).

Serial output:

```
[  1021][I][sdpsensor.cpp:86] initI2C(): I2C0 line initialized
[  1021][I][sdpsensor.cpp:204] reset(): SDPSensor::reset ESP_OK
[  1042][I][sdpsensor.cpp:157] initSensor(): Initialized SDP31 500Pa sensor (PID=0x03010188)
[  1133][I][sdpsensor.cpp:169] initSensor(): SDP31 pressure scale: 60
[  1133][I][sdpsensor.cpp:182] startContinuous(): SDPSensor::startContinuous ESP_OK

[  2154][I][sdpsensor-esp-arduino.ino:32] loop(): raw pressure: 0, err code: ESP_OK
...
```

