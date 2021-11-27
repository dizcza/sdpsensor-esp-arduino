# arduino-esp library for Sensirion SDP3x and SDP8xx series

This repository is tailored specifically for ESP boards and achieves **3X** faster performance.

To read the first three bytes (two for diff pressure and one CRC), it takes only 158 us compared to
450 us of a general-purpose arduino library [SDP3x-Arduino](https://github.com/DataDrake/SDP3x-Arduino).

## Usage

See [sdpsensor-esp-arduino.ino](./sdpsensor-esp-arduino.ino).

