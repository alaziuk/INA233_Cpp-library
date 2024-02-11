# INA233 C++ Library

## Overview

This library is fork from [infinityPV_INA233_Arduino-library](https://github.com/infinityPV/infinityPV_INA233_Arduino-library). It was repurposed for general C++ usage. This C++ library provides functionality to interface with the TI INA233 current/power monitor. The library facilitates communication with the device over the I2C bus, reads and writes data, and performs calibration and scaling as per the specifications outlined in section 7.5.2 of the TI INA233 datasheet.


## Features

- **I2C Communication**: Establishes communication with the INA233 over the I2C bus.
- **Register Read/Write**: Reads and writes 16-bit words to specific registers on the INA233.
- **Calibration and Scaling**: Implements calibration and scaling functions as per TI INA233 datasheet.
- **Raw Measurements**: Retrieves raw values for bus voltage, shunt voltage, current, power, and energy.
- **Processed Measurements**: Provides processed measurements for voltage, current, power, and average power in appropriate units.

## Repository Contents

* Source files for the library (.cpp, .h).

## Documentation

* **[INA233 datasheet](http://www.ti.com/lit/ds/symlink/ina233.pdf)** - INA233 ADC/I2C datasheet from Texas Instruments.

## How to Use

1. **Include the Library**: Include the INA233.h header file in your project.
2. **Instantiate INA233 Object**: Create an INA233 object with the desired I2C bus and device address.
3. **Calibrate the Device**: Use the `calibrate` function to calibrate the INA233 for accurate measurements.
4. **Read Measurements**: Utilize the provided functions to obtain raw and processed measurements from the INA233.

## Dependencies

- `iostream`: C++ Standard I/O Library
- `fcntl.h`: File Control Library
- `linux/i2c-dev.h`: Linux I2C Device Access Library
- `i2c/smbus.h`: I2C SMBus Protocol Library
- `sys/ioctl.h`: Input/Output Control Library
- `unistd.h`: Standard Symbolic Constants and Types Library
- `cmath`: C++ Standard Mathematical Functions Library

Version History
---------------


## License

This product is _**open source**_! 

Please review the [license.txt](license.txt) file for license information. 

Distributed as-is; no warranty is given.

## Version History

- **v1.0 (Feb 2024)**: Initial release.
