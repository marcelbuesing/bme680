BME680 + Rust [![Actions Status](https://github.com/marcelbuesing/bme680/workflows/Continuous%20integration/badge.svg)](https://github.com/marcelbuesing/bme680/actions?query=workflow%3A"Continuous+integration")
[![Cargo Deny Status](https://img.shields.io/badge/cargo--deny-license%20checked-green)](https://github.com/marcelbuesing/bme680/actions?query=workflow%3A"Continuous+integration")
=============

This repository contains a pure Rust implementation for the [BME680](https://www.bosch-sensortec.com/bst/products/all_products/bme680) environmental sensor. The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.

The library uses the [embedded-hal](https://github.com/japaric/embedded-hal) library to abstract reading and writing via I²C. In the examples you can find a demo how to use the library in Linux using the [linux-embedded-hal](https://github.com/japaric/linux-embedded-hal) implementation.

# Alternative
[drogue-bme680](https://github.com/drogue-iot/drogue-bme680)

# Example getting started Linux

Determine the I2C device path

```
pi@raspberrypi:~ $ i2cdetect -y -l
i2c-1    i2c       bcm2835 I2C adapter             I2C adapter
```

Determine I2C-Address of sensor, `0x76` is the primary address, `0x77` is the secondary address.
If in doubt determine the address via the following command:

```
pi@raspberrypi:~ $ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76
```

# Example Influx Client
The examples folder contains an example for a simple influx database client inserting collected values.
Below you may find examples of Chronograf graphs of an indoor measurement over a period of 30 days.

![Temperature Graph](examples/res/influx_temperature.png "Temperature measurement in C°")
![Humidity Graph](examples/res/influx_humidity.png "Humidity measurement in %")
![Air pressure Graph](examples/res/influx_pressure.png "Air pressure measurement in hPa")
![Gas resistance Graph](examples/res/influx_gas_resistance.png "Gas resistance measurement")
