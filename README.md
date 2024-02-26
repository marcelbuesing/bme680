BME680 Rust Library
=============

This repository contains a pure Rust implementation for the [BME680](https://www.bosch-sensortec.com/bst/products/all_products/bme680) environmental sensor. 
The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.

It is a fork of the library written by Marcel Buessing: https://github.com/marcelbuesing/bme680.

To use this library, create a new project and add it as a dependency:

```toml
[dependencies]
bme680 = {git = "https://github.com/dnutiu/bme680-rust.git", version = "0.8.0"}
```

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