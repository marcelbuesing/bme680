# BME680 + Rust
This repository contains a native Rust implementation for the [BME680](https://www.bosch-sensortec.com/bst/products/all_products/bme680) environmental sensor. The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.

The library uses the [embedded-hal](https://github.com/japaric/embedded-hal) library to abstract reading and writing via I²C. In the examples you can find a demo how to use the library in Linux using the [linux-embedded-hal](https://github.com/japaric/linux-embedded-hal) implementation.
