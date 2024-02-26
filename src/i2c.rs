use embedded_hal::i2c::I2c;
use crate::Bme680Error;
use crate::Bme680Error::{I2CRead, I2CWrite};

///
/// Represents the I2C address of the BME680 Sensor.
///
#[derive(Debug, Clone, Copy)]
pub enum Address {
    /// Primary Address 0x76
    Primary,
    /// Secondary Address 0x77
    Secondary,
    /// Alternative address
    Other(u8),
}

impl Address {
    pub fn addr(&self) -> u8 {
        match &self {
            Address::Primary => 0x76u8,
            Address::Secondary => 0x77u8,
            Address::Other(addr) => *addr,
        }
    }
}

impl Default for Address {
    fn default() -> Address {
        Address::Primary
    }
}

/// I2CUtility is a simple wrapper over the I2c trait to make reading and writing data easier.
pub(crate) struct I2CUtility {}

impl I2CUtility {
    /// Reads a byte from the I2C bus.
    pub fn read_byte<I2C: I2c>(
        i2c_handle: &mut I2C,
        device_address: u8,
        register_address: u8,
    ) -> Result<u8, Bme680Error>
    {
        let mut buf = [0; 1];

        i2c_handle.write(device_address, &[register_address]).map_err(|_e| { I2CWrite })?;

        match i2c_handle.read(device_address, &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(_e) => Err(I2CRead),
        }
    }

    /// Reads bytes from the I2C bus.
    pub fn read_bytes<I2C: I2c>(
        i2c_handle: &mut I2C,
        device_address: u8,
        register_address: u8,
        buffer: &mut [u8],
    ) -> Result<(), Bme680Error>
    {
        i2c_handle.write(device_address, &[register_address]).map_err(|_e| { I2CWrite })?;

        match i2c_handle.read(device_address, buffer) {
            Ok(()) => Ok(()),
            Err(_e) => Err(I2CRead),
        }
    }

    /// Writes bytes to the I2C bus.
    pub fn write_bytes<I2C: I2c>(i2c_handle: &mut I2C, device_address: u8, buffer: &[u8]) -> Result<(), Bme680Error> {
        i2c_handle.write(device_address, &buffer).map_err(|_e| { I2CWrite })
    }
}
