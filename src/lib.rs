#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use std::default::Default;
use std::result;

#[link(name = "example", kind = "static")]
mod consts;
mod device_builder;

#[cfg(test)]
mod tests {

    use device_builder::Bme680DeviceBuilder;

    extern "C" fn com(dev_id: u8, reg_addr: u8, data: *mut u8, len: u16) -> i8 {
        0
    }

    extern "C" fn delay(period: u32) {
        println!("delay {}", period);
    }

    #[test]
    fn init() {
        let dev = Bme680DeviceBuilder::new(com, com, delay).build();
        dev.init();
    }
}

pub type Result<T> = result::Result<T, Bme680Error>;

pub enum Bme680Error {
    ///
    /// aka BME680_E_NULL_PTR
    ///
    NulltPtr,
    ///
    /// aka BME680_E_COM_FAIL
    ///
    CommunicationFailure,
    ///
    /// aka BME680_E_DEV_NOT_FOUND
    ///
    DeviceNotFound,
    ///
    /// aka BME680_E_INVALID_LENGTH
    ///
    InvalidLength,
}

fn to_result(res: i8) -> Result<()> {
    match res {
        consts::BME680_OK => Ok(()),
        consts::BME680_E_NULL_PTR => Err(Bme680Error::NulltPtr),
        consts::BME680_E_COM_FAIL => Err(Bme680Error::CommunicationFailure),
        consts::BME680_E_DEV_NOT_FOUND => Err(Bme680Error::DeviceNotFound),
        consts::BME680_E_INVALID_LENGTH => Err(Bme680Error::InvalidLength),
        _ => panic!("Invalid bme680 result: {}", res),
    }
}

pub enum I2CAddr {
    Primary,
    Secondary,
}

///
/// Power mode settings
///
pub enum PowerMode {
    SleepMode,
    ForcedMode,
}
