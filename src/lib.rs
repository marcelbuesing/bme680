#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![feature(rustc_private)]
#![feature(try_from)]

//include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[macro_use]
extern crate bitflags;

extern crate byteorder;

extern crate embedded_hal as hal;

use hal::blocking::delay::{DelayMs, DelayUs};
use std::result;

#[link(name = "example", kind = "static")]
mod consts;
//mod device_builder;
mod bme680;

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

bitflags! {
    pub struct SensorSettings: u16 {
        /// To set temperature oversampling
        const OST_SEL = 1;
        /// To set pressure oversampling.
        const OSP_SEL = 2;

        /// To set humidity oversampling.
        const OSH_SEL = 4;
        /// To set gas measurement setting.
        const GAS_MEAS_SEL = 8;
        /// To set filter setting.
        const FILTER_SEL = 16;
        /// To set humidity control setting.
        const HCNTRL_SEL = 32;
        /// To set run gas setting.
        const RUN_GAS_SEL = 64;
        /// To set NB conversion setting.
        const NBCONV_SEL = 128;
        /// To set all gas sensor related settings
        const GAS_SENSOR_SEL = Self::GAS_MEAS_SEL.bits | Self::RUN_GAS_SEL.bits | Self::NBCONV_SEL.bits;
    }
}
