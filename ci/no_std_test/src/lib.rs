#![no_std]

use bme680::*;
use log::info;

pub fn test_no_std() {
    info!("chipid {:?}", BME680_CHIP_ID);
}