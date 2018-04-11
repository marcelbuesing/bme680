pub const BME680_OK: i8 = 0;
pub const BME680_E_NULL_PTR: i8 = -1;
pub const BME680_E_COM_FAIL: i8 = -2;
pub const BME680_E_DEV_NOT_FOUND: i8 = -3;
pub const BME680_E_INVALID_LENGTH: i8 = -4;

pub const BME680_I2C_ADDR_PRIMARY: u8 = 0x76;
pub const BME680_I2C_ADDR_SECONDARY: u8 = 0x77;

pub const BME680_CHIP_ID: u8 = 0x61;

pub const BME680_SLEEP_MODE: u8 = 0;
pub const BME680_FORCED_MODE: u8 = 1;

///
/// SPI memory page settings
///
pub const BME680_MEM_PAGE0: u8 = 0x10;

///
/// SPI memory page settings
///
pub const BME680_MEM_PAGE1: u8 = 0x00;
