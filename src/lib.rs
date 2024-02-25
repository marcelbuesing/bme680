//! This crate is a pure Rust implementation for the BME680 environmental sensor.
//! The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.
//!
//! The library uses the embedded-hal crate to abstract reading and writing via I²C.
//! In the examples you can find a demo how to use the library in Linux using the linux-embedded-hal crate (e.g. on a RPI).
//! ```no_run


//! use bme680::{Bme680, Error, I2CAddress, IIRFilterSize, OversamplingSetting, PowerMode, SettingsBuilder};
//! use core::result;
//! use core::time::Duration;
//! use embedded_hal::delay::DelayNs;
//! use linux_embedded_hal as hal;
//! use linux_embedded_hal::{Delay, I2CError};
//! use log::info;
//!
//! // Please export RUST_LOG=info in order to see logs in the console.
//! fn main() -> result::Result<(), Error<I2CError>>
//! {
//!     env_logger::init();
//!
//!     let i2c = hal::I2cdev::new("/dev/i2c-1").unwrap();
//!     let mut delayer = Delay {};
//!
//!     let mut dev = Bme680::init(i2c, &mut delayer, I2CAddress::Primary)?;
//!     let mut delay = Delay {};
//!
//!     let settings = SettingsBuilder::new()
//!         .with_humidity_oversampling(OversamplingSetting::OS2x)
//!         .with_pressure_oversampling(OversamplingSetting::OS4x)
//!         .with_temperature_oversampling(OversamplingSetting::OS8x)
//!         .with_temperature_filter(IIRFilterSize::Size3)
//!         .with_gas_measurement(Duration::from_millis(1500), 320, 25)
//!         .with_temperature_offset(-2.2)
//!         .with_run_gas(true)
//!         .build();
//!
//!     let profile_dur = dev.get_profile_dur(&settings.0)?;
//!     info!("Profile duration {:?}", profile_dur);
//!     info!("Setting sensor settings");
//!     dev.set_sensor_settings(&mut delayer, settings)?;
//!     info!("Setting forced power modes");
//!     dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)?;
//!
//!     let sensor_settings = dev.get_sensor_settings(settings.1);
//!     info!("Sensor settings: {:?}", sensor_settings);
//!
//!     loop {
//!         let _ = delay.delay_ms(5000u32);
//!         let power_mode = dev.get_sensor_mode();
//!         info!("Sensor power mode: {:?}", power_mode);
//!         info!("Setting forced power modes");
//!         dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)?;
//!         info!("Retrieving sensor data");
//!         let (data, _state) = dev.get_sensor_data(&mut delayer)?;
//!         info!("Sensor Data {:?}", data);
//!         info!("Temperature {}°C", data.temperature_celsius());
//!         info!("Pressure {}hPa", data.pressure_hpa());
//!         info!("Humidity {}%", data.humidity_percent());
//!         info!("Gas Resistence {}Ω", data.gas_resistance_ohm());
//!     }
//! }

#![no_std]
#![forbid(unsafe_code)]

pub use self::settings::{
    DesiredSensorSettings, GasSett, IIRFilterSize, OversamplingSetting, SensorSettings, Settings,
    SettingsBuilder, TphSett,
};

mod calculation;
mod settings;

use crate::calculation::Calculation;
use crate::hal::delay::DelayNs;
use crate::hal::i2c::I2c;

use core::time::Duration;
use core::{marker::PhantomData, result};
use embedded_hal as hal;
use embedded_hal::i2c::ErrorType;
use log::{debug, error, info};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// BME680 General config
pub const BME680_POLL_PERIOD_MS: u8 = 10;

/// BME680 unique chip identifier
pub const BME680_CHIP_ID: u8 = 0x61;

/// BME680 field_x related defines
const BME680_FIELD_LENGTH: usize = 15;

/// BME680 coefficients related defines
const BME680_COEFF_ADDR1_LEN: usize = 25;
const BME680_COEFF_ADDR2_LEN: usize = 16;

const BME680_SOFT_RESET_CMD: u8 = 0xb6;

/// Register map
/// Other coefficient's address
const BME680_ADDR_RES_HEAT_VAL_ADDR: u8 = 0x00;
const BME680_ADDR_RES_HEAT_RANGE_ADDR: u8 = 0x02;
const BME680_ADDR_RANGE_SW_ERR_ADDR: u8 = 0x04;
const BME680_ADDR_SENS_CONF_START: u8 = 0x5A;
const BME680_ADDR_GAS_CONF_START: u8 = 0x64;

const BME680_SOFT_RESET_ADDR: u8 = 0xe0;

/// Field settings
const BME680_FIELD0_ADDR: u8 = 0x1d;

/// Heater settings
const BME680_RES_HEAT0_ADDR: u8 = 0x5a;
const BME680_GAS_WAIT0_ADDR: u8 = 0x64;

/// Sensor configuration registers
const BME680_CONF_HEAT_CTRL_ADDR: u8 = 0x70;
const BME680_CONF_ODR_RUN_GAS_NBC_ADDR: u8 = 0x71;
const BME680_CONF_OS_H_ADDR: u8 = 0x72;
const BME680_CONF_T_P_MODE_ADDR: u8 = 0x74;
const BME680_CONF_ODR_FILT_ADDR: u8 = 0x75;

/// Coefficient's address
const BME680_COEFF_ADDR1: u8 = 0x89;
const BME680_COEFF_ADDR2: u8 = 0xe1;

/// Chip identifier
const BME680_CHIP_ID_ADDR: u8 = 0xd0;

const BME680_SLEEP_MODE: u8 = 0;
const BME680_FORCED_MODE: u8 = 1;

const BME680_RESET_PERIOD: u8 = 10;

const BME680_MODE_MSK: u8 = 0x03;
const BME680_RSERROR_MSK: u8 = 0xf0;
const BME680_NEW_DATA_MSK: u8 = 0x80;
const BME680_GAS_INDEX_MSK: u8 = 0x0f;
const BME680_GAS_RANGE_MSK: u8 = 0x0f;
const BME680_GASM_VALID_MSK: u8 = 0x20;
const BME680_HEAT_STAB_MSK: u8 = 0x10;

/// Buffer length macro declaration
const BME680_TMP_BUFFER_LENGTH: usize = 40;
const BME680_REG_BUFFER_LENGTH: usize = 6;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    ///
    /// aka BME680_E_COM_FAIL
    ///
    I2CWrite(E),
    I2CRead(E),
    Delay,
    ///
    /// aka BME680_E_DEV_NOT_FOUND
    ///
    DeviceNotFound,
    ///
    /// aka BME680_E_INVALID_LENGTH
    ///
    InvalidLength,
    ///
    /// Warning aka BME680_W_DEFINE_PWR_MODE
    ///
    DefinePwrMode,
    ///
    /// Warning aka BME680_W_DEFINE_PWR_MODE
    ///
    NoNewData,
    ///
    /// Warning Boundary Check
    ///
    BoundaryCheckFailure(&'static str),
}

/// Abbreviates `std::result::Result` type
pub type Result<T, E> = result::Result<T, Error<E>>;

///
/// Power mode settings
///
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PowerMode {
    SleepMode,
    ForcedMode,
}

impl PowerMode {
    // TODO replace with TryFrom once stabilized
    fn from(power_mode: u8) -> Self {
        match power_mode {
            BME680_SLEEP_MODE => PowerMode::SleepMode,
            BME680_FORCED_MODE => PowerMode::ForcedMode,
            _ => panic!("Unknown power mode: {}", power_mode),
        }
    }

    fn value(&self) -> u8 {
        match self {
            PowerMode::SleepMode => BME680_SLEEP_MODE,
            PowerMode::ForcedMode => BME680_FORCED_MODE,
        }
    }
}

///
/// I2C Slave Address
/// To determine the slave address of your device you can use `i2cdetect -y 1` on linux.
/// The 7-bit device address is 111011x. The 6 MSB bits are fixed.
/// The last bit is changeable by SDO value and can be changed during operation.
/// Connecting SDO to GND results in slave address 1110110 (0x76); connecting it to V DDIO results in slave
/// address 1110111 (0x77), which is the same as BMP280’s I2C address.
///
#[derive(Debug, Clone, Copy)]
pub enum I2CAddress {
    /// Primary Slave Address 0x76
    Primary,
    /// Secondary Slave Address 0x77
    Secondary,
    /// Alternative address
    Other(u8),
}

impl I2CAddress {
    pub fn addr(&self) -> u8 {
        match &self {
            I2CAddress::Primary => 0x76u8,
            I2CAddress::Secondary => 0x77u8,
            I2CAddress::Other(addr) => *addr,
        }
    }
}

impl Default for I2CAddress {
    fn default() -> I2CAddress {
        I2CAddress::Primary
    }
}

/// Calibration data used during initalization
#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct CalibrationData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: u8,
}

impl Clone for CalibrationData {
    fn clone(&self) -> Self {
        *self
    }
}

/// Contains read sensors values  e.g. temperature, pressure, humidity etc.
#[derive(Debug, Default, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(C)]
pub struct FieldData {
    /// Contains new_data, gasm_valid & heat_stab
    status: u8,
    /// Index of heater profile used
    gas_index: u8,
    /// Measurement index
    meas_index: u8,
    temperature: i16,
    pressure: u32,
    humidity: u32,
    gas_resistance: u32,
}

impl Clone for FieldData {
    fn clone(&self) -> Self {
        *self
    }
}

impl FieldData {
    /// Temperature in degree celsius (°C)
    pub fn temperature_celsius(&self) -> f32 {
        self.temperature as f32 / 100f32
    }

    /// Pressure in hectopascal (hPA)
    pub fn pressure_hpa(&self) -> f32 {
        self.pressure as f32 / 100f32
    }

    /// Humidity in % relative humidity
    pub fn humidity_percent(&self) -> f32 {
        self.humidity as f32 / 1000f32
    }

    pub fn gas_resistance_ohm(&self) -> u32 {
        self.gas_resistance
    }

    /// Whether a real (and not a dummy) gas reading was performed.
    pub fn gas_valid(&self) -> bool {
        self.status & BME680_GASM_VALID_MSK != 0
    }

    /// Whether the heater target temperature for the gas reading was reached.
    ///
    /// If this values is `false`, the heating duration was likely too short or
    /// the target temperature too high.
    pub fn heat_stable(&self) -> bool {
        self.status & BME680_HEAT_STAB_MSK != 0
    }
}

/// Shows if new data is available
#[derive(PartialEq, Debug)]
pub enum FieldDataCondition {
    ///
    /// Data changed since last read
    ///
    NewData,
    ///
    /// Data has not changed since last read
    ///
    Unchanged,
}

struct I2CUtil {}

impl I2CUtil {
    pub fn read_byte<I2C>(
        i2c: &mut I2C,
        dev_id: u8,
        reg_addr: u8,
    ) -> Result<u8, <I2C as ErrorType>::Error>
        where
            I2C: I2c
    {
        let mut buf = [0; 1];

        i2c.write(dev_id, &[reg_addr]).map_err(Error::I2CWrite)?;

        match i2c.read(dev_id, &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(Error::I2CRead(e)),
        }
    }

    pub fn read_bytes<I2C>(
        i2c: &mut I2C,
        dev_id: u8,
        reg_addr: u8,
        buf: &mut [u8],
    ) -> Result<(), <I2C as ErrorType>::Error>
        where
            I2C: I2c,
    {
        i2c.write(dev_id, &[reg_addr]).map_err(Error::I2CWrite)?;

        match i2c.read(dev_id, buf) {
            Ok(()) => Ok(()),
            Err(e) => Err(Error::I2CRead(e)),
        }
    }
}

/// Driver for the BME680 environmental sensor
#[repr(C)]
pub struct Bme680<I2C, D> {
    i2c: I2C,
    delay: PhantomData<D>,
    dev_id: I2CAddress,
    calib: CalibrationData,
    // TODO remove ? as it may not reflect the state of the device
    tph_sett: TphSett,
    // TODO remove ? as it may not reflect the state of the device
    gas_sett: GasSett,
    // TODO remove ? as it may not reflect the state of the device
    power_mode: PowerMode,
}

fn boundary_check<I2C>(
    value: Option<u8>,
    value_name: &'static str,
    min: u8,
    max: u8,
) -> Result<u8, <I2C as ErrorType>::Error>
    where
        I2C: I2c,
{
    let value = value.ok_or(Error::BoundaryCheckFailure(value_name))?;

    if value < min {
        const MIN: &str = "Boundary check failure, value exceeds maximum";
        error!("{}, value name: {}", MIN, value_name);
        return Err(Error::BoundaryCheckFailure(MIN));
    }

    if value > max {
        const MAX: &str = "Boundary check, value exceeds minimum";
        error!("{}, value name: {}", MAX, value_name);
        return Err(Error::BoundaryCheckFailure(MAX));
    }
    Ok(value)
}

impl<I2C, D> Bme680<I2C, D>
    where
        D: DelayNs,
        I2C: I2c,
{
    pub fn soft_reset(
        i2c: &mut I2C,
        delay: &mut D,
        dev_id: I2CAddress,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        let tmp_buff: [u8; 2] = [BME680_SOFT_RESET_ADDR, BME680_SOFT_RESET_CMD];

        i2c.write(dev_id.addr(), &tmp_buff)
            .map_err(Error::I2CWrite)?;

        delay.delay_ms(BME680_RESET_PERIOD as u32);
        Ok(())
    }

    pub fn init(
        mut i2c: I2C,
        delay: &mut D,
        dev_id: I2CAddress,
    ) -> Result<Bme680<I2C, D>, <I2C as ErrorType>::Error> {
        Bme680::soft_reset(&mut i2c, delay, dev_id)?;

        debug!("Reading chip id");
        /* Soft reset to restore it to default values*/
        let chip_id = I2CUtil::read_byte::<I2C>(&mut i2c, dev_id.addr(), BME680_CHIP_ID_ADDR)?;
        debug!("Chip id: {}", chip_id);

        if chip_id == BME680_CHIP_ID {
            debug!("Reading calibration data");
            let calibration = Bme680::<I2C, D>::get_calib_data::<I2C>(&mut i2c, dev_id)?;
            debug!("Calibration data {:?}", calibration);
            let dev = Bme680 {
                i2c,
                delay: PhantomData,
                dev_id,
                calib: calibration,
                power_mode: PowerMode::ForcedMode,
                tph_sett: Default::default(),
                gas_sett: Default::default(),
            };
            info!("Finished device init");
            Ok(dev)
        } else {
            error!("Device does not match chip id {}", BME680_CHIP_ID);
            Err(Error::DeviceNotFound)
        }
    }

    fn bme680_set_regs(
        &mut self,
        reg: &[(u8, u8)],
    ) -> Result<(), <I2C as ErrorType>::Error> {
        if reg.is_empty() || reg.len() > (BME680_TMP_BUFFER_LENGTH / 2) as usize {
            return Err(Error::InvalidLength);
        }

        for (reg_addr, reg_data) in reg {
            let tmp_buff: [u8; 2] = [*reg_addr, *reg_data];
            debug!(
                "Setting register reg: {:?} tmp_buf: {:?}",
                reg_addr, tmp_buff
            );
            self.i2c
                .write(self.dev_id.addr(), &tmp_buff)
                .map_err(Error::I2CWrite)?;
        }

        Ok(())
    }

    /// Set the settings to be used during the sensor measurements
    pub fn set_sensor_settings(
        &mut self,
        delay: &mut D,
        settings: Settings,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        let (sensor_settings, desired_settings) = settings;
        let tph_sett = sensor_settings.tph_sett;
        let gas_sett = sensor_settings.gas_sett;

        let mut reg: [(u8, u8); BME680_REG_BUFFER_LENGTH] = [(0, 0); BME680_REG_BUFFER_LENGTH];
        let intended_power_mode = self.power_mode;

        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            debug!("GAS_MEAS_SEL: true");
            self.set_gas_config(gas_sett)?;
        }

        let power_mode = self.power_mode;
        self.set_sensor_mode(delay, power_mode)?;

        let mut element_index = 0;
        // Selecting the filter
        if desired_settings.contains(DesiredSensorSettings::FILTER_SEL) {
            let mut data =
                I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_ODR_FILT_ADDR)?;

            debug!("FILTER_SEL: true");
            data = (data as i32 & !0x1ci32
                | (tph_sett.filter.unwrap_or(IIRFilterSize::Size0) as i32) << 2i32 & 0x1ci32)
                as u8;
            reg[element_index] = (BME680_CONF_ODR_FILT_ADDR, data);
            element_index += 1;
        }

        if desired_settings.contains(DesiredSensorSettings::HCNTRL_SEL) {
            debug!("HCNTRL_SEL: true");
            let gas_sett_heatr_ctrl =
                boundary_check::<I2C>(gas_sett.heatr_ctrl, "GasSett.heatr_ctrl", 0x0u8, 0x8u8)?;
            let mut data = I2CUtil::read_byte(
                &mut self.i2c,
                self.dev_id.addr(),
                BME680_CONF_HEAT_CTRL_ADDR,
            )?;
            data = (data as i32 & !0x8i32 | gas_sett_heatr_ctrl as i32 & 0x8) as u8;
            reg[element_index] = (BME680_CONF_HEAT_CTRL_ADDR, data);
            element_index += 1;
        }

        // Selecting heater T,P oversampling for the sensor
        if desired_settings
            .contains(DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL)
        {
            let mut data =
                I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;

            if desired_settings.contains(DesiredSensorSettings::OST_SEL) {
                debug!("OST_SEL: true");
                let tph_sett_os_temp = boundary_check::<I2C>(
                    tph_sett.os_temp.map(|x| x as u8),
                    "TphSett.os_temp",
                    0,
                    5,
                )?;
                data = (data as i32 & !0xe0i32 | (tph_sett_os_temp as i32) << 5i32 & 0xe0i32) as u8;
            }

            if desired_settings.contains(DesiredSensorSettings::OSP_SEL) {
                debug!("OSP_SEL: true");
                let tph_sett_os_pres = tph_sett.os_temp.unwrap_or(OversamplingSetting::OS1x);
                data = (data as i32 & !0x1ci32 | (tph_sett_os_pres as i32) << 2i32 & 0x1ci32) as u8;
            }
            reg[element_index] = (BME680_CONF_T_P_MODE_ADDR, data);
            element_index += 1;
        }

        // Selecting humidity oversampling for the sensor
        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            debug!("OSH_SEL: true");
            let tph_sett_os_hum =
                boundary_check::<I2C>(tph_sett.os_hum.map(|x| x as u8), "TphSett.os_hum", 0, 5)?;
            let mut data =
                I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_OS_H_ADDR)?;
            data = (data as i32 & !0x7i32 | tph_sett_os_hum as i32 & 0x7i32) as u8;
            reg[element_index] = (BME680_CONF_OS_H_ADDR, data);
            element_index += 1;
        }

        // Selecting the runGas and NB conversion settings for the sensor
        if desired_settings
            .contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL)
        {
            let mut data = I2CUtil::read_byte(
                &mut self.i2c,
                self.dev_id.addr(),
                BME680_CONF_ODR_RUN_GAS_NBC_ADDR,
            )?;

            if desired_settings.contains(DesiredSensorSettings::RUN_GAS_SEL) {
                debug!("RUN_GAS_SEL: true");
                data = (data as i32 & !0x10i32
                    | (gas_sett.run_gas_measurement as i32) << 4i32 & 0x10i32)
                    as u8;
            }

            if desired_settings.contains(DesiredSensorSettings::NBCONV_SEL) {
                debug!("NBCONV_SEL: true");
                let gas_sett_nb_conv =
                    boundary_check::<I2C>(Some(gas_sett.nb_conv), "GasSett.nb_conv", 0, 10)?;
                data = (data as i32 & !0xfi32 | gas_sett_nb_conv as i32 & 0xfi32) as u8;
            }

            reg[element_index] = (BME680_CONF_ODR_RUN_GAS_NBC_ADDR, data);
            element_index += 1;
        }

        self.bme680_set_regs(&reg[0..element_index])?;

        // Restore previous intended power mode
        self.power_mode = intended_power_mode;
        self.tph_sett = tph_sett;
        Ok(())
    }

    /// Retrieve settings from sensor registers
    ///
    /// # Arguments
    ///
    /// * `desired_settings` - Settings to be retrieved. Setting values may stay `None` if not retrieved.
    pub fn get_sensor_settings(
        &mut self,
        desired_settings: DesiredSensorSettings,
    ) -> Result<SensorSettings, <I2C as ErrorType>::Error> {
        let reg_addr: u8 = 0x70u8;
        let mut data_array: [u8; BME680_REG_BUFFER_LENGTH] = [0; BME680_REG_BUFFER_LENGTH];
        let mut sensor_settings: SensorSettings = Default::default();
        sensor_settings.tph_sett.temperature_offset = self.tph_sett.temperature_offset;

        I2CUtil::read_bytes(&mut self.i2c, self.dev_id.addr(), reg_addr, &mut data_array)?;

        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            sensor_settings.gas_sett = self.get_gas_config()?;
        }

        if desired_settings.contains(DesiredSensorSettings::FILTER_SEL) {
            sensor_settings.tph_sett.filter = Some(IIRFilterSize::from_u8(
                ((data_array[5usize] as i32 & 0x1ci32) >> 2i32) as u8,
            ));
        }

        if desired_settings
            .contains(DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL)
        {
            let os_temp: u8 = ((data_array[4usize] as i32 & 0xe0i32) >> 5i32) as u8;
            let os_pres: u8 = ((data_array[4usize] as i32 & 0x1ci32) >> 2i32) as u8;
            sensor_settings.tph_sett.os_temp = Some(OversamplingSetting::from_u8(os_temp));
            sensor_settings.tph_sett.os_pres = Some(OversamplingSetting::from_u8(os_pres));
        }

        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            let os_hum: u8 = (data_array[2usize] as i32 & 0x7i32) as u8;
            sensor_settings.tph_sett.os_hum = Some(OversamplingSetting::from_u8(os_hum));
        }

        if desired_settings.contains(DesiredSensorSettings::HCNTRL_SEL) {
            sensor_settings.gas_sett.heatr_ctrl = Some((data_array[0usize] as i32 & 0x8i32) as u8);
        }

        if desired_settings
            .contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL)
        {
            sensor_settings.gas_sett.nb_conv = (data_array[1usize] as i32 & 0xfi32) as u8;
            sensor_settings.gas_sett.run_gas_measurement =
                ((data_array[1usize] as i32 & 0x10i32) >> 4i32) == 0;
        }

        Ok(sensor_settings)
    }

    /// Set the sensor into a certain power mode
    ///
    /// # Arguments
    ///
    /// * `target_power_mode` - Desired target power mode
    pub fn set_sensor_mode(
        &mut self,
        delay: &mut D,
        target_power_mode: PowerMode,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        let mut tmp_pow_mode: u8;
        let mut current_power_mode: PowerMode;

        // Call repeatedly until in sleep
        loop {
            tmp_pow_mode =
                I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;

            // Put to sleep before changing mode
            current_power_mode = PowerMode::from(tmp_pow_mode & BME680_MODE_MSK);

            debug!("Current power mode: {:?}", current_power_mode);

            if current_power_mode != PowerMode::SleepMode {
                // Set to sleep
                tmp_pow_mode &= !BME680_MODE_MSK;
                debug!("Setting to sleep tmp_pow_mode: {}", tmp_pow_mode);
                self.bme680_set_regs(&[(BME680_CONF_T_P_MODE_ADDR, tmp_pow_mode)])?;
                delay
                    .delay_ms(BME680_POLL_PERIOD_MS as u32);
            } else {
                // TODO do while in Rust?
                break;
            }
        }

        // Already in sleep
        if target_power_mode != PowerMode::SleepMode {
            tmp_pow_mode = tmp_pow_mode & !BME680_MODE_MSK | target_power_mode.value();
            debug!("Already in sleep Target power mode: {}", tmp_pow_mode);
            self.bme680_set_regs(&[(BME680_CONF_T_P_MODE_ADDR, tmp_pow_mode)])?;
        }
        Ok(())
    }

    /// Retrieve current sensor power mode via registers
    pub fn get_sensor_mode(
        &mut self,
    ) -> Result<PowerMode, <I2C as ErrorType>::Error> {
        let regs =
            I2CUtil::read_byte(&mut self.i2c, self.dev_id.addr(), BME680_CONF_T_P_MODE_ADDR)?;
        let mode = regs & BME680_MODE_MSK;
        Ok(PowerMode::from(mode))
    }

    pub fn bme680_set_profile_dur(&mut self, tph_sett: TphSett, duration: Duration) {
        let os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        // TODO check if the following unwrap_ors do not change behaviour
        // TODO replace once https://github.com/rust-lang/rust/pull/50167 has been merged
        const MILLIS_PER_SEC: u64 = 1_000;
        const NANOS_PER_MILLI: u64 = 1_000_000;
        let millis = (duration.as_secs() as u64 * MILLIS_PER_SEC)
            + (duration.subsec_nanos() as u64 / NANOS_PER_MILLI);

        let mut meas_cycles = os_to_meas_cycles
            [tph_sett.os_temp.unwrap_or(OversamplingSetting::OSNone) as usize]
            as u64;
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[tph_sett.os_pres.unwrap_or(OversamplingSetting::OSNone) as usize]
                as u64,
        );
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[tph_sett.os_hum.unwrap_or(OversamplingSetting::OSNone) as usize]
                as u64,
        );
        let mut tph_dur = meas_cycles.wrapping_mul(1963u64);
        tph_dur = tph_dur.wrapping_add(477u64.wrapping_mul(4u64));
        tph_dur = tph_dur.wrapping_add(477u64.wrapping_mul(5u64));
        tph_dur = tph_dur.wrapping_add(500u64);
        tph_dur = tph_dur.wrapping_div(1000u64);
        tph_dur = tph_dur.wrapping_add(1u64);
        self.gas_sett.heatr_dur = Some(Duration::from_millis(millis - tph_dur));
    }

    pub fn get_profile_dur(
        &self,
        sensor_settings: &SensorSettings,
    ) -> Result<Duration, <I2C as ErrorType>::Error> {
        let os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        // TODO check if the following unwrap_ors do not change behaviour
        let mut meas_cycles = os_to_meas_cycles[sensor_settings
            .tph_sett
            .os_temp
            .unwrap_or(OversamplingSetting::OSNone)
            as usize] as u32;
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .tph_sett
                .os_pres
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        meas_cycles = meas_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .tph_sett
                .os_hum
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        let mut tph_dur = meas_cycles.wrapping_mul(1963u32);
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
        tph_dur = tph_dur.wrapping_add(500u32);
        tph_dur = tph_dur.wrapping_div(1000u32);
        tph_dur = tph_dur.wrapping_add(1u32);
        let mut duration = Duration::from_millis(tph_dur as u64);
        if sensor_settings.gas_sett.run_gas_measurement {
            duration += sensor_settings.gas_sett.heatr_dur.unwrap_or(Duration::default());
        }
        Ok(duration)
    }

    fn get_calib_data<I2CX>(
        i2c: &mut I2CX,
        dev_id: I2CAddress,
    ) -> Result<CalibrationData, <I2C as ErrorType>::Error>
        where
            I2CX: I2c,
    {
        let mut calib: CalibrationData = Default::default();

        let mut coeff_array: [u8; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN] =
            [0; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN];

        I2CUtil::read_bytes::<I2CX>(
            i2c,
            dev_id.addr(),
            BME680_COEFF_ADDR1,
            &mut coeff_array[0..(BME680_COEFF_ADDR1_LEN - 1)],
        ).unwrap();

        I2CUtil::read_bytes::<I2CX>(
            i2c,
            dev_id.addr(),
            BME680_COEFF_ADDR2,
            &mut coeff_array
                [BME680_COEFF_ADDR1_LEN..(BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN - 1)],
        ).unwrap();

        calib.par_t1 = ((coeff_array[34usize] as i32) << 8i32 | coeff_array[33usize] as i32) as u16;
        calib.par_t2 = ((coeff_array[2usize] as i32) << 8i32 | coeff_array[1usize] as i32) as i16;
        calib.par_t3 = coeff_array[3usize] as i8;
        calib.par_p1 = ((coeff_array[6usize] as i32) << 8i32 | coeff_array[5usize] as i32) as u16;
        calib.par_p2 = ((coeff_array[8usize] as i32) << 8i32 | coeff_array[7usize] as i32) as i16;
        calib.par_p3 = coeff_array[9usize] as i8;
        calib.par_p4 = ((coeff_array[12usize] as i32) << 8i32 | coeff_array[11usize] as i32) as i16;
        calib.par_p5 = ((coeff_array[14usize] as i32) << 8i32 | coeff_array[13usize] as i32) as i16;
        calib.par_p6 = coeff_array[16usize] as i8;
        calib.par_p7 = coeff_array[15usize] as i8;
        calib.par_p8 = ((coeff_array[20usize] as i32) << 8i32 | coeff_array[19usize] as i32) as i16;
        calib.par_p9 = ((coeff_array[22usize] as i32) << 8i32 | coeff_array[21usize] as i32) as i16;
        calib.par_p10 = coeff_array[23usize];
        calib.par_h1 =
            ((coeff_array[27usize] as i32) << 4i32 | coeff_array[26usize] as i32 & 0xfi32) as u16;
        calib.par_h2 =
            ((coeff_array[25usize] as i32) << 4i32 | coeff_array[26usize] as i32 >> 4i32) as u16;
        calib.par_h3 = coeff_array[28usize] as i8;
        calib.par_h4 = coeff_array[29usize] as i8;
        calib.par_h5 = coeff_array[30usize] as i8;
        calib.par_h6 = coeff_array[31usize];
        calib.par_h7 = coeff_array[32usize] as i8;
        calib.par_gh1 = coeff_array[37usize] as i8;
        calib.par_gh2 =
            ((coeff_array[36usize] as i32) << 8i32 | coeff_array[35usize] as i32) as i16;
        calib.par_gh3 = coeff_array[38usize] as i8;

        calib.res_heat_range =
            (I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RES_HEAT_RANGE_ADDR).unwrap()
                & 0x30)
                / 16;

        calib.res_heat_val =
            I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RES_HEAT_VAL_ADDR).unwrap() as i8;

        calib.range_sw_err =
            (I2CUtil::read_byte::<I2CX>(i2c, dev_id.addr(), BME680_ADDR_RANGE_SW_ERR_ADDR).unwrap()
                & BME680_RSERROR_MSK)
                / 16;

        Ok(calib)
    }

    fn set_gas_config(
        &mut self,
        gas_sett: GasSett,
    ) -> Result<(), <I2C as ErrorType>::Error> {
        if self.power_mode != PowerMode::ForcedMode {
            return Err(Error::DefinePwrMode);
        }

        // TODO check whether unwrap_or changes behaviour
        let reg: [(u8, u8); 2] = [
            (
                BME680_RES_HEAT0_ADDR,
                Calculation::heater_resistance(
                    &self.calib,
                    gas_sett.ambient_temperature,
                    gas_sett.heatr_temp.unwrap_or(0),
                ),
            ),
            (
                BME680_GAS_WAIT0_ADDR,
                Calculation::heater_duration(gas_sett.heatr_dur.unwrap_or_else(|| Duration::from_secs(0))),
            ),
        ];

        self.gas_sett.nb_conv = 0;
        self.bme680_set_regs(&reg)
    }

    fn get_gas_config(&mut self) -> Result<GasSett, <I2C as ErrorType>::Error> {
        let heatr_temp = Some(I2CUtil::read_byte(
            &mut self.i2c,
            self.dev_id.addr(),
            BME680_ADDR_SENS_CONF_START,
        )? as u16);

        let heatr_dur_ms = I2CUtil::read_byte(
            &mut self.i2c,
            self.dev_id.addr(),
            BME680_ADDR_GAS_CONF_START,
        )? as u64;

        let gas_sett = GasSett {
            heatr_temp,
            heatr_dur: Some(Duration::from_millis(heatr_dur_ms)),
            ..Default::default()
        };

        Ok(gas_sett)
    }

    /// Retrieve the current sensor information
    pub fn get_sensor_data(
        &mut self,
        delay: &mut D,
    ) -> Result<(FieldData, FieldDataCondition), <I2C as ErrorType>::Error> {
        let mut buff: [u8; BME680_FIELD_LENGTH] = [0; BME680_FIELD_LENGTH];

        debug!("Buf {:?}, len: {}", buff, buff.len());
        let mut data: FieldData = Default::default();

        const TRIES: u8 = 10;
        for _ in 0..TRIES {
            I2CUtil::read_bytes(
                &mut self.i2c,
                self.dev_id.addr(),
                BME680_FIELD0_ADDR,
                &mut buff,
            )?;

            debug!("Field data read {:?}, len: {}", buff, buff.len());

            data.status = buff[0] & BME680_NEW_DATA_MSK;
            data.gas_index = buff[0] & BME680_GAS_INDEX_MSK;
            data.meas_index = buff[1];

            let adc_pres = (buff[2] as u32).wrapping_mul(4096)
                | (buff[3] as u32).wrapping_mul(16)
                | (buff[4] as u32).wrapping_div(16);
            let adc_temp = (buff[5] as u32).wrapping_mul(4096)
                | (buff[6] as u32).wrapping_mul(16)
                | (buff[7] as u32).wrapping_div(16);
            let adc_hum = ((buff[8] as u32).wrapping_mul(256) | buff[9] as u32) as u16;
            let adc_gas_res =
                ((buff[13] as u32).wrapping_mul(4) | (buff[14] as u32).wrapping_div(64)) as u16;
            let gas_range = buff[14] & BME680_GAS_RANGE_MSK;

            data.status |= buff[14] & BME680_GASM_VALID_MSK;
            data.status |= buff[14] & BME680_HEAT_STAB_MSK;

            if data.status & BME680_NEW_DATA_MSK != 0 {
                let (temp, t_fine) =
                    Calculation::temperature(&self.calib, adc_temp, self.tph_sett.temperature_offset);
                debug!(
                    "adc_temp: {} adc_pres: {} adc_hum: {} adc_gas_res: {}, t_fine: {}",
                    adc_temp, adc_pres, adc_hum, adc_gas_res, t_fine
                );
                data.temperature = temp;
                data.pressure = Calculation::pressure(&self.calib, t_fine, adc_pres);
                data.humidity = Calculation::humidity(&self.calib, t_fine, adc_hum);
                data.gas_resistance =
                    Calculation::gas_resistance(&self.calib, adc_gas_res, gas_range);
                return Ok((data, FieldDataCondition::NewData));
            }

            delay.delay_ms(BME680_POLL_PERIOD_MS as u32);
        }
        Ok((data, FieldDataCondition::Unchanged))
    }
}
