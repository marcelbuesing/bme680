//! This crate is a pure Rust implementation for the BME680 environmental sensor.
//! The library can be used to read the gas, pressure, humidity and temperature sensors via I²C.
//!
//! The library uses the embedded-hal crate to abstract reading and writing via I²C.
//! In the examples you can find a demo how to use the library in Linux using the linux-embedded-hal crate (e.g. on a RPI).
//! ```no_run

//! use bme680::{Bme680, IIRFilterSize, OversamplingSetting, PowerMode, SettingsBuilder};
//! use core::time::Duration;
//! use embedded_hal::delay::DelayNs;
//! use linux_embedded_hal as hal;
//! use linux_embedded_hal::Delay;
//! use log::info;
//! use bme680::i2c::Address;
//!
//! // Please export RUST_LOG=info in order to see logs in the console.
//! fn main() -> Result<(), anyhow::Error>
//! {
//!     env_logger::init();
//!
//!     let i2c = hal::I2cdev::new("/dev/i2c-1").unwrap();
//!     let mut delayer = Delay {};
//!
//!     let mut dev = Bme680::init(i2c, &mut delayer, Address::Primary)?;
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
//!     let profile_dur = dev.get_profile_duration(&settings.0)?;
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
//!         let (data, _state) = dev.get_measurement(&mut delayer)?;
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
    DesiredSensorSettings, GasSettings, IIRFilterSize, OversamplingSetting, SensorSettings,
    Settings, SettingsBuilder, TemperatureSettings,
};
use core::cell::RefCell;

mod calculation;
pub mod i2c;
mod settings;

use crate::calculation::Calculation;
use crate::hal::delay::DelayNs;
use crate::hal::i2c::I2c;

use anyhow::anyhow;
use core::marker::PhantomData;
use core::ops::DerefMut;
use core::time::Duration;
use embedded_hal as hal;
use log::{debug, error, info};

use i2c::{Address, I2CUtility};
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

///
/// Power mode settings of the sensor.
///
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum PowerMode {
    SleepMode,
    ForcedMode,
}

impl PowerMode {
    fn from(power_mode: u8) -> Self {
        match power_mode {
            BME680_SLEEP_MODE => PowerMode::SleepMode,
            BME680_FORCED_MODE => PowerMode::ForcedMode,
            _ => panic!("Unknown power mode: {}", power_mode),
        }
    }

    /// Retrieves the power mode value.
    fn value(&self) -> u8 {
        match self {
            PowerMode::SleepMode => BME680_SLEEP_MODE,
            PowerMode::ForcedMode => BME680_FORCED_MODE,
        }
    }
}

/// Calibration data used during initialization
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
    measurement_index: u8,
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

/// Driver for the BME680 environmental sensor
#[repr(C)]
pub struct Bme680<I2C, D> {
    i2c_bus_handle: RefCell<I2C>,
    delay: PhantomData<D>,
    device_address: Address,
    calibration_data: CalibrationData,
    temperature_offset: f32,
    power_mode: PowerMode,
}

/// Checks if an u8 value is within the boundary and returns a Result containing an error
/// if the value is not within the boundary.
///
/// * `value` - The value to check for
/// * `value_name` - The name of the value
/// * `min` - The minimum boundary.
/// * `max` - The maximum boundary.
fn boundary_check_u8(
    value: Option<u8>,
    value_name: &'static str,
    min: u8,
    max: u8,
) -> Result<u8, anyhow::Error> {
    let value = value.ok_or(anyhow!("Boundary check failed for {}", value_name))?;

    if value < min {
        const MIN: &str = "Boundary check failure, value exceeds maximum";
        error!("{}, value name: {}", MIN, value_name);
        return Err(anyhow!(
            "Failed MIN={} boundary check for {}",
            MIN,
            value_name
        ));
    }

    if value > max {
        const MAX: &str = "Boundary check, value exceeds minimum";
        error!("{}, value name: {}", MAX, value_name);
        return Err(anyhow!(
            "Failed MAX={} boundary check for {}",
            MAX,
            value_name
        ));
    }
    Ok(value)
}

impl<I2C: I2c, D: DelayNs> Bme680<I2C, D>
where
    D: DelayNs,
    I2C: I2c,
{
    /// Sends the soft reset command to the chip.
    pub fn soft_reset(
        i2c_handle: &mut I2C,
        delay: &mut D,
        device_address: Address,
    ) -> Result<(), anyhow::Error> {
        let tmp_buff: [u8; 2] = [BME680_SOFT_RESET_ADDR, BME680_SOFT_RESET_CMD];
        I2CUtility::write_bytes(i2c_handle, device_address.addr(), &tmp_buff)?;
        delay.delay_ms(BME680_RESET_PERIOD as u32);
        Ok(())
    }

    /// Initializes the BME680 sensor with an I2C handle.
    pub fn init(
        mut i2c_handle: I2C,
        delay: &mut D,
        device_address: Address,
    ) -> Result<Bme680<I2C, D>, anyhow::Error> {
        Bme680::soft_reset(&mut i2c_handle, delay, device_address)?;

        debug!("Reading chip id");
        /* Soft reset to restore it to default values*/
        let chip_id = I2CUtility::read_byte::<I2C>(
            &mut i2c_handle,
            device_address.addr(),
            BME680_CHIP_ID_ADDR,
        )?;
        debug!("Chip id: {}", chip_id);

        if chip_id == BME680_CHIP_ID {
            debug!("Reading calibration data");
            let calibration_data =
                Bme680::<I2C, D>::get_calibration_data::<I2C>(&mut i2c_handle, device_address)?;
            debug!("Calibration data {:?}", calibration_data);
            let device = Bme680 {
                i2c_bus_handle: RefCell::new(i2c_handle),
                delay: PhantomData,
                device_address,
                calibration_data,
                temperature_offset: 0.0,
                power_mode: PowerMode::ForcedMode,
            };
            info!("Finished device init");
            Ok(device)
        } else {
            error!("Device does not match chip id {}", BME680_CHIP_ID);
            Err(anyhow!("Device address not found"))
        }
    }

    /// Sets the sensor registers.
    fn bme680_set_registers(&mut self, registers: &[(u8, u8)]) -> Result<(), anyhow::Error> {
        if registers.is_empty() || registers.len() > (BME680_TMP_BUFFER_LENGTH / 2) {
            return Err(anyhow!("Invalid register length!"));
        }

        for (register_address, register_data) in registers {
            let buffer: [u8; 2] = [*register_address, *register_data];
            debug!(
                "Setting register reg: {:?} buffer: {:?}",
                register_address, buffer
            );

            I2CUtility::write_bytes(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                &buffer,
            )?;
        }

        Ok(())
    }

    /// Set the settings to be used during the sensor measurements
    pub fn set_sensor_settings(
        &mut self,
        delay: &mut D,
        settings: Settings,
    ) -> Result<(), anyhow::Error> {
        let (sensor_settings, desired_settings) = settings;
        let tph_sett = sensor_settings.temperature_settings;
        let gas_sett = sensor_settings.gas_settings;

        self.temperature_offset = sensor_settings
            .temperature_settings
            .temperature_offset
            .unwrap_or(0f32);

        let mut reg: [(u8, u8); BME680_REG_BUFFER_LENGTH] = [(0, 0); BME680_REG_BUFFER_LENGTH];
        let intended_power_mode = self.power_mode;

        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            debug!("GAS_MEAS_SEL: true");
            self.set_gas_settings(gas_sett)?;
        }

        let power_mode = self.power_mode;
        self.set_sensor_mode(delay, power_mode)?;

        let mut element_index = 0;
        // Selecting the filter
        if desired_settings.contains(DesiredSensorSettings::FILTER_SIZE_SEL) {
            let mut data = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_CONF_ODR_FILT_ADDR,
            )?;

            debug!("FILTER_SEL: true");
            data = (data as i32 & !0x1ci32
                | (tph_sett.filter_size.unwrap_or(IIRFilterSize::Size0) as i32) << 2i32 & 0x1ci32)
                as u8;
            reg[element_index] = (BME680_CONF_ODR_FILT_ADDR, data);
            element_index += 1;
        }

        if desired_settings.contains(DesiredSensorSettings::HUMIDITY_CONTROL_SEL) {
            debug!("HCNTRL_SEL: true");
            let gas_sett_heatr_ctrl =
                boundary_check_u8(gas_sett.heater_control, "GasSett.heatr_ctrl", 0x0u8, 0x8u8)?;
            let mut data = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
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
            let mut data = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_CONF_T_P_MODE_ADDR,
            )?;

            if desired_settings.contains(DesiredSensorSettings::OST_SEL) {
                debug!("OST_SEL: true");
                let tph_sett_os_temp = boundary_check_u8(
                    tph_sett.temperature_oversampling.map(|x| x as u8),
                    "TphSett.os_temp",
                    0,
                    5,
                )?;
                data = (data as i32 & !0xe0i32 | (tph_sett_os_temp as i32) << 5i32 & 0xe0i32) as u8;
            }

            if desired_settings.contains(DesiredSensorSettings::OSP_SEL) {
                debug!("OSP_SEL: true");
                let tph_sett_os_pres = tph_sett
                    .temperature_oversampling
                    .unwrap_or(OversamplingSetting::OS1x);
                data = (data as i32 & !0x1ci32 | (tph_sett_os_pres as i32) << 2i32 & 0x1ci32) as u8;
            }
            reg[element_index] = (BME680_CONF_T_P_MODE_ADDR, data);
            element_index += 1;
        }

        // Selecting humidity oversampling for the sensor
        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            debug!("OSH_SEL: true");
            let tph_sett_os_hum = boundary_check_u8(
                tph_sett.humidity_oversampling.map(|x| x as u8),
                "TphSett.os_hum",
                0,
                5,
            )?;
            let mut data = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_CONF_OS_H_ADDR,
            )?;
            data = (data as i32 & !0x7i32 | tph_sett_os_hum as i32 & 0x7i32) as u8;
            reg[element_index] = (BME680_CONF_OS_H_ADDR, data);
            element_index += 1;
        }

        // Selecting the runGas and NB conversion settings for the sensor
        if desired_settings
            .contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL)
        {
            let mut data = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_CONF_ODR_RUN_GAS_NBC_ADDR,
            )?;

            if desired_settings.contains(DesiredSensorSettings::RUN_GAS_SEL) {
                debug!("RUN_GAS_SEL: true");
                data = (data as i32 & !0x10i32
                    | (gas_sett.enable_gas_measurement as i32) << 4i32 & 0x10i32)
                    as u8;
            }

            if desired_settings.contains(DesiredSensorSettings::NBCONV_SEL) {
                debug!("NBCONV_SEL: true");
                let gas_sett_nb_conv =
                    boundary_check_u8(Some(gas_sett.nb_conv), "GasSett.nb_conv", 0, 10)?;
                data = (data as i32 & !0xfi32 | gas_sett_nb_conv as i32 & 0xfi32) as u8;
            }

            reg[element_index] = (BME680_CONF_ODR_RUN_GAS_NBC_ADDR, data);
            element_index += 1;
        }

        self.bme680_set_registers(&reg[0..element_index])?;

        // Restore previous intended power mode
        self.power_mode = intended_power_mode;
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
    ) -> Result<SensorSettings, anyhow::Error> {
        let reg_addr: u8 = 0x70u8;
        let mut data_array: [u8; BME680_REG_BUFFER_LENGTH] = [0; BME680_REG_BUFFER_LENGTH];
        let mut sensor_settings: SensorSettings = Default::default();

        I2CUtility::read_bytes(
            self.i2c_bus_handle.borrow_mut().deref_mut(),
            self.device_address.addr(),
            reg_addr,
            &mut data_array,
        )?;

        if desired_settings.contains(DesiredSensorSettings::GAS_MEAS_SEL) {
            sensor_settings.gas_settings = self.get_gas_settings()?;
        }

        if desired_settings.contains(DesiredSensorSettings::FILTER_SIZE_SEL) {
            sensor_settings.temperature_settings.filter_size = Some(IIRFilterSize::from_u8(
                ((data_array[5usize] as i32 & 0x1ci32) >> 2i32) as u8,
            ));
        }

        if desired_settings
            .contains(DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL)
        {
            let os_temp: u8 = ((data_array[4usize] as i32 & 0xe0i32) >> 5i32) as u8;
            let os_pres: u8 = ((data_array[4usize] as i32 & 0x1ci32) >> 2i32) as u8;
            sensor_settings
                .temperature_settings
                .temperature_oversampling = Some(OversamplingSetting::from_u8(os_temp));
            sensor_settings.temperature_settings.pressure_oversampling =
                Some(OversamplingSetting::from_u8(os_pres));
        }

        if desired_settings.contains(DesiredSensorSettings::OSH_SEL) {
            let os_hum: u8 = (data_array[2usize] as i32 & 0x7i32) as u8;
            sensor_settings.temperature_settings.humidity_oversampling =
                Some(OversamplingSetting::from_u8(os_hum));
        }

        if desired_settings.contains(DesiredSensorSettings::HUMIDITY_CONTROL_SEL) {
            sensor_settings.gas_settings.heater_control =
                Some((data_array[0usize] as i32 & 0x8i32) as u8);
        }

        if desired_settings
            .contains(DesiredSensorSettings::RUN_GAS_SEL | DesiredSensorSettings::NBCONV_SEL)
        {
            sensor_settings.gas_settings.nb_conv = (data_array[1usize] as i32 & 0xfi32) as u8;
            sensor_settings.gas_settings.enable_gas_measurement =
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
    ) -> Result<(), anyhow::Error> {
        let mut power_mode_byte: u8;
        let mut power_mode: PowerMode;

        // Call repeatedly until in sleep
        loop {
            power_mode_byte = I2CUtility::read_byte(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_CONF_T_P_MODE_ADDR,
            )?;

            // Put to sleep before changing mode
            power_mode = PowerMode::from(power_mode_byte & BME680_MODE_MSK);

            debug!("Current power mode: {:?}", power_mode);

            if power_mode != PowerMode::SleepMode {
                // Set to sleep
                power_mode_byte &= !BME680_MODE_MSK;
                debug!("Setting to sleep tmp_pow_mode: {}", power_mode_byte);
                self.bme680_set_registers(&[(BME680_CONF_T_P_MODE_ADDR, power_mode_byte)])?;
                delay.delay_ms(BME680_POLL_PERIOD_MS as u32);
            } else {
                break;
            }
        }

        // Already in sleep
        if target_power_mode != PowerMode::SleepMode {
            power_mode_byte = power_mode_byte & !BME680_MODE_MSK | target_power_mode.value();
            debug!("Already in sleep Target power mode: {}", power_mode_byte);
            self.bme680_set_registers(&[(BME680_CONF_T_P_MODE_ADDR, power_mode_byte)])?;
        }
        Ok(())
    }

    /// Retrieve current sensor power mode via registers
    pub fn get_sensor_mode(&mut self) -> Result<PowerMode, anyhow::Error> {
        let registers = I2CUtility::read_byte(
            self.i2c_bus_handle.borrow_mut().deref_mut(),
            self.device_address.addr(),
            BME680_CONF_T_P_MODE_ADDR,
        )?;
        let mode = registers & BME680_MODE_MSK;
        Ok(PowerMode::from(mode))
    }

    pub fn get_profile_duration(
        &self,
        sensor_settings: &SensorSettings,
    ) -> Result<Duration, anyhow::Error> {
        let os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        let mut measurement_cycles = os_to_meas_cycles[sensor_settings
            .temperature_settings
            .temperature_oversampling
            .unwrap_or(OversamplingSetting::OSNone)
            as usize] as u32;
        measurement_cycles = measurement_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .temperature_settings
                .pressure_oversampling
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        measurement_cycles = measurement_cycles.wrapping_add(
            os_to_meas_cycles[sensor_settings
                .temperature_settings
                .humidity_oversampling
                .unwrap_or(OversamplingSetting::OSNone) as usize] as u32,
        );
        let mut temperature_duration = measurement_cycles.wrapping_mul(1963u32);
        temperature_duration = temperature_duration.wrapping_add(477u32.wrapping_mul(4u32));
        temperature_duration = temperature_duration.wrapping_add(477u32.wrapping_mul(5u32));
        temperature_duration = temperature_duration.wrapping_add(500u32);
        temperature_duration = temperature_duration.wrapping_div(1000u32);
        temperature_duration = temperature_duration.wrapping_add(1u32);
        let mut duration = Duration::from_millis(temperature_duration as u64);
        if sensor_settings.gas_settings.enable_gas_measurement {
            duration += sensor_settings
                .gas_settings
                .heater_duration
                .unwrap_or_default();
        }
        Ok(duration)
    }

    fn get_calibration_data<I2CX>(
        i2c: &mut I2CX,
        device_address: Address,
    ) -> Result<CalibrationData, anyhow::Error>
    where
        I2CX: I2c,
    {
        let mut calibration_data: CalibrationData = Default::default();

        let mut coefficients_array: [u8; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN] =
            [0; BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN];

        I2CUtility::read_bytes::<I2CX>(
            i2c,
            device_address.addr(),
            BME680_COEFF_ADDR1,
            &mut coefficients_array[0..(BME680_COEFF_ADDR1_LEN - 1)],
        )
        .map_err(|_e| {
            anyhow!(
                "Failed to get calibration data from device: {}",
                device_address
            )
        })?;

        I2CUtility::read_bytes::<I2CX>(
            i2c,
            device_address.addr(),
            BME680_COEFF_ADDR2,
            &mut coefficients_array
                [BME680_COEFF_ADDR1_LEN..(BME680_COEFF_ADDR1_LEN + BME680_COEFF_ADDR2_LEN - 1)],
        )
        .map_err(|_e| {
            anyhow!(
                "Failed to get calibration data from device: {}",
                device_address
            )
        })?;

        calibration_data.par_t1 = ((coefficients_array[34usize] as i32) << 8i32
            | coefficients_array[33usize] as i32) as u16;
        calibration_data.par_t2 = ((coefficients_array[2usize] as i32) << 8i32
            | coefficients_array[1usize] as i32) as i16;
        calibration_data.par_t3 = coefficients_array[3usize] as i8;
        calibration_data.par_p1 = ((coefficients_array[6usize] as i32) << 8i32
            | coefficients_array[5usize] as i32) as u16;
        calibration_data.par_p2 = ((coefficients_array[8usize] as i32) << 8i32
            | coefficients_array[7usize] as i32) as i16;
        calibration_data.par_p3 = coefficients_array[9usize] as i8;
        calibration_data.par_p4 = ((coefficients_array[12usize] as i32) << 8i32
            | coefficients_array[11usize] as i32) as i16;
        calibration_data.par_p5 = ((coefficients_array[14usize] as i32) << 8i32
            | coefficients_array[13usize] as i32) as i16;
        calibration_data.par_p6 = coefficients_array[16usize] as i8;
        calibration_data.par_p7 = coefficients_array[15usize] as i8;
        calibration_data.par_p8 = ((coefficients_array[20usize] as i32) << 8i32
            | coefficients_array[19usize] as i32) as i16;
        calibration_data.par_p9 = ((coefficients_array[22usize] as i32) << 8i32
            | coefficients_array[21usize] as i32) as i16;
        calibration_data.par_p10 = coefficients_array[23usize];
        calibration_data.par_h1 = ((coefficients_array[27usize] as i32) << 4i32
            | coefficients_array[26usize] as i32 & 0xfi32) as u16;
        calibration_data.par_h2 = ((coefficients_array[25usize] as i32) << 4i32
            | coefficients_array[26usize] as i32 >> 4i32) as u16;
        calibration_data.par_h3 = coefficients_array[28usize] as i8;
        calibration_data.par_h4 = coefficients_array[29usize] as i8;
        calibration_data.par_h5 = coefficients_array[30usize] as i8;
        calibration_data.par_h6 = coefficients_array[31usize];
        calibration_data.par_h7 = coefficients_array[32usize] as i8;
        calibration_data.par_gh1 = coefficients_array[37usize] as i8;
        calibration_data.par_gh2 = ((coefficients_array[36usize] as i32) << 8i32
            | coefficients_array[35usize] as i32) as i16;
        calibration_data.par_gh3 = coefficients_array[38usize] as i8;

        calibration_data.res_heat_range = (I2CUtility::read_byte::<I2CX>(
            i2c,
            device_address.addr(),
            BME680_ADDR_RES_HEAT_RANGE_ADDR,
        )
        .map_err(|_e| anyhow!("Failed to read from register BME680_ADDR_RES_HEAT_RANGE_ADDR"))?
            & 0x30)
            / 16;

        calibration_data.res_heat_val = I2CUtility::read_byte::<I2CX>(
            i2c,
            device_address.addr(),
            BME680_ADDR_RES_HEAT_VAL_ADDR,
        )
        .map_err(|_e| anyhow!("Failed to read from register BME680_ADDR_RES_HEAT_VAL_ADDR"))?
            as i8;

        calibration_data.range_sw_err = (I2CUtility::read_byte::<I2CX>(
            i2c,
            device_address.addr(),
            BME680_ADDR_RANGE_SW_ERR_ADDR,
        )
        .map_err(|_e| anyhow!("Failed to read from register BME680_ADDR_RANGE_SW_ERR_ADDR"))?
            & BME680_RSERROR_MSK)
            / 16;

        Ok(calibration_data)
    }

    fn set_gas_settings(&mut self, gas_settings: GasSettings) -> Result<(), anyhow::Error> {
        if self.power_mode != PowerMode::ForcedMode {
            return Err(anyhow!("Current power mode is not forced"));
        }

        let reg: [(u8, u8); 2] = [
            (
                BME680_RES_HEAT0_ADDR,
                Calculation::heater_resistance(
                    &self.calibration_data,
                    gas_settings.ambient_temperature,
                    gas_settings.heater_temperature.unwrap_or(0),
                ),
            ),
            (
                BME680_GAS_WAIT0_ADDR,
                Calculation::heater_duration(
                    gas_settings
                        .heater_duration
                        .unwrap_or_else(|| Duration::from_secs(0)),
                ),
            ),
        ];

        self.bme680_set_registers(&reg)
    }

    fn get_gas_settings(&mut self) -> Result<GasSettings, anyhow::Error> {
        let heater_temperature = Some(I2CUtility::read_byte(
            self.i2c_bus_handle.borrow_mut().deref_mut(),
            self.device_address.addr(),
            BME680_ADDR_SENS_CONF_START,
        )? as u16);

        let heater_duration_ms = I2CUtility::read_byte(
            self.i2c_bus_handle.borrow_mut().deref_mut(),
            self.device_address.addr(),
            BME680_ADDR_GAS_CONF_START,
        )? as u64;

        let gas_sett = GasSettings {
            heater_temperature,
            heater_duration: Some(Duration::from_millis(heater_duration_ms)),
            ..Default::default()
        };

        Ok(gas_sett)
    }

    /// Retrieve the current sensor measurement.
    pub fn get_measurement(
        &mut self,
        delay: &mut D,
    ) -> Result<(FieldData, FieldDataCondition), anyhow::Error> {
        let mut buffer: [u8; BME680_FIELD_LENGTH] = [0; BME680_FIELD_LENGTH];

        debug!("Buf {:?}, len: {}", buffer, buffer.len());
        let mut data: FieldData = Default::default();

        const TRIES: u8 = 10;
        for _ in 0..TRIES {
            I2CUtility::read_bytes(
                self.i2c_bus_handle.borrow_mut().deref_mut(),
                self.device_address.addr(),
                BME680_FIELD0_ADDR,
                &mut buffer,
            )?;

            debug!("Field data read {:?}, len: {}", buffer, buffer.len());

            data.status = buffer[0] & BME680_NEW_DATA_MSK;
            data.gas_index = buffer[0] & BME680_GAS_INDEX_MSK;
            data.measurement_index = buffer[1];

            let adc_pressure = (buffer[2] as u32).wrapping_mul(4096)
                | (buffer[3] as u32).wrapping_mul(16)
                | (buffer[4] as u32).wrapping_div(16);
            let adc_temperature = (buffer[5] as u32).wrapping_mul(4096)
                | (buffer[6] as u32).wrapping_mul(16)
                | (buffer[7] as u32).wrapping_div(16);
            let adc_humidity = ((buffer[8] as u32).wrapping_mul(256) | buffer[9] as u32) as u16;
            let adc_gas_resistance =
                ((buffer[13] as u32).wrapping_mul(4) | (buffer[14] as u32).wrapping_div(64)) as u16;
            let gas_range = buffer[14] & BME680_GAS_RANGE_MSK;

            data.status |= buffer[14] & BME680_GASM_VALID_MSK;
            data.status |= buffer[14] & BME680_HEAT_STAB_MSK;

            if data.status & BME680_NEW_DATA_MSK != 0 {
                let (temp, t_fine) = Calculation::temperature(
                    &self.calibration_data,
                    adc_temperature,
                    Some(self.temperature_offset),
                );
                debug!(
                    "adc_temp: {} adc_pres: {} adc_hum: {} adc_gas_res: {}, t_fine: {}",
                    adc_temperature, adc_pressure, adc_humidity, adc_gas_resistance, t_fine
                );
                data.temperature = temp;
                data.pressure = Calculation::pressure(&self.calibration_data, t_fine, adc_pressure);
                data.humidity = Calculation::humidity(&self.calibration_data, t_fine, adc_humidity);
                data.gas_resistance = Calculation::gas_resistance(
                    &self.calibration_data,
                    adc_gas_resistance,
                    gas_range,
                );
                return Ok((data, FieldDataCondition::NewData));
            }

            delay.delay_ms(BME680_POLL_PERIOD_MS as u32);
        }
        Ok((data, FieldDataCondition::Unchanged))
    }
}
