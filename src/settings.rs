use bitflags::bitflags;
use core::time::Duration;

/// Over-sampling settings
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum OversamplingSetting {
    OSNone = 0,
    OS1x = 1,
    OS2x = 2,
    OS4x = 3,
    OS8x = 4,
    OS16x = 5,
}

impl OversamplingSetting {
    pub fn from_u8(value: u8) -> OversamplingSetting {
        match value {
            0 => OversamplingSetting::OSNone,
            1 => OversamplingSetting::OS1x,
            2 => OversamplingSetting::OS2x,
            3 => OversamplingSetting::OS4x,
            4 => OversamplingSetting::OS8x,
            5 => OversamplingSetting::OS16x,
            _ => panic!("Unknown oversampling setting: {}", value),
        }
    }
}

/// IIR filter settings
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum IIRFilterSize {
    Size0 = 0,
    Size1 = 1,
    Size3 = 2,
    Size7 = 3,
    Size15 = 4,
    Size31 = 5,
    Size63 = 6,
    Size127 = 7,
}

impl IIRFilterSize {
    pub fn from_u8(value: u8) -> IIRFilterSize {
        match value {
            0 => IIRFilterSize::Size0,
            1 => IIRFilterSize::Size1,
            2 => IIRFilterSize::Size3,
            3 => IIRFilterSize::Size7,
            4 => IIRFilterSize::Size15,
            5 => IIRFilterSize::Size31,
            6 => IIRFilterSize::Size63,
            7 => IIRFilterSize::Size127,
            _ => panic!("Unknown IIRFilterSize: {}", value),
        }
    }
}

/// Temperature settings
#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct TemperatureSettings {
    /// Humidity oversampling
    pub humidity_oversampling: Option<OversamplingSetting>,
    /// Temperature oversampling
    pub temperature_oversampling: Option<OversamplingSetting>,
    /// Pressure oversampling
    pub pressure_oversampling: Option<OversamplingSetting>,
    /// Filter coefficient
    pub filter_size: Option<IIRFilterSize>,
    /// If set, the temperature t_fine will be increased by the given value in celsius.
    pub temperature_offset: Option<f32>,
}

impl Clone for TemperatureSettings {
    fn clone(&self) -> Self {
        *self
    }
}

/// Gas measurement settings
#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct GasSettings {
    /// nb_conv is used to select heater set-points of the sensor.
    pub nb_conv: u8,
    /// Heater control
    pub heater_control: Option<u8>,
    /// Enable measurement of gas, disabled by default
    pub enable_gas_measurement: bool,
    /// The heater temperature
    pub heater_temperature: Option<u16>,
    /// The Heating duration
    pub heater_duration: Option<Duration>,
    /// The ambient temperature.
    pub ambient_temperature: i8,
}

impl Clone for GasSettings {
    fn clone(&self) -> Self {
        *self
    }
}

/// Stores gas and temperature settings
#[derive(Debug, Default, Copy)]
pub struct SensorSettings {
    /// Gas settings
    pub gas_settings: GasSettings,
    /// Temperature settings
    pub temperature_settings: TemperatureSettings,
}

impl Clone for SensorSettings {
    fn clone(&self) -> Self {
        *self
    }
}

bitflags! {
    /// Flags that determine what settings are to be set and what settings are to be read.
    /// Use the `SettingsBuilder` to initialize an instance when setting the settings.
    #[derive(Default)]
    pub struct DesiredSensorSettings: u16 {
        /// To set temperature oversampling
        const OST_SEL = 1;
        /// To set pressure oversampling.
        const OSP_SEL = 2;
        /// To set humidity oversampling.
        const OSH_SEL = 4;
        /// To set gas measurement setting.
        const GAS_MEAS_SEL = 8;
        /// To set filter setting.
        const FILTER_SIZE_SEL = 16;
        /// To set humidity control setting.
        const HUMIDITY_CONTROL_SEL = 32;
        /// To set run gas setting.
        const RUN_GAS_SEL = 64;
        /// To set NB conversion setting.
        const NBCONV_SEL = 128;
        /// To set all gas sensor related settings
        const GAS_SENSOR_SEL = Self::GAS_MEAS_SEL.bits | Self::RUN_GAS_SEL.bits | Self::NBCONV_SEL.bits;
    }
}

///
/// Builder to construct the desired settings
///
/// # Example
/// ```
/// use bme680::{IIRFilterSize, OversamplingSetting, SettingsBuilder};
/// use std::time::Duration;
/// let settings = SettingsBuilder::default()
///     .with_humidity_oversampling(OversamplingSetting::OS2x)
///     .with_pressure_oversampling(OversamplingSetting::OS4x)
///     .with_temperature_oversampling(OversamplingSetting::OS8x)
///     .with_temperature_filter(IIRFilterSize::Size3)
///     .with_gas_measurement(Duration::from_millis(1500), 320, 25)
///     .with_temperature_offset(-4.25)
///     .with_run_gas(true)
///     .build();
/// ```
#[derive(Default)]
pub struct SettingsBuilder {
    desired_settings: DesiredSensorSettings,
    sensor_settings: SensorSettings,
}

/// Tuple of desired sensor settings flags and sensor settings
pub type Settings = (SensorSettings, DesiredSensorSettings);

impl SettingsBuilder {
    /// Constructs a new instance of the SettingsBuilder.
    pub fn new() -> SettingsBuilder {
        SettingsBuilder::default()
    }

    /// With temperature filter.
    pub fn with_temperature_filter(mut self, filter_size: IIRFilterSize) -> SettingsBuilder {
        self.sensor_settings.temperature_settings.filter_size = Some(filter_size);
        self.desired_settings |= DesiredSensorSettings::FILTER_SIZE_SEL;
        self
    }

    /// With gas heater control.
    pub fn with_gas_heater_control(mut self, heater_control: u8) -> SettingsBuilder {
        self.sensor_settings.gas_settings.heater_control = Some(heater_control);
        self.desired_settings |= DesiredSensorSettings::HUMIDITY_CONTROL_SEL;
        self
    }

    /// With temperature oversampling
    pub fn with_temperature_oversampling(
        mut self,
        temperature_oversampling: OversamplingSetting,
    ) -> SettingsBuilder {
        self.sensor_settings
            .temperature_settings
            .temperature_oversampling = Some(temperature_oversampling);
        self.desired_settings |= DesiredSensorSettings::OST_SEL;
        self
    }

    /// With pressure oversampling.
    pub fn with_pressure_oversampling(
        mut self,
        pressure_oversampling: OversamplingSetting,
    ) -> SettingsBuilder {
        self.sensor_settings
            .temperature_settings
            .pressure_oversampling = Some(pressure_oversampling);
        self.desired_settings |= DesiredSensorSettings::OSP_SEL;
        self
    }

    /// With humidity oversampling.
    pub fn with_humidity_oversampling(
        mut self,
        humidity_oversampling: OversamplingSetting,
    ) -> SettingsBuilder {
        self.sensor_settings
            .temperature_settings
            .humidity_oversampling = Some(humidity_oversampling);
        self.desired_settings |= DesiredSensorSettings::OSH_SEL;
        self
    }

    /// With gas measurement.
    pub fn with_gas_measurement(
        mut self,
        heater_duration: Duration,
        heater_temperature: u16,
        ambient_temperature: i8,
    ) -> SettingsBuilder {
        self.sensor_settings.gas_settings.heater_duration = Some(heater_duration);
        self.sensor_settings.gas_settings.heater_temperature = Some(heater_temperature);
        self.sensor_settings.gas_settings.ambient_temperature = ambient_temperature;
        self.desired_settings |= DesiredSensorSettings::GAS_SENSOR_SEL;
        self
    }

    /// With nb_conv.
    pub fn with_nb_conv(mut self, nb_conv: u8) -> SettingsBuilder {
        self.sensor_settings.gas_settings.nb_conv = nb_conv;
        self.desired_settings |= DesiredSensorSettings::GAS_SENSOR_SEL;
        self
    }

    /// With run gas.
    pub fn with_run_gas(mut self, run_gas: bool) -> SettingsBuilder {
        self.sensor_settings.gas_settings.enable_gas_measurement = run_gas;
        self.desired_settings |= DesiredSensorSettings::GAS_SENSOR_SEL;
        self
    }

    /// With temperature offset in Celsius, e.g. 4, -8, 1.25
    pub fn with_temperature_offset(mut self, offset: f32) -> SettingsBuilder {
        self.sensor_settings.temperature_settings.temperature_offset = Some(offset);
        self
    }

    /// Builds the settings object
    pub fn build(self) -> Settings {
        (self.sensor_settings, self.desired_settings)
    }
}
