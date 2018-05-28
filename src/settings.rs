use core::time::Duration;

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
    // TODO replace with TryFrom once stabilized
    pub fn from_u8(os: u8) -> OversamplingSetting {
        match os {
            0 => OversamplingSetting::OSNone,
            1 => OversamplingSetting::OS1x,
            2 => OversamplingSetting::OS2x,
            3 => OversamplingSetting::OS4x,
            4 => OversamplingSetting::OS8x,
            5 => OversamplingSetting::OS16x,
            _ => panic!("Unknown oversampling setting: {}", os),
        }
    }
}

#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct TphSett {
    pub os_hum: Option<OversamplingSetting>,
    pub os_temp: Option<OversamplingSetting>,
    pub os_pres: Option<OversamplingSetting>,
    pub filter: Option<u8>,
}

impl Clone for TphSett {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Debug, Default, Copy)]
#[repr(C)]
pub struct GasSett {
    pub nb_conv: u8,
    /// Heater control
    pub heatr_ctrl: Option<u8>,
    /// Enable measurement of gas, disabled by default
    pub run_gas_measurement: bool,
    /// Heater temperature
    pub heatr_temp: Option<u16>,
    /// Profile duration
    pub heatr_dur: Option<Duration>,
}

impl Clone for GasSett {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Debug, Default, Copy)]
pub struct SensorSettings {
    /// Gas settings
    pub gas_sett: GasSett,
    /// Temperature settings
    pub tph_sett: TphSett,
}

impl Clone for SensorSettings {
    fn clone(&self) -> Self {
        *self
    }
}

bitflags! {
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
