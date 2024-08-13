#![no_std]

use bme680::*;
use core::result;
use core::time::Duration;
use embedded_hal::delay::DelayNs;
use linux_embedded_hal as hal;
use linux_embedded_hal::Delay;
use log::info;

fn main() -> result::Result<(), ()> {
    env_logger::init();

    let i2c = hal::I2cdev::new("/dev/i2c-1").unwrap();
    let mut delayer = Delay {};

    let mut dev = Bme680::init(i2c, &mut delayer, I2CAddress::Primary).map_err(|e| {
        log::error!("Error at bme680 init {e:?}");
    })?;

    let mut delay = Delay {};

    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_temperature_offset(-2.2)
        .with_run_gas(true)
        .build();

    let profile_dur = dev.get_profile_dur(&settings.0).map_err(|e| {
        log::error!("Unable to get profile dur {e:?}");
    })?;
    info!("Profile duration {:?}", profile_dur);
    info!("Setting sensor settings");
    dev.set_sensor_settings(&mut delayer, settings)
        .map_err(|e| {
            log::error!("Unable to apply sensor settings {e:?}");
        })?;
    info!("Setting forced power modes");
    dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)
        .map_err(|e| {
            log::error!("Unable to set sensor mode {e:?}");
        })?;

    let sensor_settings = dev.get_sensor_settings(settings.1);
    info!("Sensor settings: {:?}", sensor_settings);

    loop {
        delay.delay_ms(5000u32);
        let power_mode = dev.get_sensor_mode();
        info!("Sensor power mode: {:?}", power_mode);
        info!("Setting forced power modes");
        dev.set_sensor_mode(&mut delayer, PowerMode::ForcedMode)
            .map_err(|e| {
                log::error!("Unable to set sensor mode {e:?}");
            })?;
        info!("Retrieving sensor data");
        let (data, _state) = dev.get_sensor_data(&mut delayer).map_err(|e| {
            log::error!("Unable to get sensor data {e:?}");
        })?;
        info!("Sensor Data {:?}", data);
        info!("Temperature {}°C", data.temperature_celsius());
        info!("Pressure {}hPa", data.pressure_hpa());
        info!("Humidity {}%", data.humidity_percent());
        info!("Gas Resistence {}Ω", data.gas_resistance_ohm());
    }
}
