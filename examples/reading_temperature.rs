extern crate embedded_hal;
extern crate env_logger;
extern crate linux_embedded_hal as hal;
#[macro_use]
extern crate log;
extern crate bme680_rs;

use bme680_rs::*;
use embedded_hal::blocking::i2c;
use hal::*;
use std::result;
use std::thread;
use std::time::Duration;

fn main() -> result::Result<
    (),
    Bme680Error<<hal::I2cdev as i2c::Read>::Error, <hal::I2cdev as i2c::Write>::Error>,
> {
    env_logger::init();

    let i2c = I2cdev::new("/dev/i2c-1").unwrap();

    let mut dev = Bme680_dev::init(i2c, Delay {}, 0x76, 25)?;

    let mut sensor_settings: SensorSettings = Default::default();

    sensor_settings.tph_sett.os_hum = Some(OversamplingSetting::OS2x);
    sensor_settings.tph_sett.os_pres = Some(OversamplingSetting::OS4x);
    sensor_settings.tph_sett.os_temp = Some(OversamplingSetting::OS8x);
    sensor_settings.tph_sett.filter = Some(2);

    sensor_settings.gas_sett.run_gas = Some(0x01);
    sensor_settings.gas_sett.heatr_dur = Some(1500);
    sensor_settings.gas_sett.heatr_temp = Some(320);

    let settings_sel = DesiredSensorSettings::OST_SEL | DesiredSensorSettings::OSP_SEL
        | DesiredSensorSettings::OSH_SEL
        | DesiredSensorSettings::GAS_SENSOR_SEL;

    let profile_dur = dev.get_profile_dur(&sensor_settings)?;
    info!("Duration {}", profile_dur);
    info!("Setting sensor settings");
    dev.set_sensor_settings(settings_sel, &sensor_settings)?;
    info!("Setting forced power modes");
    dev.set_sensor_mode(PowerMode::ForcedMode)?;

    let sensor_settings = dev.get_sensor_settings(settings_sel);
    info!("Sensor settings: {:?}", sensor_settings);

    loop {
        thread::sleep(Duration::from_millis(5000));
        let power_mode = dev.get_sensor_mode();
        info!("Sensor power mode: {:?}", power_mode);
        info!("Setting forced power modes");
        dev.set_sensor_mode(PowerMode::ForcedMode)?;
        info!("Retrieving sensor data");
        let (data, state) = dev.get_sensor_data()?;
        info!("Sensor Data {:?}", data);
        info!("Temperature {}°C", data.temperature_celsius());
        info!("Pressure {}hPa", data.pressure_hpa());
        info!("Humidity {}%", data.humidity_percent());
    }
    Ok(())
}
