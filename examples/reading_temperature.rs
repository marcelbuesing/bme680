//extern crate i2cdev;
extern crate embedded_hal;
extern crate linux_embedded_hal as hal;
extern crate bme680_rs;

//use i2cdev::linux::LinuxI2CDevice;
use bme680_rs::*;
use std::io;
use hal::*;
use std::thread;
use std::result;
use std::time::Duration;

fn main() -> result::Result<(), Bme680Error>{

    let i2c = I2cdev::new("/dev/i2c-1").unwrap();

    let mut dev = Bme680_dev::init(i2c, Delay{}, 0x76, 25)?;

    let mut sensor_settings: SensorSettings = Default::default();

    sensor_settings.tph_sett.os_hum = Some(BME680_OS_1X);
    sensor_settings.tph_sett.os_pres = Some(BME680_OS_16X);
    sensor_settings.tph_sett.os_temp = Some(BME680_OS_2X);

    sensor_settings.gas_sett.run_gas = Some(0x01);
    sensor_settings.gas_sett.heatr_dur = Some(2000);

    let settings_sel =
        DesiredSensorSettings::OST_SEL |
        DesiredSensorSettings::OSP_SEL |
        DesiredSensorSettings::OSH_SEL |
        DesiredSensorSettings::GAS_SENSOR_SEL;

    let profile_dur = dev.get_profile_dur(&sensor_settings)?;
    println!("Duration {}", profile_dur);

    dev.set_sensor_settings(settings_sel, &sensor_settings)?;
    dev.set_sensor_mode(PowerMode::ForcedMode)?;

    thread::sleep(Duration::from_millis(profile_dur as u64));

    let data = dev.get_sensor_data()?;
    println!("Sensor Data {:?}", data);
    Ok(())
}
