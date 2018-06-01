///
/// This example demonstrates how to read values from the sensor and
/// continously send them to an influx database.
/// Make sure you adapt the influx constants and likely also the i2c device id and I2CAddress.
///

extern crate bme680;
extern crate embedded_hal;
extern crate env_logger;
extern crate linux_embedded_hal;
extern crate influent;

use bme680::*;
use embedded_hal::blocking::i2c;
use linux_embedded_hal::*;
use std::thread;
use std::time::Duration;

use influent::create_client;
use influent::client::{Client, Credentials};
use influent::measurement::{Measurement, Value};

const INFLUX_ADDRESS: &str = "http://127.0.0.1:8086";
const INFLUX_USER: &str     = "user";
const INFLUX_PASSWORD: &str = "pass";
const INFLUX_DATABASE: &str = "influxdb";

type I2CResult = std::result::Result<(), Error<<I2cdev as i2c::Read>::Error, <I2cdev as i2c::Write>::Error>>;

fn main() -> I2CResult
{

    // Init device
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let mut dev = Bme680::init(i2c, Delay {}, I2CAddress::Primary)?;

    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();
    dev.set_sensor_settings(settings)?;

    // Set up Influx client
    let credentials = Credentials {
        username: INFLUX_USER,
        password: INFLUX_PASSWORD,
        database: INFLUX_DATABASE
    };

    let hosts = vec![INFLUX_ADDRESS];
    let client = create_client(credentials, hosts);

    loop {
        dev.set_sensor_mode(PowerMode::ForcedMode)?;
        let (data, state) = dev.get_sensor_data()?;

        println!("State {:?}", state);
        println!("Temperature {}°C", data.temperature_celsius());
        println!("Pressure {}hPa", data.pressure_hpa());
        println!("Humidity {}%", data.humidity_percent());
        println!("Gas Resistence {}Ω", data.gas_resistance_ohm());

        if state == FieldDataCondition::NewData {

           send_value(&client, "temperature" ,Value::Float(data.temperature_celsius() as f64));
           send_value(&client, "pressure" ,Value::Float(data.pressure_hpa() as f64));
           send_value(&client, "humidity" ,Value::Float(data.humidity_percent() as f64));
           send_value(&client, "gasresistence" , Value::Float(data.gas_resistance_ohm() as f64));
        }
        thread::sleep(Duration::from_millis(5000));
    }
}

/// Sends a measured value to the influx database
fn send_value(client:&Client, type_name: &str, value: Value) {
    let mut measurement = Measurement::new("sensor");
    measurement.add_field("value", value);
    measurement.add_tag("id", "MAC");
    measurement.add_tag("name", "bme680");
    measurement.add_tag("type", type_name);

    client.write_one(measurement, None).unwrap();
}
