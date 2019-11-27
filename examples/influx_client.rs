///
/// This example demonstrates how to read values from the sensor and
/// continously send them to an influx database.
/// Make sure you adapt the influx constants and likely also the i2c device id and I2CAddress.
///
extern crate bme680;
extern crate env_logger;
extern crate futures;
extern crate influent;
extern crate linux_embedded_hal;
extern crate tokio;

use crate::futures::compat::Future01CompatExt;
use bme680::{
    Bme680, FieldDataCondition, I2CAddress, IIRFilterSize, OversamplingSetting, PowerMode,
    SettingsBuilder,
};
use futures::prelude::*;
use influent::client::{Client, ClientError, Credentials};
use influent::create_client;
use influent::measurement::{Measurement, Value};
use linux_embedded_hal::*;
use std::time::Duration;

const INFLUX_ADDRESS: &str = "http://127.0.0.1:8086";
const INFLUX_USER: &str = "user";
const INFLUX_PASSWORD: &str = "pass";
const INFLUX_DATABASE: &str = "influxdb";

#[tokio::main]
async fn main() -> Result<(), ()> {
    // Init device
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let mut dev = Bme680::init(i2c, Delay {}, I2CAddress::Primary)
        .map_err(|e| eprintln!("Init failed: {:?}", e))?;

    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();
    dev.set_sensor_settings(settings)
        .map_err(|e| eprintln!("Setting sensor settings failed: {:?}", e))?;

    // Set up Influx client
    let credentials = Credentials {
        username: INFLUX_USER,
        password: INFLUX_PASSWORD,
        database: INFLUX_DATABASE,
    };

    let hosts = vec![INFLUX_ADDRESS];
    let client = create_client(credentials, hosts);

    dev.set_sensor_mode(PowerMode::ForcedMode)
        .map_err(|e| eprintln!("Setting sensor mode failed: {:?}", e))?;
    let (data, state) = dev
        .get_sensor_data()
        .map_err(|e| eprintln!("Retrieving sensor data failed: {:?}", e))?;

    println!("State {:?}", state);
    println!("Temperature {}°C", data.temperature_celsius());
    println!("Pressure {}hPa", data.pressure_hpa());
    println!("Humidity {}%", data.humidity_percent());
    println!("Gas Resistence {}Ω", data.gas_resistance_ohm());

    if state != FieldDataCondition::NewData {
        let temperature_f = send_value(
            &client,
            "temperature",
            Value::Float(data.temperature_celsius() as f64),
        );
        let pressure_f = send_value(
            &client,
            "pressure",
            Value::Float(data.pressure_hpa() as f64),
        );
        let humidity_f = send_value(
            &client,
            "humidity",
            Value::Float(data.humidity_percent() as f64),
        );
        let gas_f = send_value(
            &client,
            "gasresistence",
            Value::Float(data.gas_resistance_ohm() as f64),
        );

        if let Err(e) = future::try_join4(temperature_f, pressure_f, humidity_f, gas_f).await {
            eprintln!("Error: {:?}", e);
        }
    }
    Ok(())
}

/// Sends a measured value to the influx database
async fn send_value<'a>(
    client: &dyn Client,
    type_name: &str,
    value: Value<'a>,
) -> Result<(), ClientError> {
    let mut measurement = Measurement::new("sensor");
    measurement.add_field("value", value);
    measurement.add_tag("id", "MAC");
    measurement.add_tag("name", "bme680");
    measurement.add_tag("type", type_name);

    client.write_one(measurement, None).compat().await?;
    Ok(())
}
