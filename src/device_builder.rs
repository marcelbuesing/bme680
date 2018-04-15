use consts;
use {to_result, PowerMode, Result, SensorSettings, bme680_calib_data, bme680_dev,
     bme680_field_data, bme680_gas_sett, bme680_get_profile_dur, bme680_get_regs,
     bme680_get_sensor_data, bme680_get_sensor_mode, bme680_get_sensor_settings, bme680_init,
     bme680_intf, bme680_intf_BME680_I2C_INTF, bme680_set_profile_dur, bme680_set_sensor_mode,
     bme680_set_sensor_settings, bme680_soft_reset, bme680_tph_sett};

impl Default for bme680_field_data {
    fn default() -> bme680_field_data {
        bme680_field_data {
            status: Default::default(),
            /// The index of the heater profile used
            gas_index: Default::default(),
            /// Measurement index to track order
            meas_index: Default::default(),
            /// Temperature in degree celsius x100
            temperature: Default::default(),
            /// Pressure in Pascal
            pressure: Default::default(),
            /// Humidity in % relative humidity x1000
            humidity: Default::default(),
            /// Gas resistance in Ohms
            gas_resistance: Default::default(),
        }
    }
}

pub struct Bme680Device {
    dev: bme680_dev,
}

impl Bme680Device {
    /// @brief This API is the entry point.
    /// It reads the chip-id and calibration data from the sensor.
    ///
    /// @param[in,out] dev : Structure instance of bme680_dev
    ///
    /// @return Result of API execution status
    pub fn init(mut self) -> Result<()> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe { to_result(bme680_init(dev_ptr)) }
    }

    /// @brief This API writes the given data to the register address
    /// of the sensor.
    ///
    /// @param[in] reg_addr : Register address from where the data to be written.
    /// @param[in] reg_data : Pointer to data buffer which is to be written
    /// in the sensor.
    /// @param[in] len : No of bytes of data to write..
    /// @param[in] dev : Structure instance of bme680_dev.
    ///
    /// @return Result of API execution status
    pub fn set_regs(mut self, reg_addr: u8, reg_data: &mut [u8]) -> Result<()> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe {
            to_result(bme680_get_regs(
                reg_addr,
                reg_data.as_mut_ptr(),
                reg_data.len() as u16,
                dev_ptr,
            ))
        }
    }
    /// @brief This API reads the data from the given register address of the sensor.
    ///
    /// @param[in] reg_addr : Register address from where the data to be read
    /// @param[out] reg_data : buffer to store the read data.
    ///
    /// @return Result of API execution status
    pub fn get_regs(mut self, reg_addr: u8, reg_data: &mut [u8]) -> Result<()> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe {
            to_result(bme680_get_regs(
                reg_addr,
                reg_data.as_mut_ptr(),
                reg_data.len() as u16,
                dev_ptr,
            ))
        }
    }
    /// @brief This API performs the soft reset of the sensor.
    ///
    /// @return Result of API execution status
    pub fn soft_reset(mut self) -> Result<()> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe { to_result(bme680_soft_reset(dev_ptr)) }
    }

    /// @brief This API is used to set the power mode of the sensor.
    ///
    /// @param[in] power_mode : Sensor power mode
    ///
    /// @return Result of API execution status
    pub fn set_sensor_mode(mut self, power_mode: PowerMode) -> Result<()> {
        self.dev.power_mode = power_mode.value();
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe { to_result(bme680_set_sensor_mode(dev_ptr)) }
    }

    /// @brief This API is used to get the power mode of the sensor.
    ///
    /// @return Sensor power mode
    pub fn get_sensor_mode(mut self) -> Result<PowerMode> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        let r = unsafe { to_result(bme680_get_sensor_mode(dev_ptr)) };
        r.map(|_| PowerMode::from(self.dev.power_mode))
    }

    /// @brief This API is used to set the profile duration of the sensor.
    ///
    /// @param[in] duration : Duration of the measurement in ms.
    pub fn set_profile_dur(mut self, duration: u16) {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe { bme680_set_profile_dur(duration, dev_ptr) }
    }

    /// @brief This API is used to get the profile duration of the sensor.
    ///
    /// @return Duration of the measurement in ms.
    pub fn get_profile_dur(mut self) -> u16 {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        let mut duration = 0;
        unsafe { bme680_get_profile_dur(&mut duration, dev_ptr) };
        duration
    }

    /// @brief This API reads the pressure, temperature and humidity and gas data
    /// from the sensor, compensates the data and store it in the bme680_data
    /// structure instance passed by the user.
    ///
    /// @param[out] data: Structure instance to hold the data.
    /// @param[in] dev : Structure instance of bme680_dev.
    ///
    /// @return Result of API execution status
    pub fn get_sensor_data(mut self) -> Result<bme680_field_data> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        let mut field_data = Default::default();
        let field_data_ptr: *mut bme680_field_data = &mut field_data;
        let r = unsafe { to_result(bme680_get_sensor_data(field_data_ptr, dev_ptr)) };
        r.map(|_| field_data)
    }

    /// @brief This API is used to set the oversampling, filter and T,P,H, gas selection
    /// settings in the sensor.
    ///
    /// @param[in] desired_settings : Variable used to select the settings which
    /// @return Result of API execution status

    pub fn set_sensor_settings(mut self, sensor_settings: SensorSettings) -> Result<()> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        unsafe { to_result(bme680_set_sensor_settings(sensor_settings.bits(), dev_ptr)) }
    }

    pub fn get_tph_sett(mut self) -> Result<bme680_tph_sett> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        let settings_sel = SensorSettings::OST_SEL | SensorSettings::OSP_SEL
            | SensorSettings::OSH_SEL | SensorSettings::FILTER_SEL;

        let r = unsafe { to_result(bme680_get_sensor_settings(settings_sel.bits(), dev_ptr)) };
        r.map(|_| self.dev.tph_sett)
    }

    pub fn get_gas_sett(mut self) -> Result<bme680_gas_sett> {
        let dev_ptr: *mut bme680_dev = &mut self.dev;
        let settings_sel = SensorSettings::GAS_SENSOR_SEL;

        let r = unsafe { to_result(bme680_get_sensor_settings(settings_sel.bits(), dev_ptr)) };
        r.map(|_| self.dev.gas_sett)
    }
}

impl Default for bme680_calib_data {
    fn default() -> bme680_calib_data {
        bme680_calib_data {
            /// Variable to store calibrated humidity data
            par_h1: Default::default(),
            /// Variable to store calibrated humidity data
            par_h2: Default::default(),
            /// Variable to store calibrated humidity data
            par_h3: Default::default(),
            /// Variable to store calibrated humidity data
            par_h4: Default::default(),
            /// Variable to store calibrated humidity data
            par_h5: Default::default(),
            /// Variable to store calibrated humidity data
            par_h6: Default::default(),
            /// Variable to store calibrated humidity data
            par_h7: Default::default(),
            /// Variable to store calibrated gas data
            par_gh1: Default::default(),
            /// Variable to store calibrated gas data
            par_gh2: Default::default(),
            /// Variable to store calibrated gas data
            par_gh3: Default::default(),
            /// Variable to store calibrated temperature data
            par_t1: Default::default(),
            /// Variable to store calibrated temperature data
            par_t2: Default::default(),
            /// Variable to store calibrated temperature data
            par_t3: Default::default(),
            /// Variable to store calibrated pressure data
            par_p1: Default::default(),
            /// Variable to store calibrated pressure data
            par_p2: Default::default(),
            /// Variable to store calibrated pressure data
            par_p3: Default::default(),
            /// Variable to store calibrated pressure data
            par_p4: Default::default(),
            /// Variable to store calibrated pressure data
            par_p5: Default::default(),
            /// Variable to store calibrated pressure data
            par_p6: Default::default(),
            /// Variable to store calibrated pressure data
            par_p7: Default::default(),
            /// Variable to store calibrated pressure data
            par_p8: Default::default(),
            /// Variable to store calibrated pressure data
            par_p9: Default::default(),
            /// Variable to store calibrated pressure data
            par_p10: Default::default(),
            /// Variable to store t_fine size
            t_fine: Default::default(),
            /// Variable to store heater resistance range
            res_heat_range: Default::default(),
            /// Variable to store heater resistance value
            res_heat_val: Default::default(),
            /// Variable to store error range
            range_sw_err: Default::default(),
        }
    }
}

impl Default for bme680_gas_sett {
    fn default() -> bme680_gas_sett {
        bme680_gas_sett {
            /// Variable to store nb conversion
            nb_conv: Default::default(),
            /// Variable to store heater control
            heatr_ctrl: Default::default(),
            /// Run gas enable value
            run_gas: Default::default(),
            /// Heater temperature value
            heatr_temp: Default::default(),
            /// Duration profile value
            heatr_dur: Default::default(),
        }
    }
}

impl Default for bme680_tph_sett {
    fn default() -> bme680_tph_sett {
        bme680_tph_sett {
            /// Humidity oversampling
            os_hum: Default::default(),
            /// Temperature oversampling
            os_temp: Default::default(),
            /// Pressure oversampling
            os_pres: Default::default(),
            /// Filter coefficient
            filter: Default::default(),
        }
    }
}

pub struct Bme680DeviceBuilder {
    dev: bme680_dev,
}

/// Generic communication function pointer
/// @param[in] dev_id: Place holder to store the id of the device structure
/// Can be used to store the index of the Chip select or
/// I2C address of the device.
/// @param[in] reg_addr: Used to select the register the where data needs to
/// be read from or written to.
/// @param[in/out] reg_data: Data array to read/write
/// @param[in] len: Length of the data array
type ComFn = extern "C" fn(dev_id: u8, reg_addr: u8, data: *mut u8, len: u16) -> i8;

type DelayFn = extern "C" fn(period: u32);

impl Bme680DeviceBuilder {
    pub fn new(read: ComFn, write: ComFn, delay_ms: DelayFn) -> Self {
        let dev = bme680_dev {
            /// Chip Id
            chip_id: consts::BME680_CHIP_ID,
            /// Device Id
            dev_id: consts::BME680_I2C_ADDR_PRIMARY,
            /// SPI/I2C interface
            intf: bme680_intf_BME680_I2C_INTF,
            /// Memory page used
            mem_page: Default::default(),
            /// Ambient temperature in Degree C
            amb_temp: Default::default(),
            /// Sensor calibration data
            calib: Default::default(),
            /// Sensor settings
            tph_sett: Default::default(),
            /// Gas Sensor settings
            gas_sett: Default::default(),
            /// Sensor power modes
            power_mode: consts::BME680_SLEEP_MODE,
            /// New sensor fields
            new_fields: Default::default(),
            /// Store the info messages
            info_msg: Default::default(),
            /// Bus read function pointer
            read: Some(read),
            /// Bus write function pointer
            write: Some(write),
            /// delay function pointer
            delay_ms: Some(delay_ms),
            /// Communication function result
            com_rslt: Default::default(),
        };

        Self { dev: dev }
    }

    pub fn with_chip_id(mut self, chip_id: u8) -> Self {
        self.dev.chip_id = chip_id;
        self
    }

    pub fn with_dev_id(mut self, dev_id: u8) -> Self {
        self.dev.dev_id = dev_id;
        self
    }

    pub fn with_intf(mut self, intf: bme680_intf) -> Self {
        self.dev.intf = intf;
        self
    }

    pub fn with_mem_page(mut self, mem_page: u8) -> Self {
        self.dev.mem_page = mem_page;
        self
    }

    pub fn with_amb_temp(mut self, amb_temp: i8) -> Self {
        self.dev.amb_temp = amb_temp;
        self
    }

    pub fn with_calib(mut self, calib: bme680_calib_data) -> Self {
        self.dev.calib = calib;
        self
    }

    pub fn with_tph_sett(mut self, tph_sett: bme680_tph_sett) -> Self {
        self.dev.tph_sett = tph_sett;
        self
    }

    pub fn with_gas_sett(mut self, gas_sett: bme680_gas_sett) -> Self {
        self.dev.gas_sett = gas_sett;
        self
    }

    pub fn with_power_mode(mut self, power_mode: u8) -> Self {
        self.dev.power_mode = power_mode;
        self
    }

    pub fn with_new_fields(mut self, new_fields: u8) -> Self {
        self.dev.new_fields = new_fields;
        self
    }

    pub fn with_info_msg(mut self, info_msg: u8) -> Self {
        self.dev.info_msg = info_msg;
        self
    }

    pub fn with_com_rslt(mut self, com_rslt: i8) -> Self {
        self.dev.com_rslt = com_rslt;
        self
    }

    pub fn build(self) -> Bme680Device {
        Bme680Device { dev: self.dev }
    }
}
