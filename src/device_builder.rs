use consts;
use {to_result, Result, bme680_calib_data, bme680_dev, bme680_gas_sett, bme680_init, bme680_intf,
     bme680_intf_BME680_I2C_INTF, bme680_tph_sett};

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
