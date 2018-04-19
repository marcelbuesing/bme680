use byteorder::{BigEndian, LittleEndian, ReadBytesExt, WriteBytesExt};
use hal::blocking::delay::DelayMs;
use std::result;

/** BME680 General config */
pub const BME680_POLL_PERIOD_MS: u8 = 10;

/** BME680 I2C addresses */
pub const BME680_I2C_ADDR_PRIMARY: u8 = 0x76;
pub const BME680_I2C_ADDR_SECONDARY: u8 = 0x77;

/** BME680 unique chip identifier */
pub const BME680_CHIP_ID: u8 = 0x61;

pub const BME680_SOFT_RESET_CMD: u8 = 0xb6;

pub const BME680_OK: i8 = 0;
pub const BME680_E_NULL_PTR: i8 = -1;
pub const BME680_E_COM_FAIL: i8 = -2;
pub const BME680_E_DEV_NOT_FOUND: i8 = -3;
pub const BME680_E_INVALID_LENGTH: i8 = -4;

pub const BME680_SOFT_RESET_ADDR: u8 = 0xe0;

/** Field settings */
pub const BME680_FIELD0_ADDR: u8 = 0x1d;

/** Heater settings */
pub const BME680_RES_HEAT0_ADDR: u8 = 0x5a;
pub const BME680_GAS_WAIT0_ADDR: u8 = 0x64;

/** Sensor configuration registers */
pub const BME680_CONF_HEAT_CTRL_ADDR: u8 = 0x70;
pub const BME680_CONF_ODR_RUN_GAS_NBC_ADDR: u8 = 0x71;
pub const BME680_CONF_OS_H_ADDR: u8 = 0x72;
pub const BME680_MEM_PAGE_ADDR: u8 = 0xf3;
pub const BME680_CONF_T_P_MODE_ADDR: u8 = 0x74;
pub const BME680_CONF_ODR_FILT_ADDR: u8 = 0x75;

/** Coefficient's address */
pub const BME680_COEFF_ADDR1: u8 = 0x89;
pub const BME680_COEFF_ADDR2: u8 = 0xe1;

/** Chip identifier */
pub const BME680_CHIP_ID_ADDR: u8 = 0xd0;

pub const BME680_SLEEP_MODE: u8 = 0;
pub const BME680_FORCED_MODE: u8 = 1;

pub const BME680_RESET_PERIOD: u32 = 10;

pub const BME680_GAS_MEAS_MSK: u8 = 0x30;
pub const BME680_NBCONV_MSK: u8 = 0x0F;
pub const BME680_FILTER_MSK: u8 = 0x1C;
pub const BME680_OST_MSK: u8 = 0xE0;
pub const BME680_OSP_MSK: u8 = 0x1C;
pub const BME680_OSH_MSK: u8 = 0x07;
pub const BME680_HCTRL_MSK: u8 = 0x08;
pub const BME680_RUN_GAS_MSK: u8 = 0x10;
pub const BME680_MODE_MSK: u8 = 0x03;
pub const BME680_RHRANGE_MSK: u8 = 0x30;
pub const BME680_RSERROR_MSK: u8 = 0xf0;
pub const BME680_NEW_DATA_MSK: u8 = 0x80;
pub const BME680_GAS_INDEX_MSK: u8 = 0x0f;
pub const BME680_GAS_RANGE_MSK: u8 = 0x0f;
pub const BME680_GASM_VALID_MSK: u8 = 0x20;
pub const BME680_HEAT_STAB_MSK: u8 = 0x10;
pub const BME680_MEM_PAGE_MSK: u8 = 0x10;
pub const BME680_SPI_RD_MSK: u8 = 0x80;
pub const BME680_SPI_WR_MSK: u8 = 0x7f;
pub const BME680_BIT_H1_DATA_MSK: u8 = 0x0F;

///
/// SPI memory page settings
///
pub const BME680_MEM_PAGE0: u8 = 0x10;

///
/// SPI memory page settings
///
pub const BME680_MEM_PAGE1: u8 = 0x00;

pub enum Bme680Error {
    ///
    /// aka BME680_E_NULL_PTR
    ///
    NulltPtr,
    ///
    /// aka BME680_E_COM_FAIL
    ///
    CommunicationFailure,
    ///
    /// aka BME680_E_DEV_NOT_FOUND
    ///
    DeviceNotFound,
    ///
    /// aka BME680_E_INVALID_LENGTH
    ///
    InvalidLength,
}

pub type Result<T> = result::Result<T, Bme680Error>;

///
/// Power mode settings
///
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

    fn value(&self) -> u8 {
        match self {
            PowerMode::SleepMode => BME680_SLEEP_MODE,
            PowerMode::ForcedMode => BME680_FORCED_MODE,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(i32)]
pub enum Bme680_intf {
    BME680_SPI_INTF,
    BME680_I2C_INTF,
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_calib_data {
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
    pub t_fine: i32,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}

impl Clone for Bme680_calib_data {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_tph_sett {
    pub os_hum: u8,
    pub os_temp: u8,
    pub os_pres: u8,
    pub filter: u8,
}

impl Clone for Bme680_tph_sett {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_gas_sett {
    pub nb_conv: u8,
    pub heatr_ctrl: u8,
    pub run_gas: u8,
    pub heatr_temp: u16,
    pub heatr_dur: u16,
}

impl Clone for Bme680_gas_sett {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_field_data {
    pub status: u8,
    pub gas_index: u8,
    pub meas_index: u8,
    pub temperature: i16,
    pub pressure: u32,
    pub humidity: u32,
    pub gas_resistance: u32,
}

impl Clone for Bme680_field_data {
    fn clone(&self) -> Self {
        *self
    }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_dev<I2C, D> {
    pub i2c: I2C,
    pub delay: D,
    pub chip_id: u8,
    pub dev_id: u8,
    pub mem_page: u8,
    pub amb_temp: i8,
    pub calib: Bme680_calib_data,
    pub tph_sett: Bme680_tph_sett,
    pub gas_sett: Bme680_gas_sett,
    pub power_mode: PowerMode,
    pub new_fields: u8,
    pub info_msg: u8,
    pub com_rslt: i8,
}

impl<I2C, D, E> Clone for Bme680_dev<I2C, D> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<I2C, D, E> Bme680_dev<I2C, D> {
    pub fn init(self) -> Result<()> {
        rslt = self.bme680_get_regs(0xd0u8, &mut self.chip_id as (*mut u8), 1u16);
        if rslt as (i32) == BME680_OK {
            if self.chip_id as (i32) == 0x61i32 {
                rslt = self.get_calib_data();
            } else {
                rslt = -3i8;
            }
        }
        rslt
    }

    pub fn bme680_get_regs(self, mut reg_addr: u8, mut reg_data: &mut [u8], mut len: u16) -> i8 {
        let mut rslt: i8;
        self.com_rslt = self.i2c.read(reg_addr, reg_data, len);
        if self.com_rslt as (i32) != 0i32 {
            rslt = -2i8;
        }
        rslt
    }

    pub fn bme680_set_regs(
        self,
        mut reg_addr: *const u8,
        mut reg_data: *const u8,
        mut len: u8,
    ) -> i8 {
        let mut rslt: i8;
        let mut tmp_buff: [u8; 40] = 0i32 as ([u8; 40]);
        let mut index: u16;
        if len as (i32) > 0i32 && (len as (i32) < 40i32 / 2i32) {
            index = 0u16;
            'loop4: loop {
                if !(index as (i32) < len as (i32)) {
                    break;
                }
                tmp_buff[(2i32 * index as (i32)) as (usize)] = *reg_addr.offset(index as (isize));
                tmp_buff[(2i32 * index as (i32) + 1i32) as (usize)] =
                    *reg_data.offset(index as (isize));
                index = (index as (i32) + 1) as (u16);
            }
            if rslt as (i32) == BME680_OK {
                self.com_rslt = (self.write)(
                    self.dev_id,
                    tmp_buff[0usize],
                    &mut tmp_buff[1usize] as (*mut u8),
                    (2i32 * len as (i32) - 1i32) as (u16),
                );
                if self.com_rslt as (i32) != 0i32 {
                    rslt = -2i8;
                }
            }
        } else {
            rslt = -4i8;
        }
        rslt
    }

    pub fn soft_reset(self, delay: D) -> Result
    where
        D: DelayMs<u8>,
    {
        let mut rslt: i8;
        let mut reg_addr: u8 = BME680_SOFT_RESET_ADDR;
        let mut soft_rst_cmd: u8 = BME680_SOFT_RESET_CMD;
        rslt = self.bme680_set_regs(
            &mut reg_addr as (*mut u8) as (*const u8),
            &mut soft_rst_cmd as (*mut u8) as (*const u8),
            1u8,
        );
        delay.delay_ms(BME680_RESET_PERIOD);
        rslt
    }

    pub fn bme680_set_sensor_settings(self, mut desired_settings: u16) -> i8 {
        let mut rslt: i8;
        let mut reg_addr: u8;
        let mut data: u8 = 0u8;
        let mut count: u8 = 0u8;
        let mut reg_array: [u8; 6] = 0i32 as ([u8; 6]);
        let mut data_array: [u8; 6] = 0i32 as ([u8; 6]);
        let mut intended_power_mode: u8 = self.power_mode;
        if desired_settings as (i32) & 8i32 != 0 {
            rslt = self.set_gas_config();
        }
        self.power_mode = 0u8;
        if rslt as (i32) == BME680_OK {
            rslt = self.bme680_set_sensor_mode();
        }
        if desired_settings as (i32) & 16i32 != 0 {
            rslt = self.boundary_check(&mut self.tph_sett.filter as (*mut u8), 0u8, 7u8);
            reg_addr = 0x75u8;
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regs(reg_addr, &mut data as (*mut u8), 1u16);
            }
            if desired_settings as (i32) & 16i32 != 0 {
                data = (data as (i32) & !0x1ci32 | self.tph_sett.filter as (i32) << 2i32 & 0x1ci32)
                    as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & 32i32 != 0 {
            rslt = self.boundary_check(&mut self.gas_sett.heatr_ctrl as (*mut u8), 0x0u8, 0x8u8);
            reg_addr = 0x70u8;
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regs(reg_addr, &mut data as (*mut u8), 1u16);
            }
            data = (data as (i32) & !0x8i32 | self.gas_sett.heatr_ctrl as (i32) & 0x8i32) as (u8);
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & (1i32 | 2i32) != 0 {
            rslt = self.boundary_check(&mut self.tph_sett.os_temp as (*mut u8), 0u8, 5u8);
            reg_addr = 0x74u8;
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regs(reg_addr, &mut data as (*mut u8), 1u16);
            }
            if desired_settings as (i32) & 1i32 != 0 {
                data = (data as (i32) & !0xe0i32 | self.tph_sett.os_temp as (i32) << 5i32 & 0xe0i32)
                    as (u8);
            }
            if desired_settings as (i32) & 2i32 != 0 {
                data = (data as (i32) & !0x1ci32 | self.tph_sett.os_pres as (i32) << 2i32 & 0x1ci32)
                    as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & 4i32 != 0 {
            rslt = self.boundary_check(&mut self.tph_sett.os_hum as (*mut u8), 0u8, 5u8);
            reg_addr = 0x72u8;
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regs(reg_addr, &mut data as (*mut u8), 1u16);
            }
            data = (data as (i32) & !0x7i32 | self.tph_sett.os_hum as (i32) & 0x7i32) as (u8);
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & (64i32 | 128i32) != 0 {
            rslt = self.boundary_check(&mut self.gas_sett.run_gas as (*mut u8), 0u8, 1u8);
            if rslt as (i32) == BME680_OK {
                rslt = self.boundary_check(&mut self.gas_sett.nb_conv as (*mut u8), 0u8, 10u8);
            }
            reg_addr = 0x71u8;
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regsy(reg_addr, &mut data as (*mut u8), 1u16);
            }
            if desired_settings as (i32) & 64i32 != 0 {
                data = (data as (i32) & !0x10i32 | self.gas_sett.run_gas as (i32) << 4i32 & 0x10i32)
                    as (u8);
            }
            if desired_settings as (i32) & 128i32 != 0 {
                data = (data as (i32) & !0xfi32 | self.gas_sett.nb_conv as (i32) & 0xfi32) as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if rslt as (i32) == BME680_OK {
            rslt = self.bme680_set_regs(
                reg_array.as_mut_ptr() as (*const u8),
                data_array.as_mut_ptr() as (*const u8),
                count,
            );
        }
        self.power_mode = intended_power_mode;
        rslt
    }

    pub fn bme680_get_sensor_settings(self, mut desired_settings: u16) -> i8 {
        let mut rslt: i8;
        let mut reg_addr: u8 = 0x70u8;
        let mut data_array: [u8; 6] = 0i32 as ([u8; 6]);
        rslt = self.bme680_get_regs(reg_addr, data_array.as_mut_ptr(), 6u16);
        if rslt as (i32) == BME680_OK {
            if desired_settings as (i32) & 8i32 != 0 {
                rslt = self.get_gas_config();
            }
            if desired_settings as (i32) & 16i32 != 0 {
                self.tph_sett.filter = ((data_array[5usize] as (i32) & 0x1ci32) >> 2i32) as (u8);
            }
            if desired_settings as (i32) & (1i32 | 2i32) != 0 {
                self.tph_sett.os_temp = ((data_array[4usize] as (i32) & 0xe0i32) >> 5i32) as (u8);
                self.tph_sett.os_pres = ((data_array[4usize] as (i32) & 0x1ci32) >> 2i32) as (u8);
            }
            if desired_settings as (i32) & 4i32 != 0 {
                self.tph_sett.os_hum = (data_array[2usize] as (i32) & 0x7i32) as (u8);
            }
            if desired_settings as (i32) & 32i32 != 0 {
                self.gas_sett.heatr_ctrl = (data_array[0usize] as (i32) & 0x8i32) as (u8);
            }
            if desired_settings as (i32) & (64i32 | 128i32) != 0 {
                self.gas_sett.nb_conv = (data_array[1usize] as (i32) & 0xfi32) as (u8);
                self.gas_sett.run_gas = ((data_array[1usize] as (i32) & 0x10i32) >> 4i32) as (u8);
            }
        }
        rslt
    }

    pub fn bme680_set_sensor_mode(self) -> i8 {
        let mut _currentBlock;
        let mut rslt: i8;
        let mut tmp_pow_mode: u8;
        let mut pow_mode: u8 = 0u8;
        let mut reg_addr: u8 = 0x74u8;
        _currentBlock = 1;
        'loop1: loop {
            if _currentBlock == 1 {
                rslt = self.bme680_get_regs(
                    BME680_CONF_T_P_MODE_ADDR,
                    &mut tmp_pow_mode as (*mut u8),
                    1u16,
                );
                if rslt as (i32) == BME680_OK {
                    pow_mode = (tmp_pow_mode as (i32) & 0x3i32) as (u8);
                    if pow_mode as (i32) != 0i32 {
                        tmp_pow_mode = (tmp_pow_mode as (i32) & !0x3i32) as (u8);
                        rslt = self.bme680_set_regs(
                            &mut reg_addr as (*mut u8) as (*const u8),
                            &mut tmp_pow_mode as (*mut u8) as (*const u8),
                            1u8,
                        );
                        (self.delay_ms)(BME680_POLL_PERIOD_MS);
                    }
                }
                if pow_mode as (i32) != BME680_SLEEP_MODE {
                    _currentBlock = 1;
                    continue;
                }
                if !(self.power_mode as (i32) != BME680_SLEEP_MODE) {
                    _currentBlock = 8;
                    continue;
                }
                tmp_pow_mode = (tmp_pow_mode as (i32) & !BME680_MODE_MSK
                    | self.power_mode as (i32) & BME680_MODE_MSK)
                    as (u8);
                if !(rslt as (i32) == BME680_OK) {
                    _currentBlock = 8;
                    continue;
                }
                rslt = self.bme680_set_regs(
                    &mut reg_addr as (*mut u8) as (*const u8),
                    &mut tmp_pow_mode as (*mut u8) as (*const u8),
                    1u8,
                );
                _currentBlock = 8;
            } else {
                return rslt;
            }
        }
    }

    pub fn bme680_get_sensor_mode(self) -> i8 {
        let mut mode: u8;
        let rslt = self.bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &mut mode as (*mut u8), 1u16);
        self.power_mode = (mode as (i32) & BME680_MODE_MSK) as (u8);
        rslt
    }

    pub fn bme680_set_profile_dur(self, mut duration: u16) {
        let mut os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        let mut meas_cycles = os_to_meas_cycles[self.tph_sett.os_temp as (usize)] as (u32);
        meas_cycles =
            meas_cycles.wrapping_add(os_to_meas_cycles[self.tph_sett.os_pres as (usize)] as (u32));
        meas_cycles =
            meas_cycles.wrapping_add(os_to_meas_cycles[self.tph_sett.os_hum as (usize)] as (u32));
        let mut tph_dur = meas_cycles.wrapping_mul(1963u32);
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
        tph_dur = tph_dur.wrapping_add(500u32);
        tph_dur = tph_dur.wrapping_div(1000u32);
        tph_dur = tph_dur.wrapping_add(1u32);
        self.gas_sett.heatr_dur = (duration as (i32) - tph_dur as (u16) as (i32)) as (u16);
    }

    pub fn bme680_get_profile_dur(self, mut duration: *mut u16) {
        let mut tph_dur: u32;
        let mut meas_cycles: u32;
        let mut os_to_meas_cycles: [u8; 6] = [0u8, 1u8, 2u8, 4u8, 8u8, 16u8];
        meas_cycles = os_to_meas_cycles[self.tph_sett.os_temp as (usize)] as (u32);
        meas_cycles =
            meas_cycles.wrapping_add(os_to_meas_cycles[self.tph_sett.os_pres as (usize)] as (u32));
        meas_cycles =
            meas_cycles.wrapping_add(os_to_meas_cycles[self.tph_sett.os_hum as (usize)] as (u32));
        tph_dur = meas_cycles.wrapping_mul(1963u32);
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
        tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
        tph_dur = tph_dur.wrapping_add(500u32);
        tph_dur = tph_dur.wrapping_div(1000u32);
        tph_dur = tph_dur.wrapping_add(1u32);
        *duration = tph_dur as (u16);
        if self.gas_sett.run_gas != 0 {
            *duration = (*duration as (i32) + self.gas_sett.heatr_dur as (i32)) as (u16);
        }
    }

    pub fn bme680_get_sensor_data(self, mut data: *mut Bme680_field_data) -> i8 {
        let mut rslt: i8;
        rslt = self.read_field_data(data);
        if rslt as (i32) == BME680_OK {
            if (*data).status as (i32) & 0x80i32 != 0 {
                self.new_fields = 1u8;
            } else {
                self.new_fields = 0u8;
            }
        }
        rslt
    }

    fn get_calib_data(self) -> i8 {
        let mut rslt: i8;
        let mut coeff_array: [u8; 41] = 0i32 as ([u8; 41]);
        let mut temp_var: u8 = 0u8;
        rslt = self.bme680_get_regs(0x89u8, coeff_array.as_mut_ptr(), 25u16);
        if rslt as (i32) == BME680_OK {
            rslt = self.bme680_get_regs(0xe1u8, &mut coeff_array[25usize] as (*mut u8), 16u16);
        }
        self.calib.par_t1 = (coeff_array[34usize] as (u16) as (i32) << 8i32
            | coeff_array[33usize] as (u16) as (i32)) as (u16);
        self.calib.par_t2 = (coeff_array[2usize] as (u16) as (i32) << 8i32
            | coeff_array[1usize] as (u16) as (i32)) as (i16);
        self.calib.par_t3 = coeff_array[3usize] as (i8);
        self.calib.par_p1 = (coeff_array[6usize] as (u16) as (i32) << 8i32
            | coeff_array[5usize] as (u16) as (i32)) as (u16);
        self.calib.par_p2 = (coeff_array[8usize] as (u16) as (i32) << 8i32
            | coeff_array[7usize] as (u16) as (i32)) as (i16);
        self.calib.par_p3 = coeff_array[9usize] as (i8);
        self.calib.par_p4 = (coeff_array[12usize] as (u16) as (i32) << 8i32
            | coeff_array[11usize] as (u16) as (i32)) as (i16);
        self.calib.par_p5 = (coeff_array[14usize] as (u16) as (i32) << 8i32
            | coeff_array[13usize] as (u16) as (i32)) as (i16);
        self.calib.par_p6 = coeff_array[16usize] as (i8);
        self.calib.par_p7 = coeff_array[15usize] as (i8);
        self.calib.par_p8 = (coeff_array[20usize] as (u16) as (i32) << 8i32
            | coeff_array[19usize] as (u16) as (i32)) as (i16);
        self.calib.par_p9 = (coeff_array[22usize] as (u16) as (i32) << 8i32
            | coeff_array[21usize] as (u16) as (i32)) as (i16);
        self.calib.par_p10 = coeff_array[23usize];
        self.calib.par_h1 = (coeff_array[27usize] as (u16) as (i32) << 4i32
            | coeff_array[26usize] as (i32) & 0xfi32) as (u16);
        self.calib.par_h2 = (coeff_array[25usize] as (u16) as (i32) << 4i32
            | coeff_array[26usize] as (i32) >> 4i32) as (u16);
        self.calib.par_h3 = coeff_array[28usize] as (i8);
        self.calib.par_h4 = coeff_array[29usize] as (i8);
        self.calib.par_h5 = coeff_array[30usize] as (i8);
        self.calib.par_h6 = coeff_array[31usize];
        self.calib.par_h7 = coeff_array[32usize] as (i8);
        self.calib.par_gh1 = coeff_array[37usize] as (i8);
        self.calib.par_gh2 = (coeff_array[36usize] as (u16) as (i32) << 8i32
            | coeff_array[35usize] as (u16) as (i32)) as (i16);
        self.calib.par_gh3 = coeff_array[38usize] as (i8);
        if rslt as (i32) == BME680_OK {
            rslt = self.bme680_get_regs(0x2u8, &mut temp_var as (*mut u8), 1u16);
            self.calib.res_heat_range = ((temp_var as (i32) & 0x30i32) / 16i32) as (u8);
            if rslt as (i32) == BME680_OK {
                rslt = self.bme680_get_regs(0x0u8, &mut temp_var as (*mut u8), 1u16);
                self.calib.res_heat_val = temp_var as (i8);
                if rslt as (i32) == BME680_OK {
                    rslt = self.bme680_get_regs(0x4u8, &mut temp_var as (*mut u8), 1u16);
                }
            }
        }
        self.calib.range_sw_err = ((temp_var as (i8) as (i32) & 0xf0i32) / 16i32) as (i8);
        rslt
    }

    fn set_gas_config(self) -> i8 {
        let mut rslt: i8;
        let mut reg_addr: [u8; 2] = 0i32 as ([u8; 2]);
        let mut reg_data: [u8; 2] = 0i32 as ([u8; 2]);
        if self.power_mode as (i32) == 1i32 {
            reg_addr[0usize] = 0x5au8;
            reg_data[0usize] = self.calc_heater_res(self.gas_sett.heatr_temp);
            reg_addr[1usize] = 0x64u8;
            reg_data[1usize] = self.calc_heater_dur(self.gas_sett.heatr_dur);
            self.gas_sett.nb_conv = 0u8;
        } else {
            rslt = 1i8;
        }
        if rslt as (i32) == BME680_OK {
            rslt = self.bme680_set_regs(
                reg_addr.as_mut_ptr() as (*const u8),
                reg_data.as_mut_ptr() as (*const u8),
                2u8,
            );
        }
        rslt
    }

    fn get_gas_config(self) -> i8 {
        let mut rslt: i8;
        let mut reg_addr1: u8 = 0x5au8;
        let mut reg_addr2: u8 = 0x64u8;
        let mut reg_data: u8 = 0u8;
        rslt = self.bme680_get_regs(reg_addr1, &mut reg_data as (*mut u8), 1u16);
        self.gas_sett.heatr_temp = reg_data as (u16);
        rslt = self.bme680_get_regs(reg_addr2, &mut reg_data as (*mut u8), 1u16);
        if rslt as (i32) == BME680_OK {
            self.gas_sett.heatr_dur = reg_data as (u16);
        }
        rslt
    }

    fn calc_heater_res(self, mut temp: u16) -> u8 {
        let mut heatr_res: u8;
        let mut var1: i32;
        let mut var2: i32;
        let mut var3: i32;
        let mut var4: i32;
        let mut var5: i32;
        let mut heatr_res_x100: i32;
        if temp as (i32) > 400i32 {
            temp = 400u16;
        }
        var1 = self.amb_temp as (i32) * self.calib.par_gh3 as (i32) / 1000i32 * 256i32;
        var2 = (self.calib.par_gh1 as (i32) + 784i32)
            * (((self.calib.par_gh2 as (i32) + 154009i32) * temp as (i32) * 5i32 / 100i32
                + 3276800i32) / 10i32);
        var3 = var1 + var2 / 2i32;
        var4 = var3 / (self.calib.res_heat_range as (i32) + 4i32);
        var5 = 131i32 * self.calib.res_heat_val as (i32) + 65536i32;
        heatr_res_x100 = (var4 / var5 - 250i32) * 34i32;
        heatr_res = ((heatr_res_x100 + 50i32) / 100i32) as (u8);
        heatr_res
    }

    fn calc_heater_dur(mut dur: u16) -> u8 {
        let mut factor: u8 = 0u8;
        let mut durval: u8;
        if dur as (i32) >= 0xfc0i32 {
            durval = 0xffu8;
        } else {
            'loop1: loop {
                if !(dur as (i32) > 0x3fi32) {
                    break;
                }
                dur = (dur as (i32) / 4i32) as (u16);
                factor = (factor as (i32) + 1i32) as (u8);
            }
            durval = (dur as (i32) + factor as (i32) * 64i32) as (u8);
        }
        durval
    }

    fn calc_temperature(self, mut temp_adc: u32) -> i16 {
        let mut var1: isize;
        let mut var2: isize;
        let mut var3: isize;
        let mut calc_temp: i16;
        var1 = ((temp_adc as (i32) >> 3i32) - (self.calib.par_t1 as (i32) << 1i32)) as (isize);
        var2 = var1 * self.calib.par_t2 as (i32) as (isize) >> 11i32;
        var3 = (var1 >> 1i32) * (var1 >> 1i32) >> 12i32;
        var3 = var3 * (self.calib.par_t3 as (i32) << 4i32) as (isize) >> 14i32;
        self.calib.t_fine = (var2 + var3) as (i32);
        calc_temp = (self.calib.t_fine * 5i32 + 128i32 >> 8i32) as (i16);
        calc_temp
    }

    fn calc_pressure(self, mut pres_adc: u32) -> u32 {
        let mut var1: i32 = 0i32;
        let mut var2: i32 = 0i32;
        let mut var3: i32 = 0i32;
        let mut pressure_comp: i32 = 0i32;
        var1 = (self.calib.t_fine >> 1i32) - 64000i32;
        var2 = ((var1 >> 2i32) * (var1 >> 2i32) >> 11i32) * self.calib.par_p6 as (i32) >> 2i32;
        var2 = var2 + (var1 * self.calib.par_p5 as (i32) << 1i32);
        var2 = (var2 >> 2i32) + (self.calib.par_p4 as (i32) << 16i32);
        var1 = (((var1 >> 2i32) * (var1 >> 2i32) >> 13i32) * (self.calib.par_p3 as (i32) << 5i32)
            >> 3i32) + (self.calib.par_p2 as (i32) * var1 >> 1i32);
        var1 = var1 >> 18i32;
        var1 = (32768i32 + var1) * self.calib.par_p1 as (i32) >> 15i32;
        pressure_comp = 1048576u32.wrapping_sub(pres_adc) as (i32);
        pressure_comp = ((pressure_comp - (var2 >> 12i32)) as (u32)).wrapping_mul(3125u32) as (i32);
        if pressure_comp >= 0x40000000i32 {
            pressure_comp = ((pressure_comp as (u32)).wrapping_div(var1 as (u32)) << 1i32) as (i32);
        } else {
            pressure_comp = ((pressure_comp << 1i32) as (u32)).wrapping_div(var1 as (u32)) as (i32);
        }
        var1 = self.calib.par_p9 as (i32)
            * ((pressure_comp >> 3i32) * (pressure_comp >> 3i32) >> 13i32) >> 12i32;
        var2 = (pressure_comp >> 2i32) * self.calib.par_p8 as (i32) >> 13i32;
        var3 = (pressure_comp >> 8i32) * (pressure_comp >> 8i32) * (pressure_comp >> 8i32)
            * self.calib.par_p10 as (i32) >> 17i32;
        pressure_comp =
            pressure_comp + (var1 + var2 + var3 + (self.calib.par_p7 as (i32) << 7i32) >> 4i32);
        pressure_comp as (u32)
    }

    fn calc_humidity(self, mut hum_adc: u16) -> u32 {
        let mut var1: i32;
        let mut var2: i32;
        let mut var3: i32;
        let mut var4: i32;
        let mut var5: i32;
        let mut var6: i32;
        let mut temp_scaled: i32;
        let mut calc_hum: i32;
        temp_scaled = self.calib.t_fine * 5i32 + 128i32 >> 8i32;
        var1 = hum_adc as (i32) - self.calib.par_h1 as (i32) * 16i32
            - (temp_scaled * self.calib.par_h3 as (i32) / 100i32 >> 1i32);
        var2 = self.calib.par_h2 as (i32)
            * (temp_scaled * self.calib.par_h4 as (i32) / 100i32
                + (temp_scaled * (temp_scaled * self.calib.par_h5 as (i32) / 100i32) >> 6i32)
                    / 100i32 + (1i32 << 14i32)) >> 10i32;
        var3 = var1 * var2;
        var4 = self.calib.par_h6 as (i32) << 7i32;
        var4 = var4 + temp_scaled * self.calib.par_h7 as (i32) / 100i32 >> 4i32;
        var5 = (var3 >> 14i32) * (var3 >> 14i32) >> 10i32;
        var6 = var4 * var5 >> 1i32;
        calc_hum = (var3 + var6 >> 10i32) * 1000i32 >> 12i32;
        if calc_hum > 100000i32 {
            calc_hum = 100000i32;
        } else if calc_hum < 0i32 {
            calc_hum = 0i32;
        }
        calc_hum as (u32)
    }

    fn calc_gas_resistance(self, mut gas_res_adc: u16, mut gas_range: u8) -> u32 {
        let mut lookupTable1: [u32; 16] = [
            2147483647u32,
            2147483647u32,
            2147483647u32,
            2147483647u32,
            2147483647u32,
            2126008810u32,
            2147483647u32,
            2130303777u32,
            2147483647u32,
            2147483647u32,
            2143188679u32,
            2136746228u32,
            2147483647u32,
            2126008810u32,
            2147483647u32,
            2147483647u32,
        ];
        let mut lookupTable2: [u32; 16] = [
            4096000000u32,
            2048000000u32,
            1024000000u32,
            512000000u32,
            255744255u32,
            127110228u32,
            64000000u32,
            32258064u32,
            16016016u32,
            8000000u32,
            4000000u32,
            2000000u32,
            1000000u32,
            500000u32,
            250000u32,
            125000u32,
        ];
        let var1 = (1340isize + 5isize * self.calib.range_sw_err as (isize))
            * lookupTable1[gas_range as (usize)] as (isize) >> 16i32;
        let var2 = ((gas_res_adc as (isize) << 15i32) - 16777216isize + var1) as (usize);
        let var3 = lookupTable2[gas_range as (usize)] as (isize) * var1 >> 9i32;
        let calc_gas_res = ((var3 + (var2 as (isize) >> 1i32)) / var2 as (isize)) as (u32);
        calc_gas_res
    }

    fn read_field_data(self, mut data: *mut Bme680_field_data) -> i8 {
        let mut _currentBlock;
        let mut rslt: i8;
        let mut buff: [u8; 15] = 0i32 as ([u8; 15]);
        let mut gas_range: u8;
        let mut adc_temp: u32;
        let mut adc_pres: u32;
        let mut adc_hum: u16;
        let mut adc_gas_res: u16;
        let mut tries: u8 = 10u8;
        'loop1: loop {
            rslt = self.bme680_get_regs(0x1du8, buff.as_mut_ptr(), 15u16);
            (*data).status = (buff[0usize] as (i32) & 0x80i32) as (u8);
            (*data).gas_index = (buff[0usize] as (i32) & 0xfi32) as (u8);
            (*data).meas_index = buff[1usize];
            adc_pres = (buff[2usize] as (u32)).wrapping_mul(4096u32)
                | (buff[3usize] as (u32)).wrapping_mul(16u32)
                | (buff[4usize] as (u32)).wrapping_div(16u32);
            adc_temp = (buff[5usize] as (u32)).wrapping_mul(4096u32)
                | (buff[6usize] as (u32)).wrapping_mul(16u32)
                | (buff[7usize] as (u32)).wrapping_div(16u32);
            adc_hum =
                ((buff[8usize] as (u32)).wrapping_mul(256u32) | buff[9usize] as (u32)) as (u16);
            adc_gas_res = ((buff[13usize] as (u32)).wrapping_mul(4u32)
                | (buff[14usize] as (u32)).wrapping_div(64u32)) as (u16);
            gas_range = (buff[14usize] as (i32) & 0xfi32) as (u8);
            (*data).status = ((*data).status as (i32) | buff[14usize] as (i32) & 0x20i32) as (u8);
            (*data).status = ((*data).status as (i32) | buff[14usize] as (i32) & 0x10i32) as (u8);
            if (*data).status as (i32) & 0x80i32 != 0 {
                _currentBlock = 6;
                break;
            }
            (self.delay_ms)(10u32);
            tries = (tries as (i32) - 1) as (u8);
            if tries == 0 {
                _currentBlock = 7;
                break;
            }
        }
        if _currentBlock == 6 {
            (*data).temperature = self.calc_temperature(adc_temp);
            (*data).pressure = self.calc_pressure(adc_pres);
            (*data).humidity = self.calc_humidity(adc_hum);
            (*data).gas_resistance = self.calc_gas_resistance(adc_gas_res, gas_range);
        }
        if tries == 0 {
            rslt = 2i8;
        }
        rslt
    }

    fn boundary_check(self, mut value: *mut u8, mut min: u8, mut max: u8) -> i8 {
        let mut rslt: i8 = 0i8;
        if value != 0i32 as (*mut ::std::os::raw::c_void) as (*mut u8) {
            if *value as (i32) < min as (i32) {
                *value = min;
                self.info_msg = (self.info_msg as (i32) | 1i32) as (u8);
            }
            if *value as (i32) > max as (i32) {
                *value = max;
                self.info_msg = (self.info_msg as (i32) | 2i32) as (u8);
            }
        } else {
            rslt = -1i8;
        }
        rslt
    }
}
