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

#[derive(Clone, Copy)]
#[repr(i32)]
pub enum bme680_intf {
    BME680_SPI_INTF,
    BME680_I2C_INTF,
}

#[derive(Copy)]
#[repr(C)]
pub struct bme680_calib_data {
    pub par_h1 : u16,
    pub par_h2 : u16,
    pub par_h3 : i8,
    pub par_h4 : i8,
    pub par_h5 : i8,
    pub par_h6 : u8,
    pub par_h7 : i8,
    pub par_gh1 : i8,
    pub par_gh2 : i16,
    pub par_gh3 : i8,
    pub par_t1 : u16,
    pub par_t2 : i16,
    pub par_t3 : i8,
    pub par_p1 : u16,
    pub par_p2 : i16,
    pub par_p3 : i8,
    pub par_p4 : i16,
    pub par_p5 : i16,
    pub par_p6 : i8,
    pub par_p7 : i8,
    pub par_p8 : i16,
    pub par_p9 : i16,
    pub par_p10 : u8,
    pub t_fine : i32,
    pub res_heat_range : u8,
    pub res_heat_val : i8,
    pub range_sw_err : i8,
}

impl Clone for bme680_calib_data {
    fn clone(&self) -> Self { *self }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_tph_sett {
    pub os_hum : u8,
    pub os_temp : u8,
    pub os_pres : u8,
    pub filter : u8,
}

impl Clone for bme680_tph_sett {
    fn clone(&self) -> Self { *self }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_gas_sett {
    pub nb_conv : u8,
    pub heatr_ctrl : u8,
    pub run_gas : u8,
    pub heatr_temp : u16,
    pub heatr_dur : u16,
}

impl Clone for Bme680_gas_sett {
    fn clone(&self) -> Self { *self }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_field_data {
    pub status : u8,
    pub gas_index : u8,
    pub meas_index : u8,
    pub temperature : i16,
    pub pressure : u32,
    pub humidity : u32,
    pub gas_resistance : u32,
}

impl Clone for Bme680_field_data {
    fn clone(&self) -> Self { *self }
}

#[derive(Copy)]
#[repr(C)]
pub struct Bme680_dev<I2C, D> {
    pub i2c: I2C,
    pub delay: D,
    pub chip_id : u8,
    pub dev_id : u8,
    pub mem_page : u8,
    pub amb_temp : i8,
    pub calib : Bme680_calib_data,
    pub tph_sett : Bme680_tph_sett,
    pub gas_sett : Bme680_gas_sett,
    pub power_mode : PowerMode,
    pub new_fields : u8,
    pub info_msg : u8,
    pub com_rslt : i8,
}

impl Clone for Bme680_dev {
    fn clone(&self) -> Self { *self }
}

impl Bme680_dev {
    
pub fn init(self) -> Result<()> {
    rslt = bme680_get_regs(
               0xd0u8,
               &mut (*dev).chip_id as (*mut u8),
               1u16,
               dev
           );
    if rslt as (i32) == BME680_OK {
        if (*dev).chip_id as (i32) == 0x61i32 {
            rslt = get_calib_data(dev);
        } else {
            rslt = -3i8;
        }
    }
    rslt
}


pub fn bme680_get_regs(
    mut reg_addr : u8,
    mut reg_data : *mut u8,
    mut len : u16,
    mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    (*dev).com_rslt = ((*dev).read)(
                          (*dev).dev_id,
                          reg_addr,
                          reg_data,
                          len
                      );
    if (*dev).com_rslt as (i32) != 0i32 {
        rslt = -2i8;
    }
    rslt
}


pub fn bme680_set_regs(
    mut reg_addr : *const u8,
    mut reg_data : *const u8,
    mut len : u8,
    mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    let mut tmp_buff : [u8; 40] = 0i32 as ([u8; 40]);
    let mut index : u16;
    rslt = null_ptr_check(dev as (*const bme680_dev));
    if rslt as (i32) == BME680_OK {
        if len as (i32) > 0i32 && (len as (i32) < 40i32 / 2i32) {
            index = 0u16;
            'loop4: loop {
                if !(index as (i32) < len as (i32)) {
                    break;
                }
                if (*dev).intf as (i32) == bme680_intf::BME680_SPI_INTF as (i32) {
                    rslt = set_mem_page(*reg_addr.offset(index as (isize)),dev);
                    tmp_buff[(2i32 * index as (i32)) as (usize)] = (*reg_addr.offset(
                                                                         index as (isize)
                                                                     ) as (i32) & 0x7fi32) as (u8);
                } else {
                    tmp_buff[(2i32 * index as (i32)) as (usize)] = *reg_addr.offset(
                                                                        index as (isize)
                                                                    );
                }
                tmp_buff[
                    (2i32 * index as (i32) + 1i32) as (usize)
                ] = *reg_data.offset(index as (isize));
                index = (index as (i32) + 1) as (u16);
            }
            if rslt as (i32) == BME680_OK {
                (*dev).com_rslt = ((*dev).write)(
                                      (*dev).dev_id,
                                      tmp_buff[0usize],
                                      &mut tmp_buff[1usize] as (*mut u8),
                                      (2i32 * len as (i32) - 1i32) as (u16)
                                  );
                if (*dev).com_rslt as (i32) != 0i32 {
                    rslt = -2i8;
                }
            }
        } else {
            rslt = -4i8;
        }
    }
    rslt
}

pub fn soft_reset(
    delay: D
) -> Result<> where
    D: DelayMs<u8>,
{
    let mut rslt : i8;
    let mut reg_addr : u8 = BME680_SOFT_RESET_ADDR;
    let mut soft_rst_cmd : u8 = BME680_SOFT_RESET_CMD;
    rslt = bme680_set_regs(
               &mut reg_addr as (*mut u8) as (*const u8),
               &mut soft_rst_cmd as (*mut u8) as (*const u8),
               1u8,
               dev
           );
    delay.delay_ms(BME680_RESET_PERIOD);
    rslt
}


pub fn bme680_set_sensor_settings(
    mut desired_settings : u16, mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    let mut reg_addr : u8;
    let mut data : u8 = 0u8;
    let mut count : u8 = 0u8;
    let mut reg_array : [u8; 6] = 0i32 as ([u8; 6]);
    let mut data_array : [u8; 6] = 0i32 as ([u8; 6]);
    let mut intended_power_mode : u8 = (*dev).power_mode;
    rslt = null_ptr_check(dev as (*const bme680_dev));
    if rslt as (i32) == BME680_OK {
        if desired_settings as (i32) & 8i32 != 0 {
            rslt = set_gas_config(dev);
        }
        (*dev).power_mode = 0u8;
        if rslt as (i32) == BME680_OK {
            rslt = bme680_set_sensor_mode(dev);
        }
        if desired_settings as (i32) & 16i32 != 0 {
            rslt = boundary_check(
                       &mut (*dev).tph_sett.filter as (*mut u8),
                       0u8,
                       7u8,
                       dev
                   );
            reg_addr = 0x75u8;
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(reg_addr,&mut data as (*mut u8),1u16,dev);
            }
            if desired_settings as (i32) & 16i32 != 0 {
                data = (data as (i32) & !0x1ci32 | (*dev).tph_sett.filter as (i32) << 2i32 & 0x1ci32) as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & 32i32 != 0 {
            rslt = boundary_check(
                       &mut (*dev).gas_sett.heatr_ctrl as (*mut u8),
                       0x0u8,
                       0x8u8,
                       dev
                   );
            reg_addr = 0x70u8;
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(reg_addr,&mut data as (*mut u8),1u16,dev);
            }
            data = (data as (i32) & !0x8i32 | (*dev).gas_sett.heatr_ctrl as (i32) & 0x8i32) as (u8);
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & (1i32 | 2i32) != 0 {
            rslt = boundary_check(
                       &mut (*dev).tph_sett.os_temp as (*mut u8),
                       0u8,
                       5u8,
                       dev
                   );
            reg_addr = 0x74u8;
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(reg_addr,&mut data as (*mut u8),1u16,dev);
            }
            if desired_settings as (i32) & 1i32 != 0 {
                data = (data as (i32) & !0xe0i32 | (*dev).tph_sett.os_temp as (i32) << 5i32 & 0xe0i32) as (u8);
            }
            if desired_settings as (i32) & 2i32 != 0 {
                data = (data as (i32) & !0x1ci32 | (*dev).tph_sett.os_pres as (i32) << 2i32 & 0x1ci32) as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & 4i32 != 0 {
            rslt = boundary_check(
                       &mut (*dev).tph_sett.os_hum as (*mut u8),
                       0u8,
                       5u8,
                       dev
                   );
            reg_addr = 0x72u8;
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(reg_addr,&mut data as (*mut u8),1u16,dev);
            }
            data = (data as (i32) & !0x7i32 | (*dev).tph_sett.os_hum as (i32) & 0x7i32) as (u8);
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if desired_settings as (i32) & (64i32 | 128i32) != 0 {
            rslt = boundary_check(
                       &mut (*dev).gas_sett.run_gas as (*mut u8),
                       0u8,
                       1u8,
                       dev
                   );
            if rslt as (i32) == BME680_OK {
                rslt = boundary_check(
                           &mut (*dev).gas_sett.nb_conv as (*mut u8),
                           0u8,
                           10u8,
                           dev
                       );
            }
            reg_addr = 0x71u8;
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(reg_addr,&mut data as (*mut u8),1u16,dev);
            }
            if desired_settings as (i32) & 64i32 != 0 {
                data = (data as (i32) & !0x10i32 | (*dev).gas_sett.run_gas as (i32) << 4i32 & 0x10i32) as (u8);
            }
            if desired_settings as (i32) & 128i32 != 0 {
                data = (data as (i32) & !0xfi32 | (*dev).gas_sett.nb_conv as (i32) & 0xfi32) as (u8);
            }
            reg_array[count as (usize)] = reg_addr;
            data_array[count as (usize)] = data;
            count = (count as (i32) + 1) as (u8);
        }
        if rslt as (i32) == BME680_OK {
            rslt = bme680_set_regs(
                       reg_array.as_mut_ptr() as (*const u8),
                       data_array.as_mut_ptr() as (*const u8),
                       count,
                       dev
                   );
        }
        (*dev).power_mode = intended_power_mode;
    }
    rslt
}


pub fn bme680_get_sensor_settings(
    mut desired_settings : u16, mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    let mut reg_addr : u8 = 0x70u8;
    let mut data_array : [u8; 6] = 0i32 as ([u8; 6]);
    rslt = null_ptr_check(dev as (*const bme680_dev));
    if rslt as (i32) == BME680_OK {
        rslt = bme680_get_regs(reg_addr,data_array.as_mut_ptr(),6u16,dev);
        if rslt as (i32) == BME680_OK {
            if desired_settings as (i32) & 8i32 != 0 {
                rslt = get_gas_config(dev);
            }
            if desired_settings as (i32) & 16i32 != 0 {
                (*dev).tph_sett.filter = ((data_array[
                                               5usize
                                           ] as (i32) & 0x1ci32) >> 2i32) as (u8);
            }
            if desired_settings as (i32) & (1i32 | 2i32) != 0 {
                (*dev).tph_sett.os_temp = ((data_array[
                                                4usize
                                            ] as (i32) & 0xe0i32) >> 5i32) as (u8);
                (*dev).tph_sett.os_pres = ((data_array[
                                                4usize
                                            ] as (i32) & 0x1ci32) >> 2i32) as (u8);
            }
            if desired_settings as (i32) & 4i32 != 0 {
                (*dev).tph_sett.os_hum = (data_array[
                                              2usize
                                          ] as (i32) & 0x7i32) as (u8);
            }
            if desired_settings as (i32) & 32i32 != 0 {
                (*dev).gas_sett.heatr_ctrl = (data_array[
                                                  0usize
                                              ] as (i32) & 0x8i32) as (u8);
            }
            if desired_settings as (i32) & (64i32 | 128i32) != 0 {
                (*dev).gas_sett.nb_conv = (data_array[
                                               1usize
                                           ] as (i32) & 0xfi32) as (u8);
                (*dev).gas_sett.run_gas = ((data_array[
                                                1usize
                                            ] as (i32) & 0x10i32) >> 4i32) as (u8);
            }
        }
    } else {
        rslt = -1i8;
    }
    rslt
}


pub fn bme680_set_sensor_mode(
    mut dev : *mut bme680_dev
) -> i8 {
    let mut _currentBlock;
    let mut rslt : i8;
    let mut tmp_pow_mode : u8;
    let mut pow_mode : u8 = 0u8;
    let mut reg_addr : u8 = 0x74u8;
    rslt = null_ptr_check(dev as (*const bme680_dev));
    if rslt as (i32) == BME680_OK {
        _currentBlock = 1;
    } else {
        _currentBlock = 8;
    }
    'loop1: loop {
        if _currentBlock == 1 {
            rslt = bme680_get_regs(
                       0x74u8,
                       &mut tmp_pow_mode as (*mut u8),
                       1u16,
                       dev
                   );
            if rslt as (i32) == BME680_OK {
                pow_mode = (tmp_pow_mode as (i32) & 0x3i32) as (u8);
                if pow_mode as (i32) != 0i32 {
                    tmp_pow_mode = (tmp_pow_mode as (i32) & !0x3i32) as (u8);
                    rslt = bme680_set_regs(
                               &mut reg_addr as (*mut u8) as (*const u8),
                               &mut tmp_pow_mode as (*mut u8) as (*const u8),
                               1u8,
                               dev
                           );
                    ((*dev).delay_ms)(10u32);
                }
            }
            if pow_mode as (i32) != 0i32 {
                _currentBlock = 1;
                continue;
            }
            if !((*dev).power_mode as (i32) != 0i32) {
                _currentBlock = 8;
                continue;
            }
            tmp_pow_mode = (tmp_pow_mode as (i32) & !0x3i32 | (*dev).power_mode as (i32) & 0x3i32) as (u8);
            if !(rslt as (i32) == BME680_OK) {
                _currentBlock = 8;
                continue;
            }
            rslt = bme680_set_regs(
                       &mut reg_addr as (*mut u8) as (*const u8),
                       &mut tmp_pow_mode as (*mut u8) as (*const u8),
                       1u8,
                       dev
                   );
            _currentBlock = 8;
        } else {
            return rslt;
        }
    }
}


pub fn bme680_get_sensor_mode(
    mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    let mut mode : u8;
    rslt = bme680_get_regs(0x74u8,&mut mode as (*mut u8),1u16,dev);
    (*dev).power_mode = (mode as (i32) & 0x3i32) as (u8);
    rslt
}


pub fn bme680_set_profile_dur(
    mut duration : u16, mut dev : *mut bme680_dev
) {
    let mut tph_dur : u32;
    let mut meas_cycles : u32;
    let mut os_to_meas_cycles
        : [u8; 6]
        = [ 0u8, 1u8, 2u8, 4u8, 8u8, 16u8 ];
    meas_cycles = os_to_meas_cycles[
                      (*dev).tph_sett.os_temp as (usize)
                  ] as (u32);
    meas_cycles = meas_cycles.wrapping_add(
                      os_to_meas_cycles[(*dev).tph_sett.os_pres as (usize)] as (u32)
                  );
    meas_cycles = meas_cycles.wrapping_add(
                      os_to_meas_cycles[(*dev).tph_sett.os_hum as (usize)] as (u32)
                  );
    tph_dur = meas_cycles.wrapping_mul(1963u32);
    tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
    tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
    tph_dur = tph_dur.wrapping_add(500u32);
    tph_dur = tph_dur.wrapping_div(1000u32);
    tph_dur = tph_dur.wrapping_add(1u32);
    (*dev).gas_sett.heatr_dur = (duration as (i32) - tph_dur as (u16) as (i32)) as (u16);
}


pub fn bme680_get_profile_dur(
    mut duration : *mut u16, mut dev : *const bme680_dev
) {
    let mut tph_dur : u32;
    let mut meas_cycles : u32;
    let mut os_to_meas_cycles
        : [u8; 6]
        = [ 0u8, 1u8, 2u8, 4u8, 8u8, 16u8 ];
    meas_cycles = os_to_meas_cycles[
                      (*dev).tph_sett.os_temp as (usize)
                  ] as (u32);
    meas_cycles = meas_cycles.wrapping_add(
                      os_to_meas_cycles[(*dev).tph_sett.os_pres as (usize)] as (u32)
                  );
    meas_cycles = meas_cycles.wrapping_add(
                      os_to_meas_cycles[(*dev).tph_sett.os_hum as (usize)] as (u32)
                  );
    tph_dur = meas_cycles.wrapping_mul(1963u32);
    tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(4u32));
    tph_dur = tph_dur.wrapping_add(477u32.wrapping_mul(5u32));
    tph_dur = tph_dur.wrapping_add(500u32);
    tph_dur = tph_dur.wrapping_div(1000u32);
    tph_dur = tph_dur.wrapping_add(1u32);
    *duration = tph_dur as (u16);
    if (*dev).gas_sett.run_gas != 0 {
        *duration = (*duration as (i32) + (*dev).gas_sett.heatr_dur as (i32)) as (u16);
    }
}

pub fn bme680_get_sensor_data(
    mut data : *mut bme680_field_data, mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8;
    rslt = read_field_data(data,dev);
    if rslt as (i32) == BME680_OK {
        if (*data).status as (i32) & 0x80i32 != 0 {
            (*dev).new_fields = 1u8;
        } else {
            (*dev).new_fields = 0u8;
        }
    }
    rslt
}

fn get_calib_data(mut dev : *mut bme680_dev) -> i8 {
    let mut rslt : i8;
    let mut coeff_array : [u8; 41] = 0i32 as ([u8; 41]);
    let mut temp_var : u8 = 0u8;
        rslt = bme680_get_regs(0x89u8,coeff_array.as_mut_ptr(),25u16,dev);
        if rslt as (i32) == BME680_OK {
            rslt = bme680_get_regs(
                       0xe1u8,
                       &mut coeff_array[25usize] as (*mut u8),
                       16u16,
                       dev
                   );
        }
        (*dev).calib.par_t1 = (coeff_array[
                                   34usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 33usize
                                                             ] as (u16) as (i32)) as (u16);
        (*dev).calib.par_t2 = (coeff_array[
                                   2usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 1usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_t3 = coeff_array[3usize] as (i8);
        (*dev).calib.par_p1 = (coeff_array[
                                   6usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 5usize
                                                             ] as (u16) as (i32)) as (u16);
        (*dev).calib.par_p2 = (coeff_array[
                                   8usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 7usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_p3 = coeff_array[9usize] as (i8);
        (*dev).calib.par_p4 = (coeff_array[
                                   12usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 11usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_p5 = (coeff_array[
                                   14usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 13usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_p6 = coeff_array[16usize] as (i8);
        (*dev).calib.par_p7 = coeff_array[15usize] as (i8);
        (*dev).calib.par_p8 = (coeff_array[
                                   20usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 19usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_p9 = (coeff_array[
                                   22usize
                               ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                 21usize
                                                             ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_p10 = coeff_array[23usize];
        (*dev).calib.par_h1 = (coeff_array[
                                   27usize
                               ] as (u16) as (i32) << 4i32 | coeff_array[
                                                                 26usize
                                                             ] as (i32) & 0xfi32) as (u16);
        (*dev).calib.par_h2 = (coeff_array[
                                   25usize
                               ] as (u16) as (i32) << 4i32 | coeff_array[
                                                                 26usize
                                                             ] as (i32) >> 4i32) as (u16);
        (*dev).calib.par_h3 = coeff_array[28usize] as (i8);
        (*dev).calib.par_h4 = coeff_array[29usize] as (i8);
        (*dev).calib.par_h5 = coeff_array[30usize] as (i8);
        (*dev).calib.par_h6 = coeff_array[31usize];
        (*dev).calib.par_h7 = coeff_array[32usize] as (i8);
        (*dev).calib.par_gh1 = coeff_array[37usize] as (i8);
        (*dev).calib.par_gh2 = (coeff_array[
                                    36usize
                                ] as (u16) as (i32) << 8i32 | coeff_array[
                                                                  35usize
                                                              ] as (u16) as (i32)) as (i16);
        (*dev).calib.par_gh3 = coeff_array[38usize] as (i8);
        if rslt as (i32) == BME680_OK {
            rslt = bme680_get_regs(0x2u8,&mut temp_var as (*mut u8),1u16,dev);
            (*dev).calib.res_heat_range = ((temp_var as (i32) & 0x30i32) / 16i32) as (u8);
            if rslt as (i32) == BME680_OK {
                rslt = bme680_get_regs(0x0u8,&mut temp_var as (*mut u8),1u16,dev);
                (*dev).calib.res_heat_val = temp_var as (i8);
                if rslt as (i32) == BME680_OK {
                    rslt = bme680_get_regs(0x4u8,&mut temp_var as (*mut u8),1u16,dev);
                }
            }
        }
        (*dev).calib.range_sw_err = ((temp_var as (i8) as (i32) & 0xf0i32) / 16i32) as (i8);
    rslt
}

fn set_gas_config(self) -> i8 {
    let mut rslt : i8;
        let mut reg_addr : [u8; 2] = 0i32 as ([u8; 2]);
        let mut reg_data : [u8; 2] = 0i32 as ([u8; 2]);
        if (*dev).power_mode as (i32) == 1i32 {
            reg_addr[0usize] = 0x5au8;
            reg_data[0usize] = calc_heater_res(
                                   (*dev).gas_sett.heatr_temp,
                                   dev as (*const bme680_dev)
                               );
            reg_addr[1usize] = 0x64u8;
            reg_data[1usize] = calc_heater_dur((*dev).gas_sett.heatr_dur);
            (*dev).gas_sett.nb_conv = 0u8;
        } else {
            rslt = 1i8;
        }
        if rslt as (i32) == BME680_OK {
            rslt = bme680_set_regs(
                       reg_addr.as_mut_ptr() as (*const u8),
                       reg_data.as_mut_ptr() as (*const u8),
                       2u8,
                       dev
                   );
        }
    rslt
}

fn get_gas_config(self) -> i8 {
    let mut rslt : i8;
    let mut reg_addr1 : u8 = 0x5au8;
    let mut reg_addr2 : u8 = 0x64u8;
    let mut reg_data : u8 = 0u8;
    rslt = bme680_get_regs(
               reg_addr1,
               &mut reg_data as (*mut u8),
               1u16,
               dev
           );
        (*dev).gas_sett.heatr_temp = reg_data as (u16);
        rslt = bme680_get_regs(
                   reg_addr2,
                   &mut reg_data as (*mut u8),
                   1u16,
                   dev
               );
        if rslt as (i32) == BME680_OK {
            (*dev).gas_sett.heatr_dur = reg_data as (u16);
        }
    rslt
}

fn calc_heater_res(
    mut temp : u16, mut dev : *const bme680_dev
) -> u8 {
    let mut heatr_res : u8;
    let mut var1 : i32;
    let mut var2 : i32;
    let mut var3 : i32;
    let mut var4 : i32;
    let mut var5 : i32;
    let mut heatr_res_x100 : i32;
    if temp as (i32) > 400i32 {
        temp = 400u16;
    }
    var1 = (*dev).amb_temp as (i32) * (*dev).calib.par_gh3 as (i32) / 1000i32 * 256i32;
    var2 = ((*dev).calib.par_gh1 as (i32) + 784i32) * ((((*dev).calib.par_gh2 as (i32) + 154009i32) * temp as (i32) * 5i32 / 100i32 + 3276800i32) / 10i32);
    var3 = var1 + var2 / 2i32;
    var4 = var3 / ((*dev).calib.res_heat_range as (i32) + 4i32);
    var5 = 131i32 * (*dev).calib.res_heat_val as (i32) + 65536i32;
    heatr_res_x100 = (var4 / var5 - 250i32) * 34i32;
    heatr_res = ((heatr_res_x100 + 50i32) / 100i32) as (u8);
    heatr_res
}

fn calc_heater_dur(mut dur : u16) -> u8 {
    let mut factor : u8 = 0u8;
    let mut durval : u8;
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

fn calc_temperature(
    mut temp_adc : u32, mut dev : *mut bme680_dev
) -> i16 {
    let mut var1 : isize;
    let mut var2 : isize;
    let mut var3 : isize;
    let mut calc_temp : i16;
    var1 = ((temp_adc as (i32) >> 3i32) - ((*dev).calib.par_t1 as (i32) << 1i32)) as (isize);
    var2 = var1 * (*dev).calib.par_t2 as (i32) as (isize) >> 11i32;
    var3 = (var1 >> 1i32) * (var1 >> 1i32) >> 12i32;
    var3 = var3 * ((*dev).calib.par_t3 as (i32) << 4i32) as (isize) >> 14i32;
    (*dev).calib.t_fine = (var2 + var3) as (i32);
    calc_temp = ((*dev).calib.t_fine * 5i32 + 128i32 >> 8i32) as (i16);
    calc_temp
}

fn calc_pressure(
    mut pres_adc : u32, mut dev : *const bme680_dev
) -> u32 {
    let mut var1 : i32 = 0i32;
    let mut var2 : i32 = 0i32;
    let mut var3 : i32 = 0i32;
    let mut pressure_comp : i32 = 0i32;
    var1 = ((*dev).calib.t_fine >> 1i32) - 64000i32;
    var2 = ((var1 >> 2i32) * (var1 >> 2i32) >> 11i32) * (*dev).calib.par_p6 as (i32) >> 2i32;
    var2 = var2 + (var1 * (*dev).calib.par_p5 as (i32) << 1i32);
    var2 = (var2 >> 2i32) + ((*dev).calib.par_p4 as (i32) << 16i32);
    var1 = (((var1 >> 2i32) * (var1 >> 2i32) >> 13i32) * ((*dev).calib.par_p3 as (i32) << 5i32) >> 3i32) + ((*dev).calib.par_p2 as (i32) * var1 >> 1i32);
    var1 = var1 >> 18i32;
    var1 = (32768i32 + var1) * (*dev).calib.par_p1 as (i32) >> 15i32;
    pressure_comp = 1048576u32.wrapping_sub(pres_adc) as (i32);
    pressure_comp = ((pressure_comp - (var2 >> 12i32)) as (u32)).wrapping_mul(
                        3125u32
                    ) as (i32);
    if pressure_comp >= 0x40000000i32 {
        pressure_comp = ((pressure_comp as (u32)).wrapping_div(
                             var1 as (u32)
                         ) << 1i32) as (i32);
    } else {
        pressure_comp = ((pressure_comp << 1i32) as (u32)).wrapping_div(
                            var1 as (u32)
                        ) as (i32);
    }
    var1 = (*dev).calib.par_p9 as (i32) * ((pressure_comp >> 3i32) * (pressure_comp >> 3i32) >> 13i32) >> 12i32;
    var2 = (pressure_comp >> 2i32) * (*dev).calib.par_p8 as (i32) >> 13i32;
    var3 = (pressure_comp >> 8i32) * (pressure_comp >> 8i32) * (pressure_comp >> 8i32) * (*dev).calib.par_p10 as (i32) >> 17i32;
    pressure_comp = pressure_comp + (var1 + var2 + var3 + ((*dev).calib.par_p7 as (i32) << 7i32) >> 4i32);
    pressure_comp as (u32)
}

fn calc_humidity(
    mut hum_adc : u16, mut dev : *const bme680_dev
) -> u32 {
    let mut var1 : i32;
    let mut var2 : i32;
    let mut var3 : i32;
    let mut var4 : i32;
    let mut var5 : i32;
    let mut var6 : i32;
    let mut temp_scaled : i32;
    let mut calc_hum : i32;
    temp_scaled = (*dev).calib.t_fine * 5i32 + 128i32 >> 8i32;
    var1 = hum_adc as (i32) - (*dev).calib.par_h1 as (i32) * 16i32 - (temp_scaled * (*dev).calib.par_h3 as (i32) / 100i32 >> 1i32);
    var2 = (*dev).calib.par_h2 as (i32) * (temp_scaled * (*dev).calib.par_h4 as (i32) / 100i32 + (temp_scaled * (temp_scaled * (*dev).calib.par_h5 as (i32) / 100i32) >> 6i32) / 100i32 + (1i32 << 14i32)) >> 10i32;
    var3 = var1 * var2;
    var4 = (*dev).calib.par_h6 as (i32) << 7i32;
    var4 = var4 + temp_scaled * (*dev).calib.par_h7 as (i32) / 100i32 >> 4i32;
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

fn calc_gas_resistance(
    mut gas_res_adc : u16,
    mut gas_range : u8,
    mut dev : *const bme680_dev
) -> u32 {
    let mut var1 : isize;
    let mut var2 : usize;
    let mut var3 : isize;
    let mut calc_gas_res : u32;
    let mut lookupTable1
        : [u32; 16]
        = [   2147483647u32,
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
              2147483647u32
          ];
    let mut lookupTable2
        : [u32; 16]
        = [   4096000000u32,
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
              125000u32
          ];
    var1 = (1340isize + 5isize * (*dev).calib.range_sw_err as (isize)) * lookupTable1[
                                                                             gas_range as (usize)
                                                                         ] as (isize) >> 16i32;
    var2 = ((gas_res_adc as (isize) << 15i32) - 16777216isize + var1) as (usize);
    var3 = lookupTable2[
               gas_range as (usize)
           ] as (isize) * var1 >> 9i32;
    calc_gas_res = ((var3 + (var2 as (isize) >> 1i32)) / var2 as (isize)) as (u32);
    calc_gas_res
}

fn read_field_data(
    mut data : *mut bme680_field_data, mut dev : *mut bme680_dev
) -> i8 {
    let mut _currentBlock;
    let mut rslt : i8;
    let mut buff : [u8; 15] = 0i32 as ([u8; 15]);
    let mut gas_range : u8;
    let mut adc_temp : u32;
    let mut adc_pres : u32;
    let mut adc_hum : u16;
    let mut adc_gas_res : u16;
    let mut tries : u8 = 10u8;
    rslt = null_ptr_check(dev as (*const bme680_dev));
    'loop1: loop {
        if rslt as (i32) == BME680_OK {
            rslt = bme680_get_regs(0x1du8,buff.as_mut_ptr(),15u16,dev);
            (*data).status = (buff[0usize] as (i32) & 0x80i32) as (u8);
            (*data).gas_index = (buff[0usize] as (i32) & 0xfi32) as (u8);
            (*data).meas_index = buff[1usize];
            adc_pres = (buff[2usize] as (u32)).wrapping_mul(4096u32) | (buff[
                                                                            3usize
                                                                        ] as (u32)).wrapping_mul(
                                                                           16u32
                                                                       ) | (buff[
                                                                                4usize
                                                                            ] as (u32)).wrapping_div(
                                                                               16u32
                                                                           );
            adc_temp = (buff[5usize] as (u32)).wrapping_mul(4096u32) | (buff[
                                                                            6usize
                                                                        ] as (u32)).wrapping_mul(
                                                                           16u32
                                                                       ) | (buff[
                                                                                7usize
                                                                            ] as (u32)).wrapping_div(
                                                                               16u32
                                                                           );
            adc_hum = ((buff[8usize] as (u32)).wrapping_mul(256u32) | buff[
                                                                          9usize
                                                                      ] as (u32)) as (u16);
            adc_gas_res = ((buff[13usize] as (u32)).wrapping_mul(4u32) | (buff[
                                                                              14usize
                                                                          ] as (u32)).wrapping_div(
                                                                             64u32
                                                                         )) as (u16);
            gas_range = (buff[14usize] as (i32) & 0xfi32) as (u8);
            (*data).status = ((*data).status as (i32) | buff[
                                                            14usize
                                                        ] as (i32) & 0x20i32) as (u8);
            (*data).status = ((*data).status as (i32) | buff[
                                                            14usize
                                                        ] as (i32) & 0x10i32) as (u8);
            if (*data).status as (i32) & 0x80i32 != 0 {
                _currentBlock = 6;
                break;
            }
            ((*dev).delay_ms)(10u32);
        }
        tries = (tries as (i32) - 1) as (u8);
        if tries == 0 {
            _currentBlock = 7;
            break;
        }
    }
    if _currentBlock == 6 {
        (*data).temperature = calc_temperature(adc_temp,dev);
        (*data).pressure = calc_pressure(
                               adc_pres,
                               dev as (*const bme680_dev)
                           );
        (*data).humidity = calc_humidity(
                               adc_hum,
                               dev as (*const bme680_dev)
                           );
        (*data).gas_resistance = calc_gas_resistance(
                                     adc_gas_res,
                                     gas_range,
                                     dev as (*const bme680_dev)
                                 );
    }
    if tries == 0 {
        rslt = 2i8;
    }
    rslt
}

fn boundary_check(
    mut value : *mut u8,
    mut min : u8,
    mut max : u8,
    mut dev : *mut bme680_dev
) -> i8 {
    let mut rslt : i8 = 0i8;
    if value != 0i32 as (*mut ::std::os::raw::c_void) as (*mut u8) {
        if *value as (i32) < min as (i32) {
            *value = min;
            (*dev).info_msg = ((*dev).info_msg as (i32) | 1i32) as (u8);
        }
        if *value as (i32) > max as (i32) {
            *value = max;
            (*dev).info_msg = ((*dev).info_msg as (i32) | 2i32) as (u8);
        }
    } else {
        rslt = -1i8;
    }
    rslt
}

}
