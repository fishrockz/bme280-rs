#![doc(html_root_url = "https://docs.rs/bme280")]
#![doc(issue_tracker_base_url = "https://github.com/uber-foo/bme280/issues/")]
#![deny(
    missing_docs,
    missing_debug_implementations,
    missing_copy_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    unused_variables,
    unreachable_code,
    unused_comparisons,
    unused_imports,
    unused_must_use
)]
#![no_std]

//! A platform agnostic Rust driver for the Bosch BME280 and BMP280, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## The Device
//!
//! The [Bosch BME280](https://www.bosch-sensortec.com/bst/products/all_products/bme280)
//! is a highly accurate sensor for atmospheric temperature, pressure, and
//! relative humidity.
//!
//! The [Bosch BMP280](https://www.bosch-sensortec.com/bst/products/all_products/bmp280)
//! is a highly accurate sensor for atmospheric temperature, and pressure.
//!
//! The device has I²C and SPI interfaces.
//!
//! ## Usage
//!
//! ```no_run
//! use linux_embedded_hal::{Delay, I2cdev};
//! use bme280::i2c::BME280;
//!
//! // using Linux I2C Bus #1 in this example
//! let i2c_bus = I2cdev::new("/dev/i2c-1").unwrap();
//!
//! // initialize the BME280 using the primary I2C address 0x76
//! let mut bme280 = BME280::new_primary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using the secondary I2C address 0x77
//! // let mut bme280 = BME280::new_secondary(i2c_bus, Delay);
//!
//! // or, initialize the BME280 using a custom I2C address
//! // let bme280_i2c_addr = 0x88;
//! // let mut bme280 = BME280::new(i2c_bus, bme280_i2c_addr, Delay);
//!
//! // initialize the sensor
//! bme280.init().unwrap();
//!
//! // measure temperature, pressure, and humidity
//! let measurements = bme280.measure().unwrap();
//!
//! println!("Relative Humidity = {}%", measurements.humidity);
//! println!("Temperature = {} deg C", measurements.temperature);
//! println!("Pressure = {} pascals", measurements.pressure);
//! ```

pub mod i2c;
//pub mod spi;

use bit_byte_structs::registers::{BitByteStructError, BitStruct};
use core::marker::PhantomData;
use embedded_hal::blocking::delay::DelayMs;

use bit_byte_structs::bus::{Interface, InterfaceError};

#[cfg(feature = "serde")]
use serde::Serialize;

const BME280_PWR_CTRL_ADDR: u8 = 0xF4;
const BME280_CTRL_HUM_ADDR: u8 = 0xF2;
const BME280_CTRL_MEAS_ADDR: u8 = 0xF4;
const BME280_CONFIG_ADDR: u8 = 0xF5;

const BME280_RESET_ADDR: u8 = 0xE0;
const BME280_SOFT_RESET_CMD: u8 = 0xB6;

const BME280_CHIP_ID: u8 = 0x60;
const BMP280_CHIP_ID: u8 = 0x58;
const BME280_CHIP_ID_ADDR: u8 = 0xD0;

const BME280_DATA_ADDR: u8 = 0xF7;
const BME280_P_T_H_DATA_LEN: usize = 8;

const BME280_P_T_CALIB_DATA_ADDR: u8 = 0x88;
const BME280_P_T_CALIB_DATA_LEN: usize = 26;

const BME280_H_CALIB_DATA_ADDR: u8 = 0xE1;
const BME280_H_CALIB_DATA_LEN: usize = 7;

const BME280_TEMP_MIN: f32 = -40.0;
const BME280_TEMP_MAX: f32 = 85.0;

const BME280_PRESSURE_MIN: f32 = 30000.0;
const BME280_PRESSURE_MAX: f32 = 110000.0;

const BME280_HUMIDITY_MIN: f32 = 0.0;
const BME280_HUMIDITY_MAX: f32 = 100.0;

const BME280_SLEEP_MODE: u8 = 0x00;
const BME280_FORCED_MODE: u8 = 0x01;
const BME280_NORMAL_MODE: u8 = 0x03;

const BME280_SENSOR_MODE_MSK: u8 = 0x03;

const BME280_CTRL_HUM_MSK: u8 = 0x07;

const BME280_CTRL_PRESS_MSK: u8 = 0x1C;
const BME280_CTRL_PRESS_POS: u8 = 0x02;

const BME280_CTRL_TEMP_MSK: u8 = 0xE0;
const BME280_CTRL_TEMP_POS: u8 = 0x05;

const BME280_FILTER_MSK: u8 = 0x1C;
const BME280_FILTER_POS: u8 = 0x02;
const BME280_FILTER_COEFF_16: u8 = 0x04;

const BME280_OVERSAMPLING_1X: u8 = 0x01;
const BME280_OVERSAMPLING_2X: u8 = 0x02;
const BME280_OVERSAMPLING_16X: u8 = 0x05;

macro_rules! concat_bytes {
    ($msb:expr, $lsb:expr) => {
        (($msb as u16) << 8) | ($lsb as u16)
    };
}

macro_rules! set_bits {
    ($reg_data:expr, $mask:expr, $pos:expr, $data:expr) => {
        ($reg_data & !$mask) | (($data << $pos) & $mask)
    };
}

/// BME280 errors
#[derive(Debug)]
pub enum Bme280Error<E> {
    /// Failed to compensate a raw measurement
    CompensationFailed,
    /// I²C or SPI bus error
    Bus(E),
    /// BitByte error
    BitByte(BitByteStructError<InterfaceError<E>>),
    /// Interface error
    Interface(InterfaceError<E>),
    /// Failed to parse sensor data
    InvalidData,
    /// No calibration data is available (probably forgot to call or check BME280::init for failure)
    NoCalibrationData,
    /// Chip ID doesn't match expected value
    UnsupportedChip,
}

impl<E> From<BitByteStructError<InterfaceError<E>>> for Bme280Error<E> {
    fn from(err: BitByteStructError<InterfaceError<E>>) -> Self {
        Bme280Error::BitByte(err)
    }
}
impl<E> From<InterfaceError<E>> for Bme280Error<E> {
    fn from(err: InterfaceError<E>) -> Self {
        Bme280Error::Interface(err)
    }
}
impl<E> From<E> for Bme280Error<E> {
    fn from(err: E) -> Self {
        Bme280Error::Bus(err)
    }
}
/// BME280 operating mode
#[derive(Debug, Copy, Clone)]
pub enum SensorMode {
    /// Sleep mode
    Sleep,
    /// Forced mode
    Forced,
    /// Normal mode
    Normal,
}

#[derive(Debug)]
struct CalibrationData {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
    t_fine: i32,
}

/// Measurement data
#[cfg_attr(feature = "serde", derive(Serialize))]
#[derive(Debug)]
pub struct Measurements<E> {
    /// temperature in degrees celsius
    pub temperature: f32,
    /// pressure in pascals
    pub pressure: f32,
    /// percent relative humidity (`0` with BMP280)
    pub humidity: f32,
    #[cfg_attr(feature = "serde", serde(skip))]
    _e: PhantomData<E>,
}

impl<E> Measurements<E> {
    fn parse(
        data: [u8; BME280_P_T_H_DATA_LEN],
        calibration: &mut CalibrationData,
    ) -> Result<Self, Bme280Error<E>> {
        let data_msb: u32 = (data[0] as u32) << 12;
        let data_lsb: u32 = (data[1] as u32) << 4;
        let data_xlsb: u32 = (data[2] as u32) >> 4;
        let pressure = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[3] as u32) << 12;
        let data_lsb: u32 = (data[4] as u32) << 4;
        let data_xlsb: u32 = (data[5] as u32) >> 4;
        let temperature = data_msb | data_lsb | data_xlsb;

        let data_msb: u32 = (data[6] as u32) << 8;
        let data_lsb: u32 = data[7] as u32;
        let humidity = data_msb | data_lsb;

        let temperature = Measurements::compensate_temperature(temperature, calibration)?;
        let pressure = Measurements::compensate_pressure(pressure, calibration)?;
        let humidity = Measurements::compensate_humidity(humidity, calibration)?;

        Ok(Measurements {
            temperature,
            pressure,
            humidity,
            _e: PhantomData,
        })
    }

    fn compensate_temperature(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Bme280Error<E>> {
        let var1: f32 = uncompensated as f32 / 16384.0 - calibration.dig_t1 as f32 / 1024.0;
        let var1 = var1 * calibration.dig_t2 as f32;
        let var2 = uncompensated as f32 / 131072.0 - calibration.dig_t1 as f32 / 8192.0;
        let var2 = var2 * var2 * calibration.dig_t3 as f32;

        calibration.t_fine = (var1 + var2) as i32;

        let temperature = (var1 + var2) / 5120.0;
        let temperature = if temperature < BME280_TEMP_MIN {
            BME280_TEMP_MIN
        } else if temperature > BME280_TEMP_MAX {
            BME280_TEMP_MAX
        } else {
            temperature
        };
        Ok(temperature)
    }

    fn compensate_pressure(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Bme280Error<E>> {
        let var1: f32 = calibration.t_fine as f32 / 2.0 - 64000.0;
        let var2: f32 = var1 * var1 * calibration.dig_p6 as f32 / 32768.0;
        let var2: f32 = var2 + var1 * calibration.dig_p5 as f32 * 2.0;
        let var2: f32 = var2 / 4.0 + calibration.dig_p4 as f32 * 65536.0;
        let var3: f32 = calibration.dig_p3 as f32 * var1 * var1 / 524288.0;
        let var1: f32 = (var3 + calibration.dig_p2 as f32 * var1) / 524288.0;
        let var1: f32 = (1.0 + var1 / 32768.0) * calibration.dig_p1 as f32;

        let pressure = if var1 > 0.0 {
            let pressure: f32 = 1048576.0 - uncompensated as f32;
            let pressure: f32 = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
            let var1: f32 = calibration.dig_p9 as f32 * pressure * pressure / 2147483648.0;
            let var2: f32 = pressure * calibration.dig_p8 as f32 / 32768.0;
            let pressure: f32 = pressure + (var1 + var2 + calibration.dig_p7 as f32) / 16.0;
            if pressure < BME280_PRESSURE_MIN {
                BME280_PRESSURE_MIN
            } else if pressure > BME280_PRESSURE_MAX {
                BME280_PRESSURE_MAX
            } else {
                pressure
            }
        } else {
            return Err(Bme280Error::InvalidData);
        };
        Ok(pressure)
    }

    fn compensate_humidity(
        uncompensated: u32,
        calibration: &mut CalibrationData,
    ) -> Result<f32, Bme280Error<E>> {
        let var1: f32 = calibration.t_fine as f32 - 76800.0;
        let var2: f32 =
            calibration.dig_h4 as f32 * 64.0 + (calibration.dig_h5 as f32 / 16384.0) * var1;
        let var3: f32 = uncompensated as f32 - var2;
        let var4: f32 = calibration.dig_h2 as f32 / 65536.0;
        let var5: f32 = 1.0 + (calibration.dig_h3 as f32 / 67108864.0) * var1;
        let var6: f32 = 1.0 + (calibration.dig_h6 as f32 / 67108864.0) * var1 * var5;
        let var6: f32 = var3 * var4 * (var5 * var6);

        let humidity: f32 = var6 * (1.0 - calibration.dig_h1 as f32 * var6 / 524288.0);
        let humidity = if humidity < BME280_HUMIDITY_MIN {
            BME280_HUMIDITY_MIN
        } else if humidity > BME280_HUMIDITY_MAX {
            BME280_HUMIDITY_MAX
        } else {
            humidity
        };
        Ok(humidity)
    }
}

/// Common driver code for I2C and SPI interfaces
//#[derive(Debug)]
struct BME280Common<InterfaceThing: ?Sized, E> {
    phantom: PhantomData<InterfaceThing>,
    chip_id: BitStruct<dyn Interface<Error = InterfaceError<E>>>,
    /// calibration data
    calibration: Option<CalibrationData>,
}

impl<I, E> BME280Common<I, E>
where
    I: Interface,
    I: ?Sized,
{
    fn new() -> Result<Self, Bme280Error<E>> {
        let chip_id =
            BitStruct::<dyn Interface<Error = InterfaceError<E>>>::new(BME280_CHIP_ID_ADDR, 8, 0)?;
        Ok(Self {
            phantom: PhantomData,
            chip_id,
            calibration: None,
        })
    }
    /// Initializes the BME280
    fn init(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,

        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        self.verify_chip_id(interface)?;
        self.soft_reset(interface, delay)?;
        self.calibrate(interface)?;
        self.configure(interface, delay)
    }

    fn verify_chip_id(
        &self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
    ) -> Result<(), Bme280Error<E>> {
        let chip_id = self.chip_id.read(interface)?;
        if chip_id == BME280_CHIP_ID || chip_id == BMP280_CHIP_ID {
            Ok(())
        } else {
            Err(Bme280Error::UnsupportedChip)
        }
    }

    fn soft_reset(
        &self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        interface.write_register(BME280_RESET_ADDR, &[BME280_SOFT_RESET_CMD])?;
        delay.delay_ms(2); // startup time is 2ms
        Ok(())
    }

    fn calibrate(
        &mut self,

        interface: &mut dyn Interface<Error = InterfaceError<E>>,
    ) -> Result<(), Bme280Error<E>> {
        let mut pt_calib_data: [u8; BME280_P_T_CALIB_DATA_LEN] = [0; BME280_P_T_CALIB_DATA_LEN];

        interface.read_register(BME280_P_T_CALIB_DATA_ADDR, &mut pt_calib_data)?;

        let mut h_calib_data: [u8; BME280_H_CALIB_DATA_LEN] = [0; BME280_H_CALIB_DATA_LEN];
        interface.read_register(BME280_H_CALIB_DATA_ADDR, &mut h_calib_data)?;

        self.calibration = Some(parse_calib_data(&pt_calib_data, &h_calib_data));
        Ok(())
    }

    fn configure(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,

        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        match self.mode(interface)? {
            SensorMode::Sleep => {}
            _ => self.soft_reset(interface, delay)?,
        };

        interface.write_register(
            BME280_CTRL_HUM_ADDR,
            &[BME280_OVERSAMPLING_1X & BME280_CTRL_HUM_MSK],
        )?;
        let mut ctrl_meas: [u8; 1] = [0];
        interface.read_register(BME280_CTRL_MEAS_ADDR, &mut ctrl_meas)?;
        interface.write_register(BME280_CTRL_MEAS_ADDR, &ctrl_meas)?;

        let mut data: [u8; 1] = [0];
        interface.read_register(BME280_CTRL_MEAS_ADDR, &mut data)?;
        let data = [set_bits!(
            data[0],
            BME280_CTRL_PRESS_MSK,
            BME280_CTRL_PRESS_POS,
            BME280_OVERSAMPLING_16X
        )];
        let data = [set_bits!(
            data[0],
            BME280_CTRL_TEMP_MSK,
            BME280_CTRL_TEMP_POS,
            BME280_OVERSAMPLING_2X
        )];
        interface.write_register(BME280_CTRL_MEAS_ADDR, &data)?;

        let mut data: [u8; 1] = [0];
        interface.read_register(BME280_CONFIG_ADDR, &mut data)?;
        let data = [set_bits!(
            data[0],
            BME280_FILTER_MSK,
            BME280_FILTER_POS,
            BME280_FILTER_COEFF_16
        )];
        interface.write_register(BME280_CONFIG_ADDR, &data)?;
        Ok(())
    }

    fn mode(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
    ) -> Result<SensorMode, Bme280Error<E>> {
        let mut data: [u8; 1] = [0];
        interface.read_register(BME280_PWR_CTRL_ADDR, &mut data)?;
        match data[0] & BME280_SENSOR_MODE_MSK {
            BME280_SLEEP_MODE => Ok(SensorMode::Sleep),
            BME280_FORCED_MODE => Ok(SensorMode::Forced),
            BME280_NORMAL_MODE => Ok(SensorMode::Normal),
            _ => Err(Bme280Error::InvalidData),
        }
    }

    fn forced(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        self.set_mode(interface, delay, BME280_FORCED_MODE)
    }

    fn set_mode(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
        delay: &mut dyn DelayMs<u16>,

        mode: u8,
    ) -> Result<(), Bme280Error<E>> {
        match self.mode(interface)? {
            SensorMode::Sleep => {}
            _ => self.soft_reset(interface, delay)?,
        };
        let mut data: [u8; 1] = [0];
        interface.read_register(BME280_PWR_CTRL_ADDR, &mut data)?;
        let data = [set_bits!(data[0], BME280_SENSOR_MODE_MSK, 0, mode)];
        interface.write_register(BME280_PWR_CTRL_ADDR, &data)?;
        Ok(())
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    fn measure(
        &mut self,
        interface: &mut dyn Interface<Error = InterfaceError<E>>,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<Measurements<E>, Bme280Error<E>> {
        self.forced(interface, delay)?;
        delay.delay_ms(40); // await measurement
        let mut measurements: [u8; BME280_P_T_H_DATA_LEN] = [0; BME280_P_T_H_DATA_LEN];

        interface.read_register(BME280_DATA_ADDR, &mut measurements)?;
        match self.calibration.as_mut() {
            Some(calibration) => {
                let measurements = Measurements::parse(measurements, &mut *calibration)?;
                Ok(measurements)
            }
            None => Err(Bme280Error::NoCalibrationData),
        }
    }
}

fn parse_calib_data(
    pt_data: &[u8; BME280_P_T_CALIB_DATA_LEN],
    h_data: &[u8; BME280_H_CALIB_DATA_LEN],
) -> CalibrationData {
    let dig_t1 = concat_bytes!(pt_data[1], pt_data[0]);
    let dig_t2 = concat_bytes!(pt_data[3], pt_data[2]) as i16;
    let dig_t3 = concat_bytes!(pt_data[5], pt_data[4]) as i16;
    let dig_p1 = concat_bytes!(pt_data[7], pt_data[6]);
    let dig_p2 = concat_bytes!(pt_data[9], pt_data[8]) as i16;
    let dig_p3 = concat_bytes!(pt_data[11], pt_data[10]) as i16;
    let dig_p4 = concat_bytes!(pt_data[13], pt_data[12]) as i16;
    let dig_p5 = concat_bytes!(pt_data[15], pt_data[14]) as i16;
    let dig_p6 = concat_bytes!(pt_data[17], pt_data[16]) as i16;
    let dig_p7 = concat_bytes!(pt_data[19], pt_data[18]) as i16;
    let dig_p8 = concat_bytes!(pt_data[21], pt_data[20]) as i16;
    let dig_p9 = concat_bytes!(pt_data[23], pt_data[22]) as i16;
    let dig_h1 = pt_data[25];
    let dig_h2 = concat_bytes!(h_data[1], h_data[0]) as i16;
    let dig_h3 = h_data[2];
    let dig_h4 = (h_data[3] as i16 * 16) | ((h_data[4] as i16) & 0x0F);
    let dig_h5 = (h_data[5] as i16 * 16) | ((h_data[4] as i16) >> 4);
    let dig_h6 = h_data[6] as i8;

    CalibrationData {
        dig_t1,
        dig_t2,
        dig_t3,
        dig_p1,
        dig_p2,
        dig_p3,
        dig_p4,
        dig_p5,
        dig_p6,
        dig_p7,
        dig_p8,
        dig_p9,
        dig_h1,
        dig_h2,
        dig_h3,
        dig_h4,
        dig_h5,
        dig_h6,
        t_fine: 0,
    }
}
