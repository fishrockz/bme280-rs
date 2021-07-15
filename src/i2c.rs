//! BME280 driver for sensors attached via I2C.
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

use bit_byte_structs::bus::{I2CPeripheral, InterfaceError};
use core::marker::PhantomData;

use super::{BME280Common, Bme280Error, Interface, Measurements};
use core::fmt::{self, Debug};

const BME280_I2C_ADDR_PRIMARY: u8 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u8 = 0x77;

/// Representation of a BME280
//#[derive(Debug)]
pub struct BME280<I2C, E> {
    phantom: PhantomData<I2C>,
    address: u8,
    common: BME280Common<dyn Interface<Error = InterfaceError<E>>, E>,
}

impl<I2C, E> Debug for BME280<I2C, E>
//where
//    E: Debug
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        f.write_fmt(format_args!("BME280 at {:?}", self.address))?;

        Ok(())
    }
}

impl<I2C, E> BME280<I2C, E>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Create a new BME280 struct using the primary I²C address `0x76`
    pub fn new_primary() -> Result<Self, Bme280Error<E>> {
        Self::new(BME280_I2C_ADDR_PRIMARY)
    }

    /// Create a new BME280 struct using the secondary I²C address `0x77`
    pub fn new_secondary() -> Result<Self, Bme280Error<E>> {
        Self::new(BME280_I2C_ADDR_SECONDARY)
    }

    /// Create a new BME280 struct using a custom I²C address
    pub fn new(address: u8) -> Result<Self, Bme280Error<E>> {
        Ok(BME280 {
            phantom: PhantomData,
            common: BME280Common::new()?,
            address,
        })
    }

    /// Initializes the BME280
    pub fn init(
        &mut self,
        i2c_bus: &mut I2C,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        let mut interface = I2CPeripheral::<I2C, E>::new(i2c_bus, self.address);
        self.common.init(&mut interface, delay)
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure(
        &mut self,
        i2c_bus: &mut I2C,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<Measurements<E>, Bme280Error<E>> {
        let mut interface = I2CPeripheral::<I2C, E>::new(i2c_bus, self.address);
        self.common.measure(&mut interface, delay)
    }
}

/// Representation of a BME280
//#[derive(Debug)]
pub struct BME280Owned<I2C, D, E> {
    bus: I2C,
    delay: D,
    core: BME280<I2C, E>,
}

impl<I2C, D, E> Debug for BME280Owned<I2C, D, E>
//where
//    E: Debug
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        f.write_fmt(format_args!("BME280 at {:?}", self.core.address))?;

        Ok(())
    }
}

impl<I2C, D, E> BME280Owned<I2C, D, E>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u16>,
{
    /// Create a new BME280 struct using the primary I²C address `0x76`
    pub fn new_primary(bus: I2C, delay: D) -> Result<Self, Bme280Error<E>> {
        Self::new(bus, delay, BME280_I2C_ADDR_PRIMARY)
    }

    /// Create a new BME280 struct using the secondary I²C address `0x77`
    pub fn new_secondary(bus: I2C, delay: D) -> Result<Self, Bme280Error<E>> {
        Self::new(bus, delay, BME280_I2C_ADDR_SECONDARY)
    }

    /// Create a new BME280 struct using a custom I²C address
    pub fn new(bus: I2C, delay: D, address: u8) -> Result<Self, Bme280Error<E>> {
        Ok(BME280Owned {
            bus,
            delay,
            core: BME280::<I2C, E>::new(address)?,
        })
    }

    /// Initializes the BME280
    pub fn init(&mut self) -> Result<(), Bme280Error<E>> {
        self.core.init(&mut self.bus, &mut self.delay)
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure(&mut self) -> Result<Measurements<E>, Bme280Error<E>> {
        self.core.measure(&mut self.bus, &mut self.delay)
    }

    /// Release the bus and time so they can be re-used
    pub fn release(self) -> (I2C, D) {
        (self.bus, self.delay)
    }
}
