//! BME280 driver for sensors attached via SPI.

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;

use bit_byte_structs::bus::{InterfaceError, SPIPeripheral};
use core::marker::PhantomData;

use super::{BME280Common, Bme280Error, Interface, Measurements};
use core::fmt::{self, Debug};

/// Representation of a BME280
pub struct BME280<SPI, CS, E> {
    phantom: PhantomData<SPI>,
    phantom_cs: PhantomData<CS>,
    common: BME280Common<dyn Interface<Error = InterfaceError<E>>, E>,
}

impl<SPI, CS, E> Debug for BME280<SPI, CS, E>
//where
//    E: Debug
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        f.write_str("BME280 on SPI")?;

        Ok(())
    }
}

impl<SPI, CS, E> BME280<SPI, CS, E>
where
    SPI: Transfer<u8, Error = E>,
    CS: OutputPin,
{
    /// Create a new BME280 struct
    pub fn new() -> Result<Self, Bme280Error<E>> {
        // Deassert chip-select.

        Ok(BME280 {
            phantom: PhantomData,
            phantom_cs: PhantomData,
            common: BME280Common::new()?,
        })
    }

    /// Initializes the BME280
    pub fn init(
        &mut self,
        spi_bus: &mut SPI,
        cs: &mut CS,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<(), Bme280Error<E>> {
        let mut interface = SPIPeripheral::<SPI, CS, E>::new(spi_bus, cs);
        self.common.init(&mut interface, delay)
    }

    /// Captures and processes sensor data for temperature, pressure, and humidity
    pub fn measure(
        &mut self,
        spi_bus: &mut SPI,
        cs: &mut CS,
        delay: &mut dyn DelayMs<u16>,
    ) -> Result<Measurements<E>, Bme280Error<E>> {
        let mut interface = SPIPeripheral::<SPI, CS, E>::new(spi_bus, cs);
        self.common.measure(&mut interface, delay)
    }
}
