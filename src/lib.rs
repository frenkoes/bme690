#![no_std]
// src/lib.rs

mod config;
mod error;
mod interface;
mod registers;

use crate::config::{CalibrationData, compensate_gas_resistance, encode_gas_wait, encode_res_heat};
use crate::error::Error;
use crate::interface::I2cInterface;
use crate::registers::{Bme690 as Device, Filter, Mode, Oversampling};
use embedded_hal::{delay::DelayNs, i2c::I2c};

/// Expected value of the `chip_id` register.
const CHIP_ID: u8 = 0x61;
/// Ambient temperature (degrees Celsius) assumed for the heater calculation
/// before a first temperature reading is available.
const DEFAULT_AMBIENT_TEMP_C: i8 = 25;
/// Default heater target temperature in degrees Celsius.
const DEFAULT_HEATER_TEMP_C: u16 = 300;
/// Default heater-on duration in milliseconds.
const DEFAULT_HEATER_DURATION_MS: u16 = 100;

pub struct Bme690<I2C, D> {
    device: Device<I2cInterface<I2C>>,
    delay: D,
    calibration: CalibrationData,
}

impl<I2C, D> Bme690<I2C, D>
where
    D: DelayNs,
    I2C: I2c,
{
    /// Creates a new instance of the Bme690 driver.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C peripheral to use for communication.
    /// * `delay` - The delay implementation to use for timing operations.
    /// * `address` - The I2C address of the BME690 sensor.
    ///
    /// # Returns
    ///
    /// * `Self` - A new instance of the Bme690 driver.
    pub fn new(i2c: I2C, address: u8, delay: D) -> Bme690<I2C, D> {
        Bme690 {
            device: Device::new(I2cInterface::new(i2c, address)),
            delay,
            calibration: CalibrationData::default(),
        }
    }

    /// Initializes the BME690 sensor.
    ///
    /// # Returns
    ///
    /// * `Result<(), Error>` - Ok(()) if the initialization was successful, or an Error if it failed.
    pub fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.device.soft_reset().write(|reg| reg.set_cmd(0xB6))?;
        self.delay.delay_ms(10);

        let chip_id = self.device.chip_id().read()?.value();
        if chip_id != CHIP_ID {
            return Err(Error::InvalidChipId);
        }

        self.device
            .ctrl_hum()
            .write(|reg| reg.set_osrs_h(Oversampling::X1))?;
        self.device
            .config()
            .write(|reg| reg.set_filter(Filter::Off))?;
        self.device.ctrl_meas().write(|reg| {
            reg.set_osrs_t(Oversampling::X2);
            reg.set_osrs_p(Oversampling::X4);
            reg.set_mode(Mode::Sleep);
        })?;

        self.calibration = CalibrationData::read(&mut self.device)?;

        let res_heat = encode_res_heat(
            DEFAULT_HEATER_TEMP_C,
            DEFAULT_AMBIENT_TEMP_C,
            &self.calibration,
        );
        let gas_wait = encode_gas_wait(DEFAULT_HEATER_DURATION_MS);
        self.device
            .res_heat()
            .write_at(0, |reg| reg.set_value(res_heat))?;
        self.device
            .gas_wait()
            .write_at(0, |reg| reg.set_value(gas_wait))?;

        Ok(())
    }

    /// Reads sensor data from the BME690.
    ///
    /// # Returns
    ///
    /// * `Result<SensorData, Error>` - Ok(SensorData) containing the sensor data, or an Error if it failed.
    pub fn read_sensor_data(&mut self) -> Result<SensorData, Error<I2C::Error>> {
        self.device.ctrl_gas_1().write(|reg| {
            reg.set_nb_conv(0);
            reg.set_run_gas(true);
        })?;
        self.device
            .ctrl_meas()
            .modify(|reg| reg.set_mode(Mode::Forced))?;

        // Wait for the temperature/pressure/humidity oversampling and the gas
        // heater conversion to complete, then poll for new data.
        self.delay.delay_ms(150);
        let mut status = self.device.eas_status_0().read()?;
        let mut tries = 5;
        while !status.new_data() && tries > 0 {
            self.delay.delay_ms(10);
            status = self.device.eas_status_0().read()?;
            tries -= 1;
        }
        if !status.new_data() {
            return Err(Error::ReadingTimeout);
        }

        let adc_pres = self.device.pressure().read()?.adc_pres();
        let adc_temp = self.device.temperature().read()?.adc_temp();
        let adc_hum = self.device.humidity().read()?.adc_hum();
        let gas = self.device.gas_resistance().read()?;

        let (temperature, t_lin) = self.calibration.compensate_temperature(adc_temp);
        let pressure = self.calibration.compensate_pressure(adc_pres, t_lin);
        let humidity = self.calibration.compensate_humidity(adc_hum, temperature);
        let gas_resistance = compensate_gas_resistance(gas.adc_gas_res(), gas.gas_range());

        let status_byte = ((status.new_data() as u8) << 7)
            | ((gas.gas_valid() as u8) << 5)
            | ((gas.heat_stab() as u8) << 4);

        Ok(SensorData {
            status: status_byte,
            gas_index: status.gas_meas_index(),
            meas_index: self.device.meas_index().read()?.value(),
            temperature,
            pressure,
            humidity,
            gas_resistance,
        })
    }
}

#[derive(Debug)]
pub struct SensorData {
    /// Contains new_data, gasm_valid & heat_stab
    pub status: u8,
    /// Index of heater profile used
    pub gas_index: u8,
    /// Measurement index
    pub meas_index: u8,
    /// Temperature in units of 0.01 degrees Celsius.
    pub temperature: i16,
    /// Pressure in Pascal.
    pub pressure: u32,
    /// Relative humidity in units of 0.001 %RH.
    pub humidity: u32,
    /// Gas resistance in milliohm.
    pub gas_resistance: u32,
}
