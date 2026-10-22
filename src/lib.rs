#![no_std]
// src/lib.rs

mod config;
mod error;
mod interface;
mod registers;

use crate::config::{CalibrationData, compensate_gas_resistance, encode_gas_wait, encode_res_heat};
use crate::error::Error;
use crate::interface::I2cInterface;
use crate::registers::{Bme690 as Device, Mode};
use embedded_hal::{delay::DelayNs, i2c::I2c};

pub use crate::config::Config;

/// Expected value of the `chip_id` register.
const CHIP_ID: u8 = 0x61;
/// Ambient temperature (degrees Celsius) assumed for the heater calculation
/// before a first temperature reading is available.
const DEFAULT_AMBIENT_TEMP_C: i8 = 25;
pub struct Bme690<I2C, D> {
    device: Device<I2cInterface<I2C>>,
    delay: D,
    calibration: CalibrationData,
    config: Config,
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
        Self::new_with_config(i2c, address, delay, Config::default())
    }

    /// Creates a new driver with the provided measurement configuration.
    pub fn new_with_config(i2c: I2C, address: u8, delay: D, config: Config) -> Bme690<I2C, D> {
        Bme690 {
            device: Device::new(I2cInterface::new(i2c, address)),
            delay,
            calibration: CalibrationData::default(),
            config,
        }
    }

    /// Initializes the BME690 sensor to use Forced mode with the default settings.
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
            .write(|reg| reg.set_osrs_h(self.config.oversampling_humidity))?;
        self.device
            .config()
            .write(|reg| reg.set_filter(self.config.filter_coefficient))?;
        self.device.ctrl_meas().write(|reg| {
            reg.set_osrs_t(self.config.oversampling_temperature);
            reg.set_osrs_p(self.config.oversampling_pressure);
            reg.set_mode(self.config.mode);
        })?;
        self.device
            .ctrl_gas_0()
            .write(|reg| reg.set_heat_off(!self.config.gas_heater_enabled))?;
        self.device.ctrl_gas_1().write(|reg| {
            reg.set_nb_conv(self.config.gas_heater_profile.min(9));
            reg.set_run_gas(self.config.gas_heater_enabled);
        })?;

        self.calibration = CalibrationData::read(&mut self.device)?;

        let res_heat = encode_res_heat(
            self.config.gas_heater_temperature,
            DEFAULT_AMBIENT_TEMP_C,
            &self.calibration,
        );
        let gas_wait = encode_gas_wait(self.config.gas_heater_duration);
        self.device
            .res_heat()
            .write_at(self.config.gas_heater_profile.min(9).into(), |reg| {
                reg.set_value(res_heat)
            })?;
        self.device
            .gas_wait()
            .write_at(self.config.gas_heater_profile.min(9).into(), |reg| {
                reg.set_value(gas_wait)
            })?;

        Ok(())
    }

    /// Reads sensor data from the BME690.
    ///
    /// # Returns
    ///
    /// * `Result<SensorData, Error>` - Ok(SensorData) containing the sensor data, or an Error if it failed.
    pub fn read_sensor_data(&mut self) -> Result<SensorData, Error<I2C::Error>> {
        self.device.ctrl_gas_1().write(|reg| {
            reg.set_nb_conv(self.config.gas_heater_profile.min(9));
            reg.set_run_gas(self.config.gas_heater_enabled);
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
