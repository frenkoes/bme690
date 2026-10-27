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
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

pub use crate::config::Config;

/// Compatibility name for the asynchronous driver.
pub type AsyncBme690<I2C, D> = Bme690<I2C, D>;

/// Expected value of the `chip_id` register.
const CHIP_ID: u8 = 0x61;
/// Ambient temperature (degrees Celsius) assumed for the heater calculation
/// before a first temperature reading is available.
const DEFAULT_AMBIENT_TEMP_C: i8 = 25;
/// Asynchronous BME690 driver.
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
    pub async fn init(&mut self) -> Result<(), Error<I2C::Error>> {
        self.device
            .soft_reset()
            .write_async(|reg| reg.set_cmd(0xB6))
            .await?;
        self.delay.delay_ms(10).await;

        let chip_id = self.device.chip_id().read_async().await?.value();
        if chip_id != CHIP_ID {
            return Err(Error::InvalidChipId);
        }

        self.device
            .ctrl_hum()
            .write_async(|reg| reg.set_osrs_h(self.config.oversampling_humidity))
            .await?;
        self.device
            .config()
            .write_async(|reg| reg.set_filter(self.config.filter_coefficient))
            .await?;
        self.device
            .ctrl_meas()
            .write_async(|reg| {
                reg.set_osrs_t(self.config.oversampling_temperature);
                reg.set_osrs_p(self.config.oversampling_pressure);
                reg.set_mode(self.config.mode);
            })
            .await?;
        self.device
            .ctrl_gas_0()
            .write_async(|reg| reg.set_heat_off(!self.config.gas_heater_enabled))
            .await?;
        self.device
            .ctrl_gas_1()
            .write_async(|reg| {
                reg.set_nb_conv(self.config.gas_heater_profile.min(9));
                reg.set_run_gas(self.config.gas_heater_enabled);
            })
            .await?;

        self.calibration = CalibrationData::read_async(&mut self.device).await?;

        if self.config.gas_heater_enabled {
            let res_heat = encode_res_heat(
                self.config.gas_heater_temperature,
                DEFAULT_AMBIENT_TEMP_C,
                &self.calibration,
            );
            let gas_wait = encode_gas_wait(self.config.gas_heater_duration);
            self.device
                .res_heat()
                .write_at_async(self.config.gas_heater_profile.min(9).into(), |reg| {
                    reg.set_value(res_heat)
                })
                .await?;
            self.device
                .gas_wait()
                .write_at_async(self.config.gas_heater_profile.min(9).into(), |reg| {
                    reg.set_value(gas_wait)
                })
                .await?;
        }

        Ok(())
    }

    /// Reads temperature in units of 0.01 degrees Celsius.
    pub async fn read_temperature(&mut self) -> Result<i16, Error<I2C::Error>> {
        self.start_measurement().await?;
        self.read_temperature_value().await
    }

    /// Reads gas resistance in ohms.
    pub async fn read_gas(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.start_measurement().await?;
        self.read_gas_value().await
    }

    /// Reads pressure in Pascal.
    pub async fn read_pressure(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.start_measurement().await?;
        self.read_pressure_value().await
    }

    /// Reads relative humidity in units of 0.001 %RH.
    pub async fn read_humidity(&mut self) -> Result<u32, Error<I2C::Error>> {
        self.start_measurement().await?;
        self.read_humidity_value().await
    }

    /// Reads status flags and measurement indices from the latest conversion.
    pub async fn read_status(&mut self) -> Result<SensorStatus, Error<I2C::Error>> {
        let (new_data, gas_index) = self.start_measurement().await?;
        self.read_status_value(new_data, gas_index).await
    }

    /// Reads all sensor data from one BME690 conversion.
    pub async fn read_sensor_data(&mut self) -> Result<SensorData, Error<I2C::Error>> {
        let (new_data, gas_index) = self.start_measurement().await?;

        let status = self.read_status_value(new_data, gas_index).await?;
        let temperature = self.read_temperature_value().await?;
        let pressure = self.read_pressure_value().await?;
        let humidity = self.read_humidity_value().await?;
        let gas_resistance = self.read_gas_value().await?;

        Ok(SensorData {
            status: status.status,
            gas_index: status.gas_index,
            meas_index: status.meas_index,
            temperature,
            pressure,
            humidity,
            gas_resistance,
        })
    }

    async fn start_measurement(&mut self) -> Result<(bool, u8), Error<I2C::Error>> {
        if self.config.gas_heater_enabled {
            self.device
                .ctrl_gas_0()
                .write_async(|reg| reg.set_heat_off(false))
                .await?;
            self.device
                .ctrl_gas_1()
                .write_async(|reg| {
                    reg.set_nb_conv(self.config.gas_heater_profile.min(9));
                    reg.set_run_gas(true);
                })
                .await?;
        } else {
            self.device
                .ctrl_gas_0()
                .write_async(|reg| reg.set_heat_off(true))
                .await?;
            self.device
                .ctrl_gas_1()
                .write_async(|reg| reg.set_run_gas(false))
                .await?;
        }
        self.device
            .ctrl_meas()
            .modify_async(|reg| reg.set_mode(Mode::Forced))
            .await?;

        // Wait for oversampling and, when enabled, the gas heater conversion.
        self.delay.delay_ms(150).await;
        let mut status = self.device.eas_status_0().read_async().await?;
        let mut tries = 5;
        while !status.new_data() && tries > 0 {
            self.delay.delay_ms(10).await;
            status = self.device.eas_status_0().read_async().await?;
            tries -= 1;
        }
        if !status.new_data() {
            return Err(Error::ReadingTimeout);
        }

        Ok((status.new_data(), status.gas_meas_index()))
    }

    async fn read_temperature_value(&mut self) -> Result<i16, Error<I2C::Error>> {
        let adc_temp = self.device.temperature().read_async().await?.adc_temp();
        let (temperature, _) = self.calibration.compensate_temperature(adc_temp);
        Ok(temperature)
    }

    async fn read_pressure_value(&mut self) -> Result<u32, Error<I2C::Error>> {
        let adc_pres = self.device.pressure().read_async().await?.adc_pres();
        let adc_temp = self.device.temperature().read_async().await?.adc_temp();
        let (_, t_lin) = self.calibration.compensate_temperature(adc_temp);
        Ok(self.calibration.compensate_pressure(adc_pres, t_lin))
    }

    async fn read_humidity_value(&mut self) -> Result<u32, Error<I2C::Error>> {
        let adc_hum = self.device.humidity().read_async().await?.adc_hum();
        let adc_temp = self.device.temperature().read_async().await?.adc_temp();
        let (temperature, _) = self.calibration.compensate_temperature(adc_temp);
        Ok(self.calibration.compensate_humidity(adc_hum, temperature))
    }

    async fn read_gas_value(&mut self) -> Result<u32, Error<I2C::Error>> {
        let gas = self.device.gas_resistance().read_async().await?;
        Ok(compensate_gas_resistance(
            gas.adc_gas_res(),
            gas.gas_range(),
        ))
    }

    async fn read_status_value(
        &mut self,
        new_data: bool,
        gas_index: u8,
    ) -> Result<SensorStatus, Error<I2C::Error>> {
        let gas = self.device.gas_resistance().read_async().await?;
        let status_byte = ((new_data as u8) << 7)
            | ((gas.gas_valid() as u8) << 5)
            | ((gas.heat_stab() as u8) << 4);

        Ok(SensorStatus {
            status: status_byte,
            gas_index,
            meas_index: self.device.meas_index().read_async().await?.value(),
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SensorStatus {
    /// Contains new_data, gas_valid and heat_stab flags.
    pub status: u8,
    /// Index of the heater profile used.
    pub gas_index: u8,
    /// Measurement index.
    pub meas_index: u8,
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
    /// Gas resistance in ohms.
    pub gas_resistance: u32,
}
