// src/config.rs
// Calibration data storage and the integer compensation formulas for
// temperature, pressure, humidity and gas resistance.
//
// Ported from the fixed-point (`BME69X_USE_FPU` undefined) code path of
// Bosch's official `BME690_SensorAPI` reference driver (`bme69x.c`,
// BSD-3-Clause), https://github.com/boschsensortec/BME690_SensorAPI.

use crate::registers::{Bme690 as Device, Filter, Mode, Oversampling};
use device_driver::RegisterInterface;

#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub mode: Mode,
    pub oversampling_temperature: Oversampling,
    pub oversampling_pressure: Oversampling,
    pub oversampling_humidity: Oversampling,
    pub filter_coefficient: Filter,
    pub gas_heater_temperature: u16,
    pub gas_heater_duration: u16,
    pub gas_heater_enabled: bool,
}

/// Default configuration values for the BME690 sensor. Using forced mode.  
impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Sleep,
            oversampling_temperature: Oversampling::X2,
            oversampling_pressure: Oversampling::X16,
            oversampling_humidity: Oversampling::X1,
            filter_coefficient: Filter::Off,
            gas_heater_temperature: 320, // °C
            gas_heater_duration: 150,    // ms
            gas_heater_enabled: true,
        }
    }
}
/// Calibration coefficients read once from the sensor at [`init`](crate::Bme690::init) time.
#[derive(Debug, Default, Clone, Copy)]
pub struct CalibrationData {
    par_t1: u16,
    par_t2: u16,
    par_t3: i8,
    par_p1: u16,
    par_p2: u16,
    par_p3: i8,
    par_p4: i8,
    par_p5: i16,
    par_p6: i16,
    par_p7: i8,
    par_p8: i8,
    par_p9: i16,
    par_p10: i8,
    par_p11: i8,
    par_h1: i16,
    par_h2: i8,
    par_h3: u8,
    par_h4: i8,
    par_h5: i16,
    par_h6: u8,
    par_g1: i8,
    par_g2: i16,
    par_g3: i8,
    pub(crate) res_heat_range: u8,
    pub(crate) res_heat_val: i8,
}

impl CalibrationData {
    /// Reads all calibration coefficients from the device.
    pub(crate) fn read<I>(device: &mut Device<I>) -> Result<Self, I::Error>
    where
        I: RegisterInterface<AddressType = u8>,
    {
        let par_h1_h5 = device.par_h_1_par_h_5().read()?;

        Ok(Self {
            par_t1: device.par_t_1().read()?.value(),
            par_t2: device.par_t_2().read()?.value(),
            par_t3: device.par_t_3().read()?.value(),
            par_p1: device.par_p_1().read()?.value(),
            par_p2: device.par_p_2().read()?.value(),
            par_p3: device.par_p_3().read()?.value(),
            par_p4: device.par_p_4().read()?.value(),
            par_p5: device.par_p_5().read()?.value(),
            par_p6: device.par_p_6().read()?.value(),
            par_p7: device.par_p_7().read()?.value(),
            par_p8: device.par_p_8().read()?.value(),
            par_p9: device.par_p_9().read()?.value(),
            par_p10: device.par_p_10().read()?.value(),
            par_p11: device.par_p_11().read()?.value(),
            par_h1: par_h1_h5.par_h_1(),
            par_h5: par_h1_h5.par_h_5(),
            par_h2: device.par_h_2().read()?.value(),
            par_h3: device.par_h_3().read()?.value(),
            par_h4: device.par_h_4().read()?.value(),
            par_h6: device.par_h_6().read()?.value(),
            par_g1: device.par_g_1().read()?.value(),
            par_g2: device.par_g_2().read()?.value(),
            par_g3: device.par_g_3().read()?.value(),
            res_heat_range: device.res_heat_range().read()?.value(),
            res_heat_val: device.res_heat_val().read()?.value(),
        })
    }

    /// Compensates a raw temperature ADC reading.
    ///
    /// Returns the temperature in units of 0.01 degrees Celsius, along with
    /// the intermediate `t_lin` value needed by [`compensate_pressure`](Self::compensate_pressure).
    pub(crate) fn compensate_temperature(&self, temp_adc: u32) -> (i16, i64) {
        const TEMPADC_SCALE: i64 = 256;
        const TEMPCOMP_SCALE1: i64 = 262144;
        const TEMPCOMP_SCALE2: i64 = 4294967296;
        const TEMPCOMP_SCALE3: i64 = 25;
        const TEMPCOMP_SCALE4: i64 = 16384;

        let partial_data1 = temp_adc as i64 - TEMPADC_SCALE * self.par_t1 as i64;
        let partial_data2 = partial_data1 * self.par_t2 as i64;
        let partial_data3 = partial_data1 * partial_data1;
        let partial_data4 = partial_data3 * self.par_t3 as i64;
        let partial_data5 = partial_data2 * TEMPCOMP_SCALE1 + partial_data4;
        let t_lin = partial_data5 / TEMPCOMP_SCALE2;
        let temperature = (t_lin * TEMPCOMP_SCALE3) / TEMPCOMP_SCALE4;

        (temperature as i16, t_lin)
    }

    /// Compensates a raw pressure ADC reading. Returns the pressure in Pascal.
    pub(crate) fn compensate_pressure(&self, pres_adc: u32, t_lin: i64) -> u32 {
        let pres_adc = pres_adc as i64;

        let partial_data1 = t_lin * t_lin;
        let partial_data2 = partial_data1 / 64;
        let partial_data3 = partial_data2 * t_lin / 256;
        let partial_data4 = self.par_p4 as i64 * partial_data3 / 32;
        let partial_data5 = self.par_p3 as i64 * partial_data1 * 16;
        let partial_data6 = self.par_p2 as i64 * t_lin * (1 << 22);

        let offset =
            self.par_p1 as i64 * (1i64 << 47) + partial_data4 + partial_data5 + partial_data6;

        let partial_data2 = (self.par_p8 as i64 * partial_data3) / (1 << 5);
        let partial_data4 = self.par_p7 as i64 * partial_data1 * (1 << 2);
        let partial_data5 = (self.par_p6 as i64 - 16384) * t_lin * (1 << 21);

        let sensitivity = (self.par_p5 as i64 - 16384) * (1i64 << 46)
            + partial_data2
            + partial_data4
            + partial_data5;

        let partial_data1 = (sensitivity / (1 << 24)) * pres_adc;
        let partial_data2 = self.par_p10 as i64 * t_lin;
        let partial_data3 = partial_data2 + self.par_p9 as i64 * (1 << 16);
        let partial_data4 = partial_data3 * pres_adc / (1 << 13);
        let mut partial_data5 = (pres_adc * partial_data4 / 10) / (1 << 9);
        partial_data5 *= 10;
        let partial_data6 = pres_adc * pres_adc;

        let partial_data2 = self.par_p11 as i64 * partial_data6 / (1 << 16);
        let partial_data3 = partial_data2 * pres_adc / (1 << 7);
        let partial_data4 = offset / 4 + partial_data1 + partial_data5 + partial_data3;

        let press_comp = (partial_data4 / (1i64 << 40)) * 25;

        (press_comp / 100) as u32
    }

    /// Compensates a raw humidity ADC reading.
    ///
    /// Returns the relative humidity in units of 0.001 %RH. `comp_temperature`
    /// is the value returned by [`compensate_temperature`](Self::compensate_temperature).
    pub(crate) fn compensate_humidity(&self, hum_adc: u16, comp_temperature: i16) -> u32 {
        let hum_adc = hum_adc as i64;
        let t_comp = comp_temperature as i64;

        let temp_scaled = (t_comp * 256 - 128) / 5;
        let mut var_h = temp_scaled - 76800;

        var_h =
            (((hum_adc * 16384 - (self.par_h1 as i64 * 1048576) - (self.par_h2 as i64 * var_h))
                + 16384)
                / 32768)
                * ((((((var_h * self.par_h4 as i64) / 1024)
                    * ((var_h * self.par_h3 as i64) / 2048 + 32768))
                    / 1024)
                    + 2097152)
                    * self.par_h5 as i64
                    + 8192)
                / 16384;

        var_h -= (((var_h / 32768) * (var_h / 32768)) / 128) * self.par_h6 as i64 / 16;

        let var_h = var_h.clamp(0, 419_430_400);

        (var_h / 4096) as u32
    }
}

/// Compensates a raw gas resistance ADC reading. Returns the resistance in milliohm.
pub(crate) fn compensate_gas_resistance(gas_res_adc: u16, gas_range: u8) -> u32 {
    let var1 = 262_144u32 >> gas_range;
    let var2 = (gas_res_adc as i32 - 512) * 3 + 4096;

    let calc_gas_res = (10_000u32 * var1) / var2 as u32;
    calc_gas_res * 100
}

/// Encodes a heater-on duration (in milliseconds) into the register format
/// used by `GasWait`/`SharedHeaterDuration` (6-bit value + 2-bit multiplier).
pub(crate) fn encode_gas_wait(duration_ms: u16) -> u8 {
    if duration_ms >= 0xfc0 {
        return 0xff;
    }

    let mut duration = duration_ms;
    let mut factor = 0u8;
    while duration > 0x3F {
        duration /= 4;
        factor += 1;
    }

    duration as u8 + factor * 64
}

/// Computes the `res_heat` register value for a target heater temperature
/// (in degrees Celsius) at a given ambient temperature (in degrees Celsius).
pub(crate) fn encode_res_heat(
    target_temp_c: u16,
    ambient_temp_c: i8,
    calib: &CalibrationData,
) -> u8 {
    let target_temp_c = target_temp_c.min(400) as i64;

    // Widened to i64 to match the reference driver, which relies on a `ULL`
    // literal to force 64-bit arithmetic here and avoid i32 overflow.
    let var1 = ((ambient_temp_c as i64 * calib.par_g3 as i64) / 1000) * 256;
    let var2 = (calib.par_g1 as i64 + 784)
        * (((calib.par_g2 as i64 + 154_009) * target_temp_c * 5 / 100) + 3_276_800)
        / 10;
    let var3 = var1 + (var2 >> 1);
    let var4 = var3 / (calib.res_heat_range as i64 + 4);
    let var5 = (131 * calib.res_heat_val as i64) + 65536;
    let heatr_res_x100 = ((var4 / var5) - 250) * 34;

    ((heatr_res_x100 + 50) / 100) as u8
}
