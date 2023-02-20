//! I2C driver for the SPL06-007 pressure and temperature sensor. This sensor
//! is available as a breakout board for the Arduino platform. This driver
//! may also work with the SPL06-001, but this has not been tested.
//!
//! This sensor operates on the I2c address 0x76 and is connected to the
//! I2C bus on the Arduino Uno. Instantiate the Barometer struct with a
//! reference to the I2C bus and call the init() method to initialise the
//! sensor to default values. The sensor will then be ready to read
//! temperature and pressure values.
//!
//! Example usage:
//!
//! ```notest
//! use spl06_007::Barometer;
//!
//! let mut barometer = Barometer::new(&mut i2c).expect("Failed to instantiate barometer");
//!
//! barometer.init().expect("Failed to initialise barometer");
//!
//! let temp = barometer.get_temperature().unwrap();
//! let pressure = barometer.get_pressure().unwrap();
//! let altitude = barometer.altitude(1020.0).unwrap();
//! ```
//!
//! It is necessary to call `init` before any other methods are called. This method will set some default values for the sensor and is suitable for most use cases. Alternatively you can set the mode, sample rate, and oversampling values manually:
//!
//! ```notest
//! # use embedded_hal_mock::i2c::{Mock as I2c, Transaction as I2cTransaction};
//! # use spl06_007::Barometer;
//! # let mut i2c = I2c::new(&[]);
//!
//! # let mut barometer = Barometer::new(&mut i2c).expect("Failed to instantiate barometer");
//! barometer.set_pressure_config(SampleRate::Single, SampleRate::Eight);
//! barometer.set_temperature_config(SampleRate::Single, SampleRate::Eight);
//! barometer.set_mode(Mode::ContinuousPressureTemperature);
//! ```
//!
//! This is useful if you want to change the sample rate or oversampling values, such as for more rapid updates. It is also possible to set the mode to `Mode::Standby` to reduce power consumption. Other modes, including measuring only when polled, are not well supported at this time.
//!
//!

#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![warn(missing_debug_implementations)]

#![cfg_attr(not(test), no_std)]

extern crate embedded_hal as hal;
extern crate libm;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use libm::powf;

const ADDR: u8 = 0x76;
const PRS: u8 = 0x00;
const TMP: u8 = 0x03;
const PRS_CFG: u8 = 0x06;
const TMP_CFG: u8 = 0x07;
const MEAS_CFG: u8 = 0x08;
const CFG_REG: u8 = 0x09;
// const INT_STS: u8 = 0x0A;
// const FIFO_STS: u8 = 0x0B;
const RESET: u8 = 0x0C;
const ID: u8 = 0x0D;
const COEF: u8 = 0x10;

/// Use [Barometer::set_mode] to set the mode.
///
/// In continuous mode, the sensor will take measurements at the rate set by
/// [Barometer::set_pressure_config] and [Barometer::set_temperature_config]. Note that pressure
/// readings are dependent on temperature readings, so it is not recommended to use continuous
/// mode for pressure readings because they will be calculated using out-of-date temperature
/// readings.
///
/// Temperature and Pressure modes are intended to be used when the sensor is in standby mode.
/// They will take a new temperature or pressure reading, and then set the sensor to  standby
/// mode. Setting the mode to one of these modes is equivalent to calling
/// [Barometer::request_temperature_reading] or [Barometer::request_pressure_reading].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    /// The default mode. The sensor will not take any measurements. It is still possible to read
    /// the temperature and pressure, but the values will not be updated.
    Standby = 0b000,
    /// Take a single temperature reading and then return to standby mode.
    Pressure = 0b001,
    /// Take a single pressure reading and then return to standby mode.
    Temperature = 0b010,
    /// Take continuous pressure readings at the configured sample rate. Note that this mode is
    /// not recommended because pressure readings are dependent on temperature readings, so
    /// they will be calculated using out-of-date temperature readings.
    ContinuousPressure = 0b101,
    /// Take continuous temperature readings at the configured sample rate.
    ContinuousTemperature = 0b110,
    /// Take continuous pressure and temperature readings at the configured sample rate.
    ContinuousPressureTemperature = 0b111,
}

/// Setting for the sampling and oversampling rates.
///
/// Use [Barometer::set_pressure_config] and [Barometer::set_temperature_config] to set the
/// sampling and oversampling rates. The oversampling rate is the number of samples taken and
/// averaged to produce a single reading. The sampling rate is the number of times per second
/// that the sensor will record a new reading.
///
/// Note that oversampling rates above 8 are currently broken due to a bitshift bug.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum SampleRate {
    /// Take a single measurement per second or single oversample per measurement.
    One = 0b000,
    /// Two measurements per second or two oversamples per measurement. Labelled "Low Power" mode
    /// in the datasheet.
    Two = 0b001,
    /// Four measurements per second or four oversamples per measurement.
    Four = 0b010,
    /// Eight measurements per second or eight oversamples per measurement. This is the default
    /// value.
    Eight = 0b011,
    /// Sixteen measurements per second or sixteen oversamples per measurement. Labelled "Standard"
    ///  mode in the datasheet. Note that values above Eight are currently broken due to a bitshift
    /// bug.
    Sixteen = 0b100,
    /// Thirty-two measurements per second or thirty-two oversamples per measurement. Note that
    /// values above Eight are currently broken due to a bitshift bug.
    ThirtyTwo = 0b101,
    /// Sixty-four measurements per second or sixty-four oversamples per measurement. Labelled
    /// "High Precision" mode in the datasheet. Note that values above Eight are currently broken
    /// due to a bitshift bug.
    SixtyFour = 0b110,
    /// One hundred twenty-eight measurements per second or one hundred twenty-eight oversamples
    /// per measurement. Note that values above Eight are currently broken due to a bitshift bug.
    OneTwentyEight = 0b111,
} // Sample rates > 8 currently broken due to bitshift

// pub enum FifoStatus {
//     Empty = 0b01,
//     Partial = 0b00,
//     Full = 0b10,
// }

/// The SPL06-007 barometer.
///
/// This struct is generic over the I2C bus type. See method documentation for details.
#[derive(Debug)]
pub struct Barometer<'a, I2C>
where
    I2C: Read + Write + WriteRead,
{
    i2c: &'a mut I2C,
    calibration_data: CalibrationData,
}

impl<'a, I2C, E> Barometer<'a, I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Create a new instance of the barometer.
    ///
    /// This method will read the calibration data from the sensor and store it in the struct, so
    /// it is important that the I2C bus is initialised before calling this method. This method
    /// will also set the sensor to standby mode - use [Barometer::init] to initialise the sensor
    /// to default values.
    pub fn new(i2c: &'a mut I2C) -> Result<Self, E> {
        let mut barometer = Barometer {
            i2c,
            calibration_data: CalibrationData::default(),
        };
        barometer.set_mode(Mode::Standby)?;
        barometer.write(&[CFG_REG, 0x00])?; // disable FIFO
        while !barometer.calibration_data_is_available()? {}
        barometer.calibration_data = barometer.get_calibration_data()?;
        Ok(barometer)
    }

    /// Initialise the sensor to default values:
    /// - Pressure sample rate: 1
    /// - Pressure oversample rate: 8
    /// - Temperature oversample rate: 1
    /// - Temperature sample rate: 8
    /// - Mode: Continuous pressure and temperature
    /// - FIFO disabled
    /// - Interrupts disabled
    pub fn init(&mut self) -> Result<(), E> {
        self.set_pressure_config(SampleRate::One, SampleRate::Eight)?;
        self.set_temperature_config(SampleRate::One, SampleRate::Eight)?;
        self.set_mode(Mode::ContinuousPressureTemperature)?;
        self.write(&[CFG_REG, 0x00])?; // disable FIFO
        Ok(())
    }

    /// First four bits: Product ID
    /// Last four bits: Revision ID
    pub fn sensor_id(&mut self) -> Result<u8, E> {
        self.read8(ID)
    }

    /// Read and calculate the temperature in degrees celsius
    pub fn get_temperature(&mut self) -> Result<f32, E> {
        let cal = self.calibration_data;
        let temp = cal.c1 as f32 * self.traw_sc()?;
        let offset = cal.c0 as f32 / 2.0;
        Ok(temp + offset)
    }

    /// Read and calculate the pressure in millibars
    pub fn get_pressure(&mut self) -> Result<f32, E> {
        let cal = self.calibration_data;
        let traw_sc = self.traw_sc()?;
        let praw_sc = self.praw_sc()?;

        let pcomp = cal.c00 as f32
            + praw_sc * (cal.c10 as f32 + praw_sc * (cal.c20 as f32 + praw_sc * cal.c30 as f32))
            + traw_sc * cal.c01 as f32
            + traw_sc * praw_sc * (cal.c11 as f32 + praw_sc * cal.c21 as f32);
        Ok(pcomp / 100.0)
    }

    /// Read and calculate the altitude in metres
    pub fn altitude(&mut self, sea_level_hpa: f32) -> Result<f32, E> {
        Ok(44330.0 * (1.0 - powf(self.get_pressure()? / sea_level_hpa, 0.1903)))
    }

    fn raw_pressure(&mut self) -> Result<i32, E> {
        self.read24(PRS)
    }

    fn raw_temperature(&mut self) -> Result<i32, E> {
        self.read24(TMP)
    }

    fn traw_sc(&mut self) -> Result<f32, E> {
        let mut temp = self.raw_temperature()?;
        let oversample_rate = self.temperature_oversample_rate()?;
        if oversample_rate > SampleRate::Eight {
            temp <<= 1;
        }
        Ok(temp as f32 / oversample_rate.scale_factor())
    }

    fn praw_sc(&mut self) -> Result<f32, E> {
        let mut pressure = self.raw_pressure()?;
        let oversample_rate = self.pressure_oversample_rate()?;
        if oversample_rate > SampleRate::Eight {
            pressure <<= 1;
        }
        Ok(pressure as f32 / oversample_rate.scale_factor())
    }

    fn pressure_oversample_rate(&mut self) -> Result<SampleRate, E> {
        let byte = self.read8(PRS_CFG)?;
        Ok(SampleRate::from_u8(byte))
    }

    fn temperature_oversample_rate(&mut self) -> Result<SampleRate, E> {
        let byte = self.read8(TMP_CFG)?;
        Ok(SampleRate::from_u8(byte))
    }

    fn calibration_data_is_available(&mut self) -> Result<bool, E> {
        Ok((self.read8(MEAS_CFG)? >> 7) == 1)
    }

    /// Sensor data might not be available after the sensor is powered on or settings changed.
    /// Note that you should use new_data_is_available() for checking if new data is available.
    pub fn sensor_data_is_ready(&mut self) -> Result<bool, E> {
        Ok(((self.read8(MEAS_CFG)? >> 6) & 0b1) == 1)
    }

    /// Returns a tuple of (temperature, pressure), true if new data is available
    pub fn new_data_is_available(&mut self) -> Result<(bool, bool), E> {
        let byte = self.read8(MEAS_CFG)?;
        Ok((((byte >> 5) & 1) == 1, ((byte >> 4) & 1) == 1))
    }

    // pub fn fifo_is_enabled(&mut self) -> Result<bool, E> {
    //     let byte = self.read8(CFG_REG)? >> 1;
    //     Ok((byte & 1) == 1)
    // }

    // pub fn fifo_status(&mut self) -> Result<FifoStatus, E> {
    //     match self.read8(FIFO_STS)? & 0b11 {
    //         0b01 => Ok(FifoStatus::Empty),
    //         0b00 => Ok(FifoStatus::Partial),
    //         0b10 => Ok(FifoStatus::Full),
    //         _ => unreachable!("Not a valid FIFO status"),
    //     }
    // }

    // pub fn set_fifo(&mut self, value: bool) -> Result<(), E> {
    //     let mut reg = self.read8(CFG_REG)? & 0b11111101;
    //     reg |= (value as u8) << 1;
    //     self.write(&[CFG_REG, reg])
    // }

    // pub fn flush_fifo(&mut self) -> Result<(), E> {
    //     self.write(&[RESET, 0x80])
    // }

    // fn set_pressure_shift(&mut self, value: bool) -> Result<(), E> {
    //     let mut reg = self.read8(0x0E)? & 0b11111011;
    //     reg |= (value as u8) << 2;
    //     self.write(&[0x0E, reg])
    // }

    // fn set_temp_shift(&mut self, value: bool) -> Result<(), E> {
    //     let mut reg = self.read8(0x0E)? & 0b11110111;
    //     reg |= (value as u8) << 3;
    //     self.write(&[0x0E, reg])
    // }

    /// Reset the sensor. This will reset all configuration registers to their default values.
    /// You will need to reinitialse the sensor after this.
    pub fn soft_reset(&mut self) -> Result<(), E> {
        self.write(&[RESET, 0x09])
    }

    /// The sample rate is the number of measurements available per second.
    /// The oversample rate is the number of individual measurements used to calculate the final
    /// value for each final measurement. Higher oversample rates will give more accurate results.
    ///
    /// Note that the pressure readings are dependent on temperature readings, so the temperature
    /// oversample rate should be equal or higher than the pressure oversample rate.
    ///
    /// Oversampling rates higher than 8 are currently broken due to bitshift error
    pub fn set_pressure_config(
        &mut self,
        sample: SampleRate,
        mut oversample: SampleRate,
    ) -> Result<(), E> {
        if oversample > SampleRate::Eight {
            oversample = SampleRate::Eight;
        } // FIXME: Oversampling rates higher than 8 are broken due to bitshift error
        let byte = (sample as u8) << 4 | oversample as u8;
        // self.set_pressure_shift(oversample > SampleRate::Eight)?;
        self.write(&[PRS_CFG, byte])
    }

    /// The sample rate is the number of measurements available per second.
    /// The oversample rate is the number of individual measurements used to calculate the final
    /// value for each final measurement. Higher oversample rates will give more accurate results.
    ///
    /// Oversampling rates higher than 8 are currently broken due to bitshift error
    pub fn set_temperature_config(
        &mut self,
        sample: SampleRate,
        mut oversample: SampleRate,
    ) -> Result<(), E> {
        if oversample > SampleRate::Eight {
            oversample = SampleRate::Eight;
        } // FIXME: Oversampling rates higher than 8 are broken due to bitshift error
        let byte = 0x80 | (sample as u8) << 4 | oversample as u8;
        // self.set_temp_shift(oversample > SampleRate::Eight)?;
        self.write(&[TMP_CFG, byte])
    }

    /// Set the mode of the sensor. See the [Mode] enum for more details.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), E> {
        self.write(&[MEAS_CFG, mode as u8])
    }

    /// Request a temperature reading. This function is intended to be used when the sensor is in
    /// standby mode. It will take a new temperature reading, and then return to standby mode. If
    /// the sensor is in continuous mode, this function leaves the sensor in standby mode.
    ///
    /// This will not block until the reading is complete. You can check if the reading is
    /// complete using `new_data_is_available()`. You can then read the temperature using
    /// `get_temperature()`.
    pub fn request_temperature_reading(&mut self) -> Result<(), E> {
        self.set_mode(Mode::Temperature)
    }

    /// Request a pressure reading. This function is intended to be used when the sensor is in
    /// standby mode. It will take a new temperature reading, and then return to standby mode. If
    /// the sensor is in continuous mode, this function leaves the sensor in standby mode.
    ///
    /// This will not block until the reading is complete. You can check if the reading is
    /// complete using `new_data_is_available()`. You can then read the temperature using
    /// `get_pressure()`.
    ///
    /// Because the pressure reading is dependent on the temperature reading, it is recommended
    /// that you request a temperature reading first, and then a pressure reading. This will
    /// ensure that the temperature reading is recent. If you have recently requested a temperature
    /// reading and do not expect the temperature to have changed significantly, you can skip the
    /// temperature reading.
    pub fn request_pressure_reading(&mut self) -> Result<(), E> {
        self.set_mode(Mode::Pressure)
    }

    /// Request a temperature reading. This function is intended to be used when the sensor is in
    /// standby mode. It will take a new temperature reading, and then return to standby mode. If
    /// the sensor is in continuous mode, this function leaves the sensor in standby mode.
    ///
    /// This will block until the reading is complete, and then return the result.
    pub fn get_temperature_blocking(&mut self) -> Result<f32, E> {
        self.request_temperature_reading()?;
        while !self.new_data_is_available()?.0 {
            // wait
        }
        self.get_temperature()
    }

    /// Request a pressure reading. This function is intended to be used when the sensor is in
    /// standby mode. It will take a new temperature reading, and then return to standby mode. If
    /// the sensor is in continuous mode, this function leaves the sensor in standby mode.
    ///
    /// Unlike request_pressure_reading(), this function will also request a temperature reading
    /// first, so there is no need to do this manually.
    ///
    /// This function will block until the reading is complete, and then return the result.
    pub fn get_pressure_blocking(&mut self) -> Result<f32, E> {
        self.get_temperature_blocking()?;
        self.request_pressure_reading()?;
        while !self.new_data_is_available()?.1 {
            // wait
        }
        self.get_pressure()
    }

    fn get_calibration_data(&mut self) -> Result<CalibrationData, E> {
        let mut data = [0; 2];

        self.read(COEF, &mut data)?;
        let mut c0 = ((data[0] as u16) << 4) | data[1] as u16 >> 4;
        if c0 & (1 << 11) != 0 {
            c0 |= 0xF000;
        }

        // c1
        self.read(COEF + 1, &mut data)?;
        let mut c1 = (((data[0] & 0xF) as u16) << 8) | data[1] as u16;
        if c1 & (1 << 11) != 0 {
            c1 |= 0xF000;
        }

        // c00
        let mut data = [0; 3];
        self.read(COEF + 3, &mut data)?;
        let c00 = if data[0] & 0x80 != 0 { 0xFFF00000 } else { 0 }
            | ((data[0] as u32) << 12)
            | ((data[1] as u32) << 4)
            | ((data[2] as u32) & 0xF0) >> 4;

        // c10
        self.read(COEF + 5, &mut data)?;
        let c10 = if data[0] & 0x8 != 0 { 0xFFF00000 } else { 0 }
            | ((data[0] as u32) & 0x0F) << 16
            | (data[1] as u32) << 8
            | data[2] as u32;

        Ok(CalibrationData {
            c0: c0 as i16,
            c1: c1 as i16,
            c00: c00 as i32,
            c10: c10 as i32,
            c01: self.read16(COEF + 8)? as i32,
            c11: self.read16(COEF + 10)? as i32,
            c20: self.read16(COEF + 12)? as i32,
            c21: self.read16(COEF + 14)? as i32,
            c30: self.read16(COEF + 16)? as i32,
        })
    }

    fn write(&mut self, data: &[u8]) -> Result<(), E> {
        self.i2c.write(ADDR, data)
    }

    fn read(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(ADDR, &[reg], buffer)
    }

    fn read8(&mut self, reg: u8) -> Result<u8, E> {
        let mut buffer = [0u8];
        match self.read(reg, &mut buffer) {
            Ok(_) => Ok(buffer[0]),
            Err(res) => Err(res),
        }
    }

    fn read16(&mut self, reg: u8) -> Result<i16, E> {
        let mut buffer = [0u8; 2];
        match self.read(reg, &mut buffer) {
            Ok(_) => Ok((((buffer[0] as u16) << 8) | buffer[1] as u16) as i16),
            Err(res) => Err(res),
        }
    }

    fn read24(&mut self, reg: u8) -> Result<i32, E> {
        let mut buffer = [0; 3];
        self.read(reg, &mut buffer)?;
        let [msb, lsb, xlsb] = buffer.map(|x| x as u32);
        let res: u32 =
            if msb & 0x80 != 0 { 0xFF << 24 } else { 0x00 } | (msb << 16) | (lsb << 8) | xlsb;
        Ok(res as i32)
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct CalibrationData {
    c0: i16,
    c1: i16,
    c00: i32,
    c10: i32,
    c01: i32,
    c11: i32,
    c20: i32,
    c21: i32,
    c30: i32,
}

impl SampleRate {
    fn scale_factor(&self) -> f32 {
        match self {
            Self::One => 524288.0,
            Self::Two => 1572864.0,
            Self::Four => 3670016.0,
            Self::Eight => 7864320.0,
            Self::Sixteen => 253952.0,
            Self::ThirtyTwo => 516096.0,
            Self::SixtyFour => 1040384.0,
            Self::OneTwentyEight => 2088960.0,
        }
    }

    fn from_u8(value: u8) -> SampleRate {
        match value & 0b111 {
            0b000 => Self::One,
            0b001 => Self::Two,
            0b010 => Self::Four,
            0b011 => Self::Eight,
            0b100 => Self::Sixteen,
            0b101 => Self::ThirtyTwo,
            0b110 => Self::SixtyFour,
            0b111 => Self::OneTwentyEight,
            _ => unreachable!(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
    fn expectations(to_add: Vec<I2cTransaction>) -> Vec<I2cTransaction> {
        [
            // Barometer::new
            I2cTransaction::write(ADDR, vec![MEAS_CFG, 0x00]),
            I2cTransaction::write(ADDR, vec![CFG_REG, 0x00]),
            I2cTransaction::write_read(ADDR, vec![MEAS_CFG], vec![0x80]),
            I2cTransaction::write_read(ADDR, vec![COEF], vec![0xc, 0xbe]),
            I2cTransaction::write_read(ADDR, vec![COEF + 1], vec![0xbe, 0xfc]),
            I2cTransaction::write_read(ADDR, vec![COEF + 3], vec![0x13, 0xd9, 0xaf]),
            I2cTransaction::write_read(ADDR, vec![COEF + 5], vec![0xaf, 0x2b, 0x34]),
            I2cTransaction::write_read(ADDR, vec![COEF + 8], vec![0xf3, 0xf7]),
            I2cTransaction::write_read(ADDR, vec![COEF + 10], vec![0x4, 0xff]),
            I2cTransaction::write_read(ADDR, vec![COEF + 12], vec![0xda, 0x5a]),
            I2cTransaction::write_read(ADDR, vec![COEF + 14], vec![0x0, 0xa]),
            I2cTransaction::write_read(ADDR, vec![COEF + 16], vec![0xfb, 0x1b]),
        ]
        .to_vec()
        .into_iter()
        .chain(to_add)
        .collect::<std::vec::Vec<_>>()
    }

    #[test]
    fn test_barometer_new_init() {
        let expectations = expectations(vec![
            I2cTransaction::write(ADDR, vec![PRS_CFG, SampleRate::Eight as u8]),
            I2cTransaction::write(ADDR, vec![TMP_CFG, 0x80 | SampleRate::Eight as u8]),
            I2cTransaction::write(
                ADDR,
                vec![MEAS_CFG, Mode::ContinuousPressureTemperature as u8],
            ),
            I2cTransaction::write(ADDR, vec![CFG_REG, 0x00]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer::new(&mut i2c).unwrap();
        barometer.init().unwrap();
    }

    #[test]
    fn test_barometer_read() {
        let expectations = expectations(vec![
            I2cTransaction::write_read(ADDR, vec![0], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![0], vec![0]),
            I2cTransaction::write_read(ADDR, vec![0], vec![0, 1]),
            I2cTransaction::write_read(ADDR, vec![0], vec![0, 1, 2]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer::new(&mut i2c).unwrap();

        let mut buffer = [0; 3];
        barometer.read(0, &mut buffer).unwrap();
        barometer.read8(0).unwrap();
        barometer.read16(0).unwrap();
        barometer.read24(0).unwrap();
        assert_eq!(buffer, [0, 1, 2]);
    }

    #[test]
    fn test_barometer_get_calibration_data() {
        let expectations = expectations(vec![]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer {
            i2c: &mut i2c,
            calibration_data: CalibrationData::default(),
        };
        // remainder of new
        barometer.set_mode(Mode::Standby).unwrap();
        barometer.write(&[CFG_REG, 0x00]).unwrap(); // disable FIFO
        barometer.calibration_data_is_available().unwrap();
        let cal_data = barometer.get_calibration_data().unwrap();
        assert_eq!(cal_data.c0, 203, "c0");
        assert_eq!(cal_data.c1, -260, "c1");
        assert_eq!(cal_data.c00, 81306, "c00");
        assert_eq!(cal_data.c10, -54476, "c10");
        assert_eq!(cal_data.c01, -3081, "c01");
        assert_eq!(cal_data.c11, 1279, "c11");
        assert_eq!(cal_data.c20, -9638, "c20");
        assert_eq!(cal_data.c21, 10, "c21");
        assert_eq!(cal_data.c30, -1253, "c30");
    }

    #[test]
    fn test_barometer_read_temperature() {
        let expectations = expectations(vec![
            I2cTransaction::write_read(ADDR, vec![TMP], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![TMP_CFG], vec![0]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer::new(&mut i2c).unwrap();
        barometer.get_temperature().unwrap();
    }

    #[test]
    fn test_barometer_read_pressure() {
        let expectations = expectations(vec![
            I2cTransaction::write_read(ADDR, vec![TMP], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![TMP_CFG], vec![0]),
            I2cTransaction::write_read(ADDR, vec![PRS], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![PRS_CFG], vec![0]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer::new(&mut i2c).unwrap();
        barometer.get_pressure().unwrap();
    }

    #[test]
    fn test_barometer_read_altitude() {
        let expectations = expectations(vec![
            I2cTransaction::write_read(ADDR, vec![TMP], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![TMP_CFG], vec![0]),
            I2cTransaction::write_read(ADDR, vec![PRS], vec![0, 1, 2]),
            I2cTransaction::write_read(ADDR, vec![PRS_CFG], vec![0]),
        ]);
        let mut i2c = I2cMock::new(&expectations);
        let mut barometer = Barometer::new(&mut i2c).unwrap();
        let altitude = barometer.altitude(1000.0).unwrap();
        assert_eq!(altitude, 1714.7274);
    }
}
