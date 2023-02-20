#![no_std]

extern crate embedded_hal as hal;
extern crate libm;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use libm::powf;

const ADDR: u8 = 0x76;

pub enum Mode {
    Standby = 0b000,
    Pressure = 0b001,
    Temperature = 0b010,
    ContinuousPressure = 0b101,
    ContinuousTemperature = 0b110,
    ContinuousPressureTemperature = 0b111,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum SampleRate {
    Single = 0b000,
    Two = 0b001,
    Four = 0b010,
    Eight = 0b011,
    // Sixteen = 0b100,
    // ThirtyTwo = 0b101,
    // SixtyFour = 0b110,
    // OneTwentyEight = 0b111,
} // Sample rates > 8 currently broken due to bitshift

// pub enum FifoStatus {
//     Empty = 0b01,
//     Partial = 0b00,
//     Full = 0b10,
// }

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
    pub fn new(i2c: &'a mut I2C) -> Result<Self, E> {
        let mut barometer = Barometer {
            i2c,
            calibration_data: CalibrationData::default(),
        };
        barometer.set_mode(Mode::Standby)?;
        barometer.write(&[0x09, 0x00])?; // disable FIFO
        while !barometer.calibration_data_is_available()? {}
        barometer.calibration_data = barometer.get_calibration_data()?;
        Ok(barometer)
    }

    pub fn initialise(&mut self) -> Result<(), E> {
        self.set_pressure_config(SampleRate::Single, SampleRate::Eight)?;
        self.set_temperature_config(SampleRate::Single, SampleRate::Eight)?;
        self.set_mode(Mode::ContinuousPressureTemperature)?;
        self.write(&[0x09, 0x00])?; // disable FIFO
        Ok(())
    }

    pub fn sensor_id(&mut self) -> Result<u8, E> {
        self.read8(0x0D)
    }

    /// Read and calculate the temperature in degrees celsius
    pub fn get_temperature(&mut self) -> Result<f32, E> {
        let calib = self.get_calibration_data()?;
        let temp = calib.c1 as f32 * self.traw_sc()?;
        let offset = calib.c0 as f32 / 2.0;
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
        self.read24(0x00)
    }

    fn raw_temperature(&mut self) -> Result<i32, E> {
        self.read24(0x03)
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

    pub fn pressure_oversample_rate(&mut self) -> Result<SampleRate, E> {
        let byte = self.read8(0x06)?;
        Ok(SampleRate::from_u8(byte))
    }

    pub fn pressure_sample_rate(&mut self) -> Result<u8, E> {
        let byte = self.read8(0x06)? >> 4;

        Ok(byte & 0b111)
    }

    pub fn temperature_oversample_rate(&mut self) -> Result<SampleRate, E> {
        let byte = self.read8(0x07)?;
        Ok(SampleRate::from_u8(byte))
    }

    fn calibration_data_is_available(&mut self) -> Result<bool, E> {
        Ok((self.read8(0x08)? >> 7) == 1)
    }

    pub fn sensor_data_is_ready(&mut self) -> Result<bool, E> {
        Ok(((self.read8(0x08)? >> 6) & 0b1) == 1)
    }

    /// Returns a tuple of (temperature, pressure), true if new data is available
    pub fn new_data_is_available(&mut self) -> Result<(bool, bool), E> {
        let byte = self.read8(0x08)?;
        Ok((((byte >> 5) & 1) == 1, ((byte >> 4) & 1) == 1))
    }

    pub fn get_mode(&mut self) -> Result<Mode, E> {
        Ok(Mode::from_u8(self.read8(0x08)?))
    }

    // pub fn fifo_is_enabled(&mut self) -> Result<bool, E> {
    //     let byte = self.read8(0x09)? >> 1;
    //     Ok((byte & 1) == 1)
    // }

    // pub fn fifo_status(&mut self) -> Result<FifoStatus, E> {
    //     match self.read8(0x0B)? & 0b11 {
    //         0b01 => Ok(FifoStatus::Empty),
    //         0b00 => Ok(FifoStatus::Partial),
    //         0b10 => Ok(FifoStatus::Full),
    //         _ => unreachable!("Not a valid FIFO status"),
    //     }
    // }

    // pub fn set_fifo(&mut self, value: bool) -> Result<(), E> {
    //     let mut reg = self.read8(0x09)? & 0b11111101;
    //     reg |= (value as u8) << 1;
    //     self.write(&[0x09, reg])
    // }

    // pub fn flush_fifo(&mut self) -> Result<(), E> {
    //     self.write(&[0x0C, 0x80])
    // }

    fn set_pressure_shift(&mut self, value: bool) -> Result<(), E> {
        let mut reg = self.read8(0x0E)? & 0b11111011;
        reg |= (value as u8) << 2;
        self.write(&[0x0E, reg])
    }

    fn set_temp_shift(&mut self, value: bool) -> Result<(), E> {
        let mut reg = self.read8(0x0E)? & 0b11110111;
        reg |= (value as u8) << 3;
        self.write(&[0x0E, reg])
    }

    /// Reset the sensor. This will reset all configuration registers to their default values.
    pub fn soft_reset(&mut self) -> Result<(), E> {
        self.write(&[0x0C, 0x09])
    }

    pub fn set_pressure_config(
        &mut self,
        sample: SampleRate,
        oversample: SampleRate,
    ) -> Result<(), E> {
        let byte = (sample as u8) << 4 | oversample as u8;
        // self.set_pressure_shift(oversample > SampleRate::Eight)?;
        self.write(&[0x06, byte])
    }

    pub fn set_temperature_config(
        &mut self,
        sample: SampleRate,
        oversample: SampleRate,
    ) -> Result<(), E> {
        let byte = 0x80 | (sample as u8) << 4 | oversample as u8;
        self.set_temp_shift(oversample > SampleRate::Eight)?;
        self.write(&[0x07, byte])
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), E> {
        self.write(&[0x08, mode as u8])
    }

    fn get_calibration_data(&mut self) -> Result<CalibrationData, E> {
        let mut data = [0; 2];

        self.read(0x10, &mut data)?;
        let mut c0 = ((data[0] as u16) << 4) | data[1] as u16 >> 4;
        if c0 & (1 << 11) != 0 {
            c0 |= 0xF000;
        }

        // c1
        self.read(0x11, &mut data)?;
        let mut c1 = (((data[0] & 0xF) as u16) << 8) | data[1] as u16;
        if c1 & (1 << 11) != 0 {
            c1 |= 0xF000;
        }

        // c00
        let mut data = [0; 3];
        self.read(0x13, &mut data)?;
        let c00 = if data[0] & 0x80 != 0 { 0xFFF00000 } else { 0 }
            | ((data[0] as u32) << 12)
            | ((data[1] as u32) << 4)
            | ((data[2] as u32) & 0xF0) >> 4;

        // c10
        self.read(0x15, &mut data)?;
        let c10 = if data[0] & 0x8 != 0 { 0xFFF00000 } else { 0 }
            | ((data[0] as u32) & 0x0F) << 16
            | (data[1] as u32) << 8
            | data[2] as u32;

        Ok(CalibrationData {
            c0: c0 as i16,
            c1: c1 as i16,
            c00: c00 as i32,
            c10: c10 as i32,
            c01: self.read16(0x8)? as i32,
            c11: self.read16(0x10)? as i32,
            c20: self.read16(0x12)? as i32,
            c21: self.read16(0x14)? as i32,
            c30: self.read16(0x16)? as i32,
        })
    }

    fn write(&mut self, data: &[u8]) -> Result<(), E> {
        self.i2c.write(ADDR, data)
    }

    pub fn read(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), E> {
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
    pub fn scale_factor(&self) -> f32 {
        match self {
            Self::Single => 524288.0,
            Self::Two => 1572864.0,
            Self::Four => 3670016.0,
            Self::Eight => 7864320.0,
            // Self::Sixteen => 253952.0,
            // Self::ThirtyTwo => 516096.0,
            // Self::SixtyFour => 1040384.0,
            // Self::OneTwentyEight => 2088960.0,
        }
    }

    pub fn from_u8(value: u8) -> SampleRate {
        match value & 0b111 {
            0b000 => Self::Single,
            0b001 => Self::Two,
            0b010 => Self::Four,
            0b011 => Self::Eight,
            // 0b100 => Self::Sixteen,
            // 0b101 => Self::ThirtyTwo,
            // 0b110 => Self::SixtyFour,
            // 0b111 => Self::OneTwentyEight,
            _ => unreachable!(),
        }
    }
}

impl Mode {
    pub fn from_u8(value: u8) -> Mode {
        match value & 0b111 {
            0b000 => Mode::Standby,
            0b001 => Mode::Pressure,
            0b010 => Mode::Temperature,
            0b101 => Mode::ContinuousPressure,
            0b110 => Mode::ContinuousTemperature,
            0b111 => Mode::ContinuousPressureTemperature,
            _ => unreachable!(),
        }
    }
}
