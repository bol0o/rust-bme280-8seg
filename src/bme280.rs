//! # BME280 Sensor Driver
//!
//! This module provides a high-level interface for the Bosch BME280 
//! Environmental Sensor, supporting Temperature, Pressure, and Humidity.
//!
//! It uses 32-bit fixed-point arithmetic for compensation formulas, 
//! making it ideal for microcontrollers without an FPU (like STM32G0)

use stm32g0xx_hal;

/// The BME280 driver instance.
pub struct Bme280<I2cPeriph, SDA, SCL> {
    i2c: stm32g0xx_hal::i2c::I2c<I2cPeriph, SDA, SCL>,
    address: u8,
    pub calibration_data: Option<CalibrationParams>,
    t_fine: i32,
}

/// Errors that can occur during BME280 operations.
#[derive(defmt::Format, Debug)]
pub enum BmeError {
    /// Failed to communicate with the sensor over I2C.
    I2cCommunication,
    /// The sensor at the given address did not return the expected Chip ID (0x60).
    InvalidChipId,
    /// Calibration data has not been loaded yet. Call `init_sensor` first.
    CalibrationMissing,
}

/// Factory calibration parameters stored in the sensor's non-volatile memory.
///
/// These values are unique to each individual chip and are used to 
/// compensate raw ADC readings.
#[derive(defmt::Format, Clone, Copy)]
pub struct CalibrationParams {
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
    dig_h6: i8
}

struct RawData {
    temperature: i32,
    pressure: i32,
    humidity: i32
}

/// Final compensated environmental data.
#[derive(defmt::Format, Clone, Copy)]
pub struct BmeData {
    /// Temperature in hundredths of a degree Celsius (e.g., 2512 = 25.12°C).
    pub temperature: i32,
    /// Absolute atmospheric pressure in Pascals (Pa).
    pub pressure: i32,
    /// Relative humidity in hundredths of a percent (e.g., 4532 = 45.32%).
    pub humidity: i32,
}

impl<I2cPeriph, SDA, SCL> Bme280<I2cPeriph, SDA, SCL>
where I2cPeriph: stm32g0xx_hal::i2c::Instance {
    /// Chip ID register address.
    const REGISTER_ID: u8 = 0xD0;
    /// Expected Chip ID for BME280.
    const EXPECTED_ID: u8 = 0x60;

    /// Soft reset register address.
    const REGISTER_SOFTRESET: u8 = 0xE0;
    /// Magic value to trigger a soft reset.
    const POWERONRESET: u8 = 0x86;

    /// Humidity control register.
    const REGISTER_CTRL_HUM: u8 = 0xF2;
    /// Measurement control register (Temp & Pressure).
    const REGISTER_CTRL_MEAS: u8 = 0xF4;
    /// Configuration register (Filter, Standby).
    const REGISTER_CONFIG: u8 = 0xF5;
    
    /// 1x Oversampling configuration value.
    const OVERSAMPLING_1X: u8 = 0x01;
    /// Normal mode (continuous measurement).
    const MODE_NORMAL: u8 = 0x03;
    /// Standby time of 500ms between measurements in Normal mode.
    const STANDBY_500MS: u8 = 0x04;
    /// IIR filter disabled.
    const IIR_OFF: u8 = 0x00;
    /// 3-wire SPI disabled.
    const SPI_OFF: u8 = 0x00;

    /// First block of calibration data.
    const REGISTER_CALIBRATION1: u8 = 0x88;
    /// Second block of calibration data.
    const REGISTER_CALIBRATION2: u8 = 0xE1;
    /// Starting address for reading sensor data (Pressure MSB).
    const REGISTER_PRESSURE: u8 = 0xF7;

    /// Creates a new driver instance.
    ///
    /// `address` is typically 0x76 or 0x77 depending on the SDO pin state.
    pub fn new(i2c: stm32g0xx_hal::i2c::I2c<I2cPeriph, SDA, SCL>, address: u8) -> Self {
        Self { 
            i2c,
            address, 
            calibration_data: None, 
            t_fine: 0 
        }
    }

    /// Checks if the sensor returns the correct Chip ID.
    fn check_id(&mut self) -> Result<(), BmeError> {
        let mut buffer = [0u8; 1];

        self.i2c.write_read(self.address, &[Self::REGISTER_ID], &mut buffer)
            .map_err(|_| BmeError::I2cCommunication)?;
        
        if buffer[0] == Self::EXPECTED_ID {
            Ok(())
        } else {
            Err(BmeError::InvalidChipId)
        }
    }

    /// Performs a software reset of the sensor.
    fn soft_reset<D: stm32g0xx_hal::hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), BmeError> {
        self.i2c.write(self.address, &[
            Self::REGISTER_SOFTRESET, 
            Self::POWERONRESET
        ]).map_err(|_| BmeError::I2cCommunication)?;
        
        delay.delay_ms(5);

        Ok(())
    }

    /// Configures humidity oversampling.
    fn config_humidity(&mut self) -> Result<(), BmeError> {
        self.i2c.write(self.address, &[
            Self::REGISTER_CTRL_HUM, 
            Self::OVERSAMPLING_1X
        ]).map_err(|_| BmeError::I2cCommunication)?;

        Ok(())
    }

    /// Configures temperature and pressure oversampling, and sets the power mode.
    fn config_temp_and_pressure(&mut self) -> Result<(), BmeError> {
        let to_set: u8 = (Self::OVERSAMPLING_1X << 5) | 
                         (Self::OVERSAMPLING_1X << 2) | 
                          Self::MODE_NORMAL;

        self.i2c.write(self.address, &[Self::REGISTER_CTRL_MEAS, to_set])
            .map_err(|_| BmeError::I2cCommunication)?;

        Ok(())
    }

    /// Configures the IIR filter, standby time, and SPI mode.
    fn config_other(&mut self) -> Result<(), BmeError> {
        let to_set: u8 = (Self::STANDBY_500MS << 5) | 
                         (Self::IIR_OFF << 2) | 
                          Self::SPI_OFF;

        self.i2c.write(self.address, &[Self::REGISTER_CONFIG, to_set])
            .map_err(|_| BmeError::I2cCommunication)?;

        Ok(())
    }

    /// Reads all factory calibration parameters from the sensor's ROM.
    fn read_calibration(&mut self) -> Result<CalibrationParams, BmeError> {
        let mut b1 = [0u8; 26];
        let mut b2 = [0u8; 7];

        self.i2c.write_read(self.address, &[Self::REGISTER_CALIBRATION1], &mut b1)
            .map_err(|_| BmeError::I2cCommunication)?;    
        self.i2c.write_read(self.address, &[Self::REGISTER_CALIBRATION2], &mut b2)
            .map_err(|_| BmeError::I2cCommunication)?;

        Ok(CalibrationParams {
            dig_t1: u16::from_le_bytes([b1[0], b1[1]]),
            dig_t2: i16::from_le_bytes([b1[2], b1[3]]),
            dig_t3: i16::from_le_bytes([b1[4], b1[5]]),

            dig_p1: u16::from_le_bytes([b1[6], b1[7]]),
            dig_p2: i16::from_le_bytes([b1[8], b1[9]]),
            dig_p3: i16::from_le_bytes([b1[10], b1[11]]),
            dig_p4: i16::from_le_bytes([b1[12], b1[13]]),
            dig_p5: i16::from_le_bytes([b1[14], b1[15]]),
            dig_p6: i16::from_le_bytes([b1[16], b1[17]]),
            dig_p7: i16::from_le_bytes([b1[18], b1[19]]),
            dig_p8: i16::from_le_bytes([b1[20], b1[21]]),
            dig_p9: i16::from_le_bytes([b1[22], b1[23]]),

            dig_h1: b1[25],
            dig_h2: i16::from_le_bytes([b2[0], b2[1]]),
            dig_h3: b2[2],
            dig_h4: ((b2[3] as i16) << 4) | ((b2[4] & 0x0F) as i16),
            dig_h5: ((b2[5] as i16) << 4) | (((b2[4] & 0xF0) >> 4) as i16),
            dig_h6: b2[6] as i8,
        })
    }

    /// Reads the raw ADC values from the data registers (0xF7 to 0xFE).
    fn read_raw(&mut self) -> Result<RawData, BmeError> {
        let mut b = [0u8; 8];

        self.i2c.write_read(self.address, &[Self::REGISTER_PRESSURE], &mut b)
            .map_err(|_| BmeError::I2cCommunication)?;

        Ok(RawData {
            pressure:    ((b[0] as i32) << 12) | ((b[1] as i32) << 4) | ((b[2] as i32) >> 4),
            temperature: ((b[3] as i32) << 12) | ((b[4] as i32) << 4) | ((b[5] as i32) >> 4),
            humidity:    ((b[6] as i32) << 8)  | (b[7] as i32),
        })
    }

    /// Compensates raw temperature data using 32-bit fixed-point arithmetic.
    /// Updates the internal `t_fine` variable required for pressure/humidity compensation.
    fn compensate_temp(&mut self, raw_temp: i32, cal: &CalibrationParams) -> i32 {
        let var1 = (((raw_temp >> 3) - ((cal.dig_t1 as i32) << 1)) * (cal.dig_t2 as i32)) >> 11;
        let var2 = (((((raw_temp >> 4) - (cal.dig_t1 as i32)) * ((raw_temp >> 4) - (cal.dig_t1 as i32))) >> 12) * (cal.dig_t3 as i32)) >> 14;

        self.t_fine = var1 + var2;

        (self.t_fine * 5 + 128) >> 8   
    }

    /// Compensates raw pressure data using 32-bit fixed-point arithmetic.
    /// Requires `compensate_temp` to be called first to update `t_fine`.
    fn compensate_pressure(&self, raw_pressure: i32, cal: &CalibrationParams) -> i32 {
        let mut v1 = (self.t_fine >> 1) - 64000;
        let mut v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * (cal.dig_p6 as i32);
        v2 = v2 + ((v1 * (cal.dig_p5 as i32)) << 1);
        v2 = (v2 >> 2) + ((cal.dig_p4 as i32) << 16);
        v1 = (((cal.dig_p3 as i32) * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) + (((cal.dig_p2 as i32) * v1) >> 1);
        v1 = v1 >> 18;
        v1 = ((32768 + v1) * (cal.dig_p1 as i32)) >> 15;

        if v1 == 0 { return 0; }

        let mut p = (((1048576 - raw_pressure) - (v2 >> 12)) as u32).wrapping_mul(3125);
        if p < 0x80000000 {
            p = (p << 1) / (v1 as u32);
        } else {
            p = (p / v1 as u32) * 2;
        }

        v1 = ((cal.dig_p9 as i32) * (((p >> 3) as i32 * (p >> 3) as i32) >> 13)) >> 12;
        v2 = (((p >> 2) as i32) * (cal.dig_p8 as i32)) >> 13;
        
        p as i32 + ((v1 + v2 + cal.dig_p7 as i32) >> 4)
    }

    /// Compensates raw humidity data using 32-bit fixed-point arithmetic.
    /// Requires `compensate_temp` to be called first to update `t_fine`.
    fn compensate_humidity(&self, raw_humidity: i32, cal: &CalibrationParams) -> i32 {
        let mut h = self.t_fine - 76800;
        h = ((((raw_humidity << 14) - ((cal.dig_h4 as i32) << 20) - ((cal.dig_h5 as i32) * h)) + 16384) >> 15) 
            * (((((((h * (cal.dig_h6 as i32)) >> 10) 
            * (((h * (cal.dig_h3 as i32)) >> 11) + 32768)) >> 10) + 2097152) 
            * (cal.dig_h2 as i32) + 8192) >> 14);
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * (cal.dig_h1 as i32)) >> 4);
        
        if h < 0 { h = 0; } else if h > 419430400 { h = 419430400; }
        h >> 12
    }

    /// Initializes the sensor by verifying its ID, performing a soft reset,
    /// configuring measurement parameters, and loading calibration data.
    ///
    /// # Errors
    /// Returns [`BmeError::InvalidChipId`] if the sensor is not found.
    pub fn init_sensor<D: stm32g0xx_hal::hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), BmeError> {
        self.check_id()?;
        self.soft_reset(delay)?;
        self.config_humidity()?;
        self.config_temp_and_pressure()?;
        self.config_other()?;

        let calibration_data = self.read_calibration()?;
        self.calibration_data = Some(calibration_data);

        Ok(())
    }

    /// Reads raw ADC values and applies compensation formulas.
    ///
    /// This is the primary method to get human-readable data from the sensor.
    /// It returns a [`BmeData`] struct containing temperature, pressure, and humidity.
    ///
    /// # Errors
    /// Returns [`BmeError::CalibrationMissing`] if called before [`Self::init_sensor`].
    pub fn get_data(&mut self) -> Result<BmeData, BmeError> {
        // Check if calibration data was read
        let cal = self.calibration_data.ok_or(BmeError::CalibrationMissing)?;
        let raw = self.read_raw()?;
        
        let t = self.compensate_temp(raw.temperature, &cal);
        let p = self.compensate_pressure(raw.pressure, &cal);
        let h_q10 = self.compensate_humidity(raw.humidity, &cal);
        
        Ok(BmeData {
            temperature: t,
            pressure: p,
            humidity: (h_q10 * 100) >> 10,
        })
    }

}