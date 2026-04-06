use stm32g0xx_hal;

pub struct Bme280<I2cPeriph, SDA, SCL> {
    i2c: stm32g0xx_hal::i2c::I2c<I2cPeriph, SDA, SCL>,
    address: u8,
    pub calibration_data: Option<CalibrationParams>,
    t_fine: i32,
}

#[derive(defmt::Format)]
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

#[derive(defmt::Format)]
pub struct RawData {
    pub temperature: u32,
    pub pressure: u32,
    pub humidity: u32
}

impl<I2cPeriph, SDA, SCL> Bme280<I2cPeriph, SDA, SCL>
where I2cPeriph: stm32g0xx_hal::i2c::Instance {
    const BME280_REGISTER_ID: u8 = 0xD0;
    const EXPECTED_ID: u8 = 0x60;

    const BME280_REGISTER_SOFTRESET: u8 = 0xE0;
    const BME280_POWERONRESET: u8 = 0x86;

    const BME280_REGISTER_CTRL_HUM: u8 = 0xF2;
    const BME280_REGISTER_CTRL_MEAS: u8 = 0xF4;

    const BME280_REGISTER_CONFIG: u8 = 0xF5;
        
    const BME280_OVERSAMPLING_1X: u8 = 0x01;
    const BME280_MODE_NORMAL: u8 = 0x03;

    const BME280_STANDBY_500MS: u8 = 0x04;
    const BME280_IIR_OFF: u8 = 0x00;
    const BME280_SPI_OFF: u8 = 0x00;

    const BME280_REGISTER_CALIBRATION1: u8 = 0x88;
    const BME280_REGISTER_CALIBRATION2: u8 = 0xE1;

    const BME280_REGISTER_PRESSURE: u8 = 0xF7;

    pub fn new(i2c: stm32g0xx_hal::i2c::I2c<I2cPeriph, SDA, SCL>, address: u8) -> Self {
        Self { 
        i2c, 
        address, 
        calibration_data: None,
        t_fine: 0,
    }
    }

    pub fn check_id(&mut self) -> bool {
        let mut buffer = [0u8; 1];
        
        match self.i2c.write_read(self.address, &[Self::BME280_REGISTER_ID], &mut buffer) {
            Ok(_) => buffer[0] == Self::EXPECTED_ID,
            Err(_) => false,
        }
    }

    fn soft_reset<D: stm32g0xx_hal::hal::delay::DelayNs>(&mut self, delay: &mut D) -> bool {
        if self.i2c.write(self.address, &[
            Self::BME280_REGISTER_SOFTRESET,
            Self::BME280_POWERONRESET
        ]).is_err() {
            return false;
        }

        delay.delay_ms(5);

        true
    }

    fn config_humidity(&mut self) -> bool {
        if self.i2c.write(self.address, &[
            Self::BME280_REGISTER_CTRL_HUM,
            Self::BME280_OVERSAMPLING_1X,
        ]).is_err() {
            return false;
        }

        true
    }

    fn config_temp_and_pressure(&mut self) -> bool {
        let to_set: u8 = (Self::BME280_OVERSAMPLING_1X << 5) | // osrs_t
                         (Self::BME280_OVERSAMPLING_1X << 2) | // osrs_p
                         (Self::BME280_MODE_NORMAL);           // mode
        
        if self.i2c.write(self.address, &[
            Self::BME280_REGISTER_CTRL_MEAS,
            to_set,
        ]).is_err() {
            return false;
        }

        true
    }

    fn config_other(&mut self) -> bool {
        let to_set: u8 = (Self::BME280_STANDBY_500MS << 5) | // t_sb
                         (Self::BME280_IIR_OFF << 2) |       // t_sb
                         (Self::BME280_SPI_OFF);             // spi3w_en
        
        if self.i2c.write(self.address, &[
            Self::BME280_REGISTER_CONFIG,
            to_set,
        ]).is_err() {
            return false;
        }

        true
    }

    fn read_calibration(&mut self) -> Result<CalibrationParams, ()> {
        let mut b1 = [0u8; 26];
        let mut b2 = [0u8; 7];

        self.i2c.write_read(self.address, &[Self::BME280_REGISTER_CALIBRATION1], &mut b1).map_err(|_| ())?;    
        self.i2c.write_read(self.address, &[Self::BME280_REGISTER_CALIBRATION2], &mut b2).map_err(|_| ())?;

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

            dig_h6: b2[6] as i8
        })
    }

    pub fn read_raw(&mut self) -> Result<RawData, ()> {
        let mut b = [0u8; 8];

        self.i2c.write_read(self.address, &[Self::BME280_REGISTER_PRESSURE], &mut b).map_err(|_| ())?;

        Ok(RawData {
            pressure: ((b[0] as u32) << 12) | ((b[1] as u32) << 4) | ((b[2] as u32) >> 4),
            temperature: ((b[3] as u32) << 12) | ((b[4] as u32) << 4) | ((b[5] as u32) >> 4),
            humidity: ((b[6] as u32) << 8) | (b[7] as u32)
        })
    }

    pub fn init_sensor<D: stm32g0xx_hal::hal::delay::DelayNs>(&mut self, delay: &mut D) -> bool {
        // 1. Check if sensor ID matches
        if !self.check_id() {
            return false
        }

        // 2. Perform reset
        if !self.soft_reset(delay) {
            return false;
        }

        // 3. Config
        // Humidity config
        if !self.config_humidity() {
            return false;
        }

        // Temperature and pressure config
        if !self.config_temp_and_pressure() {
            return false;
        }

        // Other config
        if !self.config_other() {
            return false;
        }

        // 4. Calibration params
        let calibration_data = self.read_calibration().ok();
        self.calibration_data = calibration_data;

        if self.calibration_data.is_none() {
            return false;
        }

        true
    }
}