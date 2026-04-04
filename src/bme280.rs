use stm32g0xx_hal::i2c::I2c;

pub struct Bme280<I2cPeriph, SDA, SCL> {
    i2c: I2c<I2cPeriph, SDA, SCL>,
    address: u8,
}

impl<I2cPeriph, SDA, SCL> Bme280<I2cPeriph, SDA, SCL>
where I2cPeriph: stm32g0xx_hal::i2c::Instance {
    const REG_ID: u8 = 0xD0;
    const EXPECTED_ID: u8 = 0x60;

    pub fn new(i2c: I2c<I2cPeriph, SDA, SCL>, address: u8) -> Self {
        Self { i2c, address }
    }

    pub fn check_id(&mut self) -> bool {
        let mut buffer = [0u8; 1];
        
        match self.i2c.write_read(self.address, &[Self::REG_ID], &mut buffer) {
            Ok(_) => buffer[0] == Self::EXPECTED_ID,
            Err(_) => false,
        }
    }
}