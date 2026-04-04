#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32g0xx_hal::{
    prelude::*,
    stm32
};

mod bme280;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpioa.pa10.into_open_drain_output();

    let i2c = dp.I2C1.i2c(
        sda,
        scl,
        100.kHz(),
        &mut rcc,
    );

    let mut sensor = bme280::Bme280::new(i2c, 0x76);

    let mut led = gpioa.pa5.into_push_pull_output();

    if sensor.check_id() {
        led.set_high().unwrap();
    } else {
        loop {
            led.toggle().unwrap();
            cortex_m::asm::delay(1_000_000);
        }
    }

    loop {

    }
}