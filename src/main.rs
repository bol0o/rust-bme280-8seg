#![no_std]
#![no_main]

// use panic_halt as _;
use cortex_m_rt::entry;
use stm32g0xx_hal::{
    hal::delay::DelayNs, prelude::*, stm32
};

use defmt_rtt as _; // Transport logów przez interfejs debugowania
use panic_probe as _; // Jeśli program spanikuje, wypisze to w konsoli

mod bme280;
use bme280::Bme280;

defmt::timestamp!("{=u32}", 0);

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let mut delay = cp.SYST.delay(&mut rcc);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    // Init BME280
    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpioa.pa10.into_open_drain_output();
    
    let i2c = dp.I2C1.i2c(
        sda,
        scl,
        100.kHz(),
        &mut rcc,
    );
    
    let mut bme280 = Bme280::new(i2c, 0x76);

    defmt::info!("Inicjalizowanie BME280...");
    if bme280.init_sensor(&mut delay) {
        defmt::info!("BME280 zainicjalizowany pomyślnie");
    } else {
        defmt::error!("Błąd inicjalizacji BME280");
    }


    if let Some(cal) = &bme280.calibration_data {
        defmt::debug!("Dane kalibracyjne: {:?}", cal)
    }


    loop {
        match bme280.read_raw() {
            Ok(raw) => {
                defmt::debug!("Surowe dane: {:?}", raw);
            }
            Err(_) => defmt::error!("Błąd odczytu!"),
        }
        delay.delay_ms(1000);
    }
}