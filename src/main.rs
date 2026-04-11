#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32g0xx_hal::{
    hal::delay::DelayNs, prelude::*, stm32
};

use defmt_rtt as _;
use panic_probe as _;

mod bme280;
use bme280::Bme280;

defmt::timestamp!("{=u32}", 0);

use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use stm32g0xx_hal::{
    exti::{Event, ExtiExt},
    gpio::SignalEdge,
    pac::interrupt,
};

//global
static EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
static DISPLAY_MODE: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();

    let mut delay = cp.SYST.delay(&mut rcc);

    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);

    let gpiod = dp.GPIOD.split(&mut rcc);

    let mut exti = dp.EXTI;

    gpiod
        .pd4
        .into_pull_up_input()
        .listen(SignalEdge::Falling, &mut exti);

    cortex_m::interrupt::free(|cs| {
        EXTI.borrow(cs).replace(Some(exti));
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::EXTI4_15);
    }

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

    match bme280.init_sensor(&mut delay) {
        Ok(_) => defmt::info!("BME280 zainicjalizowany pomyślnie"),
        Err(e) => defmt::error!("Błąd inicjalizacji BME280: {:?}", e),
    }

    if let Some(cal) = &bme280.calibration_data {
        defmt::debug!("Dane kalibracyjne: {:?}", cal);
    }

    loop {
        match bme280.get_data() {
            Ok(data) => {
                defmt::info!(
                    "Temp: {}.{:02} °C | Pres: {}.{:02} hPa | Hum: {}.{:02} %",
                    data.temperature / 100,
                    (data.temperature % 100).abs(),
                    data.pressure / 100,
                    data.pressure % 100,
                    data.humidity / 100,
                    data.humidity % 100
                );
            }
            Err(e) => {
                defmt::error!("Błąd odczytu z czujnika: {:?}", e);
            }
        }

        delay.delay_ms(1000);
    }
}

#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        if let Some(exti) = EXTI.borrow(cs).borrow_mut().as_mut() {
            exti.unpend(Event::GPIO4);
        }

        let current_mode = DISPLAY_MODE.borrow(cs).get();
        let next_mode = if current_mode >= 2 { 0 } else { current_mode + 1 };
        DISPLAY_MODE.borrow(cs).set(next_mode);

        defmt::info!("Mode changed {}", next_mode);
    });
}