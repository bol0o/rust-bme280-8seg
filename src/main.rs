#![no_std]
#![no_main]

use cortex_m_rt::entry;
use stm32g0xx_hal::{
    prelude::*, stm32
};

use defmt_rtt as _;
use panic_probe as _;

mod bme280;
use bme280::Bme280;

mod display;
use display::{ModeLeds, Display7Seg, Segments, Digits};

defmt::timestamp!("{=u32}", 0);

use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use stm32g0xx_hal::{
    exti::{Event, ExtiExt},
    gpio::SignalEdge,
    pac::interrupt,
};

// Shared state used by main loop and EXTI interrupt.
static EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
static DISPLAY_MODE: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static MODE_CHANGE_PENDING: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

const DISPLAY_REFRESH_MS: u32 = 2;
const SENSOR_READ_INTERVAL_MS: u32 = 1000;
const SENSOR_READ_EVERY_LOOPS: u32 = SENSOR_READ_INTERVAL_MS / DISPLAY_REFRESH_MS;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Core clock and SysTick delay provider.
    let mut rcc = dp.RCC.constrain();
    let mut delay = cp.SYST.delay(&mut rcc);

    // Split GPIO ports into independent pins.
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiod = dp.GPIOD.split(&mut rcc);

    let led_temp = gpiod.pd0.into_push_pull_output();
    let led_hum = gpiod.pd1.into_push_pull_output();
    let led_press = gpiod.pd2.into_push_pull_output();

    let mut mode_leds = ModeLeds {
        temp: led_temp,
        hum: led_hum,
        press: led_press,
    };

    // Configure mode button on PD4 with EXTI interrupt.
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

    // Wire 7-segment display pins.
    let segments = Segments {
        a: gpiob.pb0.into_push_pull_output(),
        b: gpiob.pb1.into_push_pull_output(),
        c: gpiob.pb2.into_push_pull_output(),
        d: gpiob.pb3.into_push_pull_output(),
        e: gpiob.pb4.into_push_pull_output(),
        f: gpiob.pb5.into_push_pull_output(),
        g: gpiob.pb6.into_push_pull_output(),
        dp: gpiob.pb7.into_push_pull_output(),
    };

    let digits = Digits {
        d1: gpioa.pa6.into_push_pull_output(),
        d2: gpioa.pa7.into_push_pull_output(),
        d3: gpioa.pa8.into_push_pull_output(),
        d4: gpioa.pa9.into_push_pull_output(),
    };

    let mut screen = Display7Seg::new(segments, digits);


    // Initialize BME280.
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

    if let Err(e) = bme280.init_sensor(&mut delay) {
        defmt::panic!("Błąd inicjalizacji BME280: {:?}", e);
    }

    defmt::info!("BME280 zainicjalizowany pomyślnie");

    if let Some(cal) = &bme280.calibration_data {
        defmt::debug!("Dane kalibracyjne: {:?}", cal);
    }

    // Loop counter for sensor polling.
    let mut loop_counter: u32 = 0;

    loop {
        cortex_m::interrupt::free(|cs| {
            if MODE_CHANGE_PENDING.borrow(cs).get() {
                MODE_CHANGE_PENDING.borrow(cs).set(false);

                let current_mode = DISPLAY_MODE.borrow(cs).get();
                let next_mode = (current_mode + 1) % 3;
                DISPLAY_MODE.borrow(cs).set(next_mode);
            }
        });

        let mode = cortex_m::interrupt::free(|cs| DISPLAY_MODE.borrow(cs).get());
        mode_leds.update(mode);

        // Keep display refreshed continuously (multiplexing).
        screen.refresh(&mut delay);

        // Poll sensor roughly once per second.
        loop_counter += 1;
        if loop_counter >= SENSOR_READ_EVERY_LOOPS {
            loop_counter = 0;

            match bme280.get_data() {
                Ok(data) => {
                    // Update display payload based on selected mode.
                    defmt::info!(
                        "Temp: {}.{:02} °C | Pres: {}.{:02} hPa | Hum: {}.{:02} %",
                        data.temperature / 100,
                        (data.temperature % 100).abs(),
                        data.pressure / 100,
                        data.pressure % 100,
                        data.humidity / 100,
                        data.humidity % 100
                    );

                    match mode {
                        0 => {
                            // Temperature: e.g. 24.50 -> 2450 with decimal point.
                            screen.show_value(data.temperature, 1);
                        }
                        1 => {
                            // Pressure: show whole hPa value on 4 digits.
                            screen.show_value(data.pressure / 100, 4);
                        }
                        2 => {
                            // Humidity: e.g. 45.30 -> 4530 with decimal point.
                            screen.show_value(data.humidity, 1);
                        }
                        _ => {}
                    }
                }
                Err(e) => {
                    defmt::error!("Błąd odczytu z czujnika: {:?}", e);
                }
            }
        }
    }
}

#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        if let Some(exti) = EXTI.borrow(cs).borrow_mut().as_mut() {
            exti.unpend(Event::GPIO4);
        }

        MODE_CHANGE_PENDING.borrow(cs).set(true);
    });
}
