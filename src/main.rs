#![no_std]
#![no_main]

//! Main application entry point for the STM32G0 environmental sensor project.
//! Handles BME280 sensor initialization, multiplexing of the 7-segment display,
//! and EXTI interrupts for button-based display mode switching.

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

/// Shared state for the EXTI peripheral to allow clearing interrupt pending bits.
static EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));

/// Shared state holding the current display mode (0: Temp, 1: Press, 2: Hum).
static DISPLAY_MODE: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

/// Shared state flag indicating that a mode change was requested via the button.
static MODE_CHANGE_PENDING: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// Display multiplexing refresh rate in milliseconds.
const DISPLAY_REFRESH_MS: u32 = 2;

/// Interval for polling the BME280 sensor in milliseconds.
const SENSOR_READ_INTERVAL_MS: u32 = 1000;

/// Number of display refresh loops required to trigger a sensor read.
const SENSOR_READ_EVERY_LOOPS: u32 = SENSOR_READ_INTERVAL_MS / DISPLAY_REFRESH_MS;

/// Main execution loop.
#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure core clock and SysTick delay provider.
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

    // Configure the mode switch button on PD4 with EXTI interrupt on falling edge.
    let mut exti = dp.EXTI;

    gpiod
        .pd4
        .into_pull_up_input()
        .listen(SignalEdge::Falling, &mut exti);

    // Store the EXTI peripheral in the shared Mutex for ISR access.
    cortex_m::interrupt::free(|cs| {
        EXTI.borrow(cs).replace(Some(exti));
    });

    // Unmask the EXTI4_15 interrupt line in the NVIC.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::EXTI4_15);
    }

    // Configure the 7-segment display segment pins.
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

    // Configure the 7-segment display digit selector pins.
    let digits = Digits {
        d1: gpioa.pa6.into_push_pull_output(),
        d2: gpioa.pa7.into_push_pull_output(),
        d3: gpioa.pa8.into_push_pull_output(),
        d4: gpioa.pa9.into_push_pull_output(),
    };

    let mut screen = Display7Seg::new(segments, digits);

    // Initialize the I2C bus and BME280 sensor.
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
        Err(e) => defmt::panic!("Błąd inicjalizacji BME280: {:?}", e),
    }

    if let Some(cal) = &bme280.calibration_data {
        defmt::debug!("Dane kalibracyjne: {:?}", cal);
    }

    // Loop counter for scheduling sensor reads without blocking multiplexing.
    let mut loop_counter: u32 = 0;

    loop {
        // Safely check and process the mode change flag set by the ISR.
        cortex_m::interrupt::free(|cs| {
            if MODE_CHANGE_PENDING.borrow(cs).get() {
                MODE_CHANGE_PENDING.borrow(cs).set(false);

                let current_mode = DISPLAY_MODE.borrow(cs).get();
                let next_mode = (current_mode + 1) % 3;
                DISPLAY_MODE.borrow(cs).set(next_mode);
            }
        });

        // Retrieve the current mode to update LEDs and display payload.
        let mode = cortex_m::interrupt::free(|cs| DISPLAY_MODE.borrow(cs).get());
        mode_leds.update(mode);

        // Keep the display refreshed continuously (hardware multiplexing cycle).
        screen.refresh(&mut delay);

        // Poll the sensor roughly once per second.
        loop_counter += 1;
        if loop_counter >= SENSOR_READ_EVERY_LOOPS {
            loop_counter = 0;

            match bme280.get_data() {
                Ok(data) => {
                    // Log the environmental data via defmt.
                    defmt::info!(
                        "Temp: {}.{:02} °C | Pres: {}.{:02} hPa | Hum: {}.{:02} %",
                        data.temperature / 100,
                        (data.temperature % 100).abs(),
                        data.pressure / 100,
                        data.pressure % 100,
                        data.humidity / 100,
                        data.humidity % 100
                    );

                    // Update the display buffer based on the selected mode.
                    match mode {
                        0 => {
                            // Temperature: e.g. 24.50 -> 2450 with decimal point at position 1.
                            screen.show_value(data.temperature, 1);
                        }
                        1 => {
                            // Pressure: show whole hPa value across all 4 digits.
                            screen.show_value(data.pressure / 100, 4);
                        }
                        2 => {
                            // Humidity: e.g. 45.30 -> 4530 with decimal point at position 1.
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

/// EXTI interrupt service routine handling the PD4 button press.
#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        // Clear the pending interrupt flag to avoid infinite ISR loops.
        if let Some(exti) = EXTI.borrow(cs).borrow_mut().as_mut() {
            exti.unpend(Event::GPIO4);
        }

        // Signal the main loop that the button was pressed.
        MODE_CHANGE_PENDING.borrow(cs).set(true);
    });
}