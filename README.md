# STM32G0 Environmental Monitor (BME280 + 7-Segment Display)

An embedded Rust `no_std` application for the STM32G0 microcontroller that reads environmental data (Temperature, Pressure, Humidity) from a Bosch BME280 sensor and displays it on a multiplexed 4-digit 7-segment display.

## Features

* **Sensors:** Reads temperature, humidity, and atmospheric pressure from the BME280 sensor via I2C.
* **Math:** Uses 32-bit fixed-point arithmetic for BME280 compensation formulas.
* **Display:** Non-blocking hardware multiplexing of a 4-digit 7-segment display.
* **Interrupt-driven UI:** Uses EXTI (External Interrupts) to handle a push-button that toggles the display mode.
* **Status Indication:** Dedicated LEDs to indicate the currently displayed metric (Temperature, Humidity, or Pressure).
* **Logging:** Fast, minimal-overhead logging over standard debugging interfaces using `defmt` and `defmt-rtt`.

## Hardware Requirements

* **Microcontroller:** STM32G0 series (supported by `stm32g0xx-hal`).
* **Sensor:** BME280 (I2C interface).
* **Display:** 4-digit 7-segment display.
* **Components:**
  * 3x LEDs (for mode indication).
  * 1x Push button.
  * Appropriate current-limiting resistors for the LEDs and the 7-segment display.

## Pinout / Wiring

Based on the hardware configuration in `main.rs`, wire your components as follows:

### BME280 Sensor (I2C1)
* **SDA:** `PA10`
* **SCL:** `PB8`

### 7-Segment Display
**Segments (A-G, DP)**
* **A:** `PB0`
* **B:** `PB1`
* **C:** `PB2`
* **D:** `PB3`
* **E:** `PB4`
* **F:** `PB5`
* **G:** `PB6`
* **DP:** `PB7`

**Digit Selectors (D1-D4)**
* **D1:** `PA6`
* **D2:** `PA7`
* **D3:** `PA8`
* **D4:** `PA9`

### User Interface
* **Mode Switch Button:** `PD4` (Configured as floating input; requires external pull-up and debouncing capacitor).
* **Temperature LED:** `PD0`
* **Humidity LED:** `PD1`
* **Pressure LED:** `PD2`

## Project Structure

* `src/main.rs`: Main application loop, hardware initialization, EXTI interrupt service routine, and display scheduling.
* `src/bme280.rs`: A custom, lightweight I2C driver for the BME280 sensor including reading calibration data and fixed-point data compensation.
* `src/display.rs`: Driver for handling the physical multiplexing of the 7-segment display, digit mapping, and status LED control.

## Building & Flashing

This project is configured for the thumbv6m-none-eabi target.

**Prerequisites**

1. **Install the rust target:**
```bash
rustup target add thumbv6m-none-eabi
```
2. **Install probe-rs:**
```bash
cargo install probe-rs-tools
```

**Running the Application**

Since the runner is already defined in .cargo/config.toml, you can build, flash, and start logging with a single command:

```bash
cargo run --release
```

The `probe-rs` tool will automatically:

1. Compile the code using link.x and defmt.x linker scripts.

2. Flash the binary to the STM32G0B1RETx.

3. Reset the chip and start the application.

4. Stream `defmt` logs (Temperature, Pressure, Humidity) directly to your terminal.

## Authors
- **Paweł Bolek** (Hardware, BME280 driver)
- **Kamil Kaczmarczyk** (7 segment display driver)