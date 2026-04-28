//! Driver for a 4-digit 7-segment display and status LEDs.
//!
//! This module provides structures and methods to handle hardware multiplexing
//! of a 7-segment display and controlling indicator LEDs for different modes.

use stm32g0xx_hal::{
    gpio::{
        gpioa::{PA6, PA7, PA8, PA9},
        gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7},
        gpiod::{PD0, PD1, PD2},
        Output, PushPull,
    },
    hal::delay::DelayNs,
    prelude::OutputPin,
};

/// LEDs indicating the active display mode.
pub struct ModeLeds {
    pub temp: PD0<Output<PushPull>>,
    pub hum: PD1<Output<PushPull>>,
    pub press: PD2<Output<PushPull>>,
}

impl ModeLeds {
    /// Turns on only the LED corresponding to the currently selected mode.
    /// 
    /// * `mode`: 0 for Temperature, 1 for Humidity, 2 for Pressure.
    pub fn update(&mut self, mode: u8) {
        if mode == 0 { let _ = self.temp.set_high(); } else { let _ = self.temp.set_low(); }
        if mode == 1 { let _ = self.hum.set_high(); } else { let _ = self.hum.set_low(); }
        if mode == 2 { let _ = self.press.set_high(); } else { let _ = self.press.set_low(); }
    }
}

/// Segment control pins for the 7-segment display.
pub struct Segments {
    pub a: PB0<Output<PushPull>>,
    pub b: PB1<Output<PushPull>>,
    pub c: PB2<Output<PushPull>>,
    pub d: PB3<Output<PushPull>>,
    pub e: PB4<Output<PushPull>>,
    pub f: PB5<Output<PushPull>>,
    pub g: PB6<Output<PushPull>>,
    pub dp: PB7<Output<PushPull>>,
}

/// Digit selection pins for 4-digit multiplexing.
pub struct Digits {
    pub d1: PA6<Output<PushPull>>,
    pub d2: PA7<Output<PushPull>>,
    pub d3: PA8<Output<PushPull>>,
    pub d4: PA9<Output<PushPull>>,
}

/// Segment masks for numbers 0-9 and a minus sign.
const NUMBERS: [u8; 11] = [
    0b0011_1111, // 0
    0b0000_0110, // 1
    0b0101_1011, // 2
    0b0100_1111, // 3
    0b0110_0110, // 4
    0b0110_1101, // 5
    0b0111_1101, // 6
    0b0000_0111, // 7
    0b0111_1111, // 8
    0b0110_1111, // 9
    0b0100_0000, // 10: minus sign
];

/// Display type configuration (true for Common Anode, false for Common Cathode).
const COMMON_ANODE: bool = true;

/// The order in which digits are updated during multiplexing.
const DIGIT_ORDER: [usize; 4] = [0, 1, 2, 3];

/// Display driver state containing the value buffer and active digit index.
pub struct Display7Seg {
    segments: Segments,
    digits: Digits,
    buffer: [u8; 4], 
    current_digit: usize,
}

impl Display7Seg {
    /// Creates a new display driver instance.
    pub fn new(segments: Segments, digits: Digits) -> Self {
        Self {
            segments,
            digits,
            buffer: [0; 4],
            current_digit: 0,
        }
    }

    /// Parses an integer value into the 4-digit buffer and sets an optional decimal point.
    /// 
    /// Handles negative values by reserving the first digit for the minus sign.
    /// 
    /// * `value`: The integer value to display.
    /// * `dot_position`: Index (0-3) where the decimal point should be placed.
    pub fn show_value(&mut self, value: i32, mut dot_position: usize) {
        let is_negative = value < 0;
        let mut val = value.saturating_abs();
        
        if is_negative {
            // Cap to 3 digits to leave room for the minus sign
            val /= 10;
            dot_position += 1;
            val = val.min(999);
            
            self.buffer[3] = (val % 10) as u8;
            val /= 10;
            self.buffer[2] = (val % 10) as u8;
            val /= 10;
            self.buffer[1] = (val % 10) as u8;
            
            // Set the first digit to the minus sign (index 10)
            self.buffer[0] = 10; 
        } else {
            // Cap to 4 digits for positive numbers
            val = val.min(9999);
            self.buffer[3] = (val % 10) as u8;
            val /= 10;
            self.buffer[2] = (val % 10) as u8;
            val /= 10;
            self.buffer[1] = (val % 10) as u8;
            val /= 10;
            self.buffer[0] = (val % 10) as u8;
        }

        if dot_position < 4 {
            self.buffer[dot_position] |= 0x80;
        }
    }

    /// Enables or disables a specific digit, respecting display polarity.
    fn set_digit(&mut self, idx: usize, on: bool) {
        let active = if COMMON_ANODE { !on } else { on };
        match idx {
            0 => {
                if active { let _ = self.digits.d1.set_high(); } else { let _ = self.digits.d1.set_low(); }
            }
            1 => {
                if active { let _ = self.digits.d2.set_high(); } else { let _ = self.digits.d2.set_low(); }
            }
            2 => {
                if active { let _ = self.digits.d3.set_high(); } else { let _ = self.digits.d3.set_low(); }
            }
            3 => {
                if active { let _ = self.digits.d4.set_high(); } else { let _ = self.digits.d4.set_low(); }
            }
            _ => {}
        }
    }

    /// Converts the logical segment state to physical pin state based on polarity.
    fn set_segment(pin_on: bool) -> bool {
        if COMMON_ANODE { !pin_on } else { pin_on }
    }

    /// Performs one multiplexing refresh cycle.
    /// 
    /// This method updates one digit per call to prevent blocking.
    /// It must be called repeatedly in the main application loop.
    pub fn refresh<D: DelayNs>(&mut self, delay: &mut D) {
        // Turn off all digits to prevent ghosting
        for i in 0..4 {
            self.set_digit(i, false);
        }

        let buffer_idx = DIGIT_ORDER[self.current_digit];
        let raw_data = self.buffer[buffer_idx];
        let has_dot = (raw_data & 0x80) != 0;
        let num_idx = (raw_data & 0x7F) as usize;
        
        let pattern = if num_idx < 11 { NUMBERS[num_idx] } else { 0 };

        if Self::set_segment((pattern & 0b0000_0001) != 0) { let _ = self.segments.a.set_high(); } else { let _ = self.segments.a.set_low(); }
        if Self::set_segment((pattern & 0b0000_0010) != 0) { let _ = self.segments.b.set_high(); } else { let _ = self.segments.b.set_low(); }
        if Self::set_segment((pattern & 0b0000_0100) != 0) { let _ = self.segments.c.set_high(); } else { let _ = self.segments.c.set_low(); }
        if Self::set_segment((pattern & 0b0000_1000) != 0) { let _ = self.segments.d.set_high(); } else { let _ = self.segments.d.set_low(); }
        if Self::set_segment((pattern & 0b0001_0000) != 0) { let _ = self.segments.e.set_high(); } else { let _ = self.segments.e.set_low(); }
        if Self::set_segment((pattern & 0b0010_0000) != 0) { let _ = self.segments.f.set_high(); } else { let _ = self.segments.f.set_low(); }
        if Self::set_segment((pattern & 0b0100_0000) != 0) { let _ = self.segments.g.set_high(); } else { let _ = self.segments.g.set_low(); }
        
        if Self::set_segment(has_dot) { let _ = self.segments.dp.set_high(); } else { let _ = self.segments.dp.set_low(); }

        // Turn on the current digit
        self.set_digit(self.current_digit, true);

        // Hardware delay to keep the digit illuminated long enough for persistence of vision
        delay.delay_ms(2);

        // Move to the next digit for the next cycle
        self.current_digit = (self.current_digit + 1) % 4;
    }
}