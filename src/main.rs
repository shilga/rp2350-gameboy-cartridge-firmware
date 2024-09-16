//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! No specific hardware is specified in this example. Only output on pin 0 is tested.
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, UART1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::uart::{self};
use embassy_rp::{block::ImageDef, uart::UartTx};
use embassy_time::Timer;
use embedded_io::{ErrorType, Write};
use smart_leds::RGB8;
use {defmt_serial as _, panic_probe as _};

use static_cell::StaticCell;

mod ws2812;
use crate::ws2812::Ws2812;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"example"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_description!(c"Blinky"),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

static SERIAL: StaticCell<SerialWrapper> = StaticCell::new();

struct SerialWrapper<'a> {
    uart: UartTx<'a, UART1, embassy_rp::uart::Blocking>,
}

impl<'a> ErrorType for SerialWrapper<'a> {
    type Error = uart::Error;
}

impl<'a> Write for SerialWrapper<'a> {
    fn write(&mut self, word: &[u8]) -> Result<usize, uart::Error> {
        self.uart.blocking_write(word).unwrap();
        Ok(word.len())
    }

    fn flush(&mut self) -> Result<(), uart::Error> {
        self.uart.blocking_flush()
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let config = uart::Config::default();
    let uart = uart::UartTx::new_blocking(p.UART1, p.PIN_4, config);

    let serialwrapper = SerialWrapper { uart };

    defmt_serial::defmt_serial(SERIAL.init(serialwrapper));

    defmt::info!("Hello defmt-world!");

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let ws2812 = Ws2812::new(&mut common, sm0, p.PIN_47);

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(blink(ws2812)).unwrap();

    loop {
        defmt::info!("hello there!");
        Timer::after_millis(1500).await;
    }
}

// Declare async tasks
#[embassy_executor::task]
async fn blink(mut led: Ws2812<'static, PIO0, 0>) {
    {
        loop {
            let off = RGB8::default();
            let blue = RGB8::new(0, 0, 16);
            let red = RGB8::new(16, 0, 0);
            let green = RGB8::new(0, 16, 0);
            led.write(&off);
            Timer::after_millis(500).await;
            led.write(&blue);
            Timer::after_millis(500).await;
            led.write(&red);
            Timer::after_millis(500).await;
            led.write(&green);
            Timer::after_millis(500).await;
        }
    }
}
