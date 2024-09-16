//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! No specific hardware is specified in this example. Only output on pin 0 is tested.
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::peripherals::{PIO0, UART1};
use embassy_rp::pio::{
    Common, Config, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::uart::{self};
use embassy_rp::{bind_interrupts, clocks};
use embassy_rp::{block::ImageDef, uart::UartTx};
use embassy_time::Timer;
use embedded_io::{ErrorType, Write};
use fixed::types::U24F8;
use fixed_macro::fixed;
use smart_leds::RGB8;
use {defmt_serial as _, panic_probe as _};

use static_cell::StaticCell;

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

pub struct Ws2812<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    pub fn new(pio: &mut Common<'d, P>, mut sm: StateMachine<'d, P, S>, pin: impl PioPin) -> Self {
        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);
        cfg.set_in_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self { sm }
    }

    pub fn write(&mut self, color: &RGB8) {
        // Precompute the word bytes from the color
        let word =
            (u32::from(color.g) << 24) | (u32::from(color.r) << 16) | (u32::from(color.b) << 8);

        // push to sm
        self.sm.tx().push(word);

        // Timer::after_micros(55).await;
    }
}

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
            led.write(&off);
            Timer::after_millis(500).await;
            led.write(&blue);
            Timer::after_millis(500).await;
        }
    }
}
