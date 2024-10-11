use embassy_rp::pio::{Instance, StateMachineRx};

use core::ptr::{self, NonNull};

pub struct Mbc1<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc1<'a, 'd, PIO, SM> {
    pub fn new(rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>, current_rom_bank: *mut u32) -> Self {
        let current_rom_bank_pointer = NonNull::new(current_rom_bank).unwrap();
        Self {
            rx_fifo,
            current_rom_bank_pointer,
        }
    }

    pub fn run(&mut self) {
        let mut rom_bank = 1u8;
        let mut rom_bank_new: u8;
        let mut rom_bank_high = 0u8;
        let mut rom_bank_low = 1u8;
        let mut mode = 0u8;

        let rom_bank_mask = 0x3Fu8;

        loop {
            while self.rx_fifo.empty() {}
            let addr = self.rx_fifo.pull();
            while self.rx_fifo.empty() {}
            let data = (self.rx_fifo.pull() & 0xFFu32) as u8;

            match addr & 0xE000u32 {
                0x0000u32 => {
                    // ram enable
                }
                0x2000u32 => {
                    rom_bank_low = data & 0x1f;
                    if rom_bank_low == 0 {
                        rom_bank_low += 1;
                    }
                }
                0x4000u32 => {
                    if mode != 0 {
                        // ram bank
                    } else {
                        rom_bank_high = data & 0x03u8;
                    }
                }
                0x6000u32 => {
                    mode = data & 1u8;
                }
                _ => {}
            }

            if mode == 0 {
                rom_bank_new = (rom_bank_high << 5) | rom_bank_low;
            } else {
                rom_bank_new = rom_bank_low;
            }
            rom_bank_new = rom_bank_new & rom_bank_mask;

            if rom_bank != rom_bank_new {
                rom_bank = rom_bank_new;
                unsafe {
                    ptr::write_volatile(
                        self.current_rom_bank_pointer.as_ptr(),
                        rom_bank as u32 * 0x4000u32,
                    )
                };
            }
        }
    }
}
