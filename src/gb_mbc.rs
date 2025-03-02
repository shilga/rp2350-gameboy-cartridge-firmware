use embassy_rp::pio::{Instance, StateMachineRx};

use core::ptr::{self, NonNull};

pub trait MbcRamControl {
    fn enable_ram_access(&mut self);
    fn disable_ram_access(&mut self);
}

pub trait Mbc {
    fn run(&mut self);
}

pub struct NoMbc {}

impl Mbc for NoMbc {
    fn run(&mut self) {
        loop {}
    }
}

pub struct Mbc1<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
    ram_control: &'a mut dyn MbcRamControl,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc1<'a, 'd, PIO, SM> {
    pub fn new(
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        current_rom_bank: *mut u32,
        ram_control: &'a mut dyn MbcRamControl,
    ) -> Self {
        let current_rom_bank_pointer = NonNull::new(current_rom_bank).unwrap();
        Self {
            rx_fifo,
            current_rom_bank_pointer,
            ram_control,
        }
    }
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc for Mbc1<'a, 'd, PIO, SM> {
    fn run(&mut self) {
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
                    let ram_enabled = (data & 0x0F) == 0x0A;
                    if ram_enabled {
                        self.ram_control.enable_ram_access();
                    } else {
                        self.ram_control.disable_ram_access();
                    }
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

pub struct Mbc3<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
    current_ram_bank_pointer: NonNull<*mut u8>,
    gb_ram_memory: &'a mut [u8],
    ram_control: &'a mut dyn MbcRamControl,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc3<'a, 'd, PIO, SM> {
    pub fn new(
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        current_rom_bank: *mut u32,
        ram_bank_pointer: *mut *mut u8,
        gb_ram_memory: &'a mut [u8],
        ram_control: &'a mut dyn MbcRamControl,
    ) -> Self {
        let current_rom_bank_pointer = NonNull::new(current_rom_bank).unwrap();
        let current_ram_bank_pointer = NonNull::new(ram_bank_pointer).unwrap();
        Self {
            rx_fifo,
            current_rom_bank_pointer,
            current_ram_bank_pointer,
            gb_ram_memory,
            ram_control,
        }
    }
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc for Mbc3<'a, 'd, PIO, SM> {
    fn run(&mut self) {
        let mut rom_bank = 1u8;
        let mut rom_bank_new = 1u8;
        let mut ram_bank = 1u8;
        let mut ram_bank_new = 1u8;

        let rom_bank_mask = 0x7Fu8;

        loop {
            while self.rx_fifo.empty() {}
            let addr = self.rx_fifo.pull();
            while self.rx_fifo.empty() {}
            let data = (self.rx_fifo.pull() & 0xFFu32) as u8;

            match addr & 0xE000u32 {
                0x0000u32 => {
                    let ram_enabled = (data & 0x0F) == 0x0A;
                    if ram_enabled {
                        self.ram_control.enable_ram_access();
                    } else {
                        self.ram_control.disable_ram_access();
                    }
                }
                0x2000u32 => {
                    rom_bank_new = (data & 0x7F);
                    if rom_bank_new == 0x00 {
                        rom_bank_new = 0x01;
                    }
                }
                0x4000u32 => {
                    ram_bank_new = data;
                }
                0x6000u32 => {
                    // rtc latch
                }
                _ => {}
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

            if ram_bank != ram_bank_new {
                ram_bank = ram_bank_new;

                unsafe {
                    ptr::write_volatile(
                        self.current_ram_bank_pointer.as_ptr(),
                        self.gb_ram_memory
                            .as_mut_ptr()
                            .add((ram_bank & 0x03) as usize * 0x2000usize),
                    )
                }
            }
        }
    }
}

pub struct Mbc5<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
    current_ram_bank_pointer: NonNull<*mut u8>,
    gb_ram_memory: &'a mut [u8],
    ram_control: &'a mut dyn MbcRamControl,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc5<'a, 'd, PIO, SM> {
    pub fn new(
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        current_rom_bank: *mut u32,
        ram_bank_pointer: *mut *mut u8,
        gb_ram_memory: &'a mut [u8],
        ram_control: &'a mut dyn MbcRamControl,
    ) -> Self {
        let current_rom_bank_pointer = NonNull::new(current_rom_bank).unwrap();
        let current_ram_bank_pointer = NonNull::new(ram_bank_pointer).unwrap();
        Self {
            rx_fifo,
            current_rom_bank_pointer,
            current_ram_bank_pointer,
            gb_ram_memory,
            ram_control,
        }
    }
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc for Mbc5<'a, 'd, PIO, SM> {
    fn run(&mut self) {
        let mut rom_bank = 1u16;
        let mut rom_bank_new = 1u16;
        let mut ram_bank = 1u8;
        let mut ram_bank_new = 1u8;

        let rom_bank_mask = 0x1FFu16;
        let ram_bank_mask = 0x0Fu8;

        loop {
            while self.rx_fifo.empty() {}
            let addr = self.rx_fifo.pull();
            while self.rx_fifo.empty() {}
            let data = (self.rx_fifo.pull() & 0xFFu32) as u8;

            match addr & 0xF000u32 {
                0x0000u32 | 0x1000u32 => {
                    let ram_enabled = (data & 0x0F) == 0x0A;
                    if ram_enabled {
                        self.ram_control.enable_ram_access();
                    } else {
                        self.ram_control.disable_ram_access();
                    }
                }
                0x2000u32 => {
                    rom_bank_new = (rom_bank & 0x0100) | data as u16;
                }
                0x3000u32 => {
                    rom_bank_new = (rom_bank & 0x00FF) | (((data as u16) << 8) & 0x0100);
                }
                0x4000u32 => {
                    ram_bank_new = data & 0x0F;
                }
                _ => {}
            }

            rom_bank_new = rom_bank_new & rom_bank_mask;
            ram_bank_new = ram_bank_new & ram_bank_mask;

            if rom_bank != rom_bank_new {
                rom_bank = rom_bank_new;
                unsafe {
                    ptr::write_volatile(
                        self.current_rom_bank_pointer.as_ptr(),
                        rom_bank as u32 * 0x4000u32,
                    )
                };
            }

            if ram_bank != ram_bank_new {
                ram_bank = ram_bank_new;

                unsafe {
                    ptr::write_volatile(
                        self.current_ram_bank_pointer.as_ptr(),
                        self.gb_ram_memory
                            .as_mut_ptr()
                            .add((ram_bank & 0x03) as usize * 0x2000usize),
                    )
                }
            }
        }
    }
}
