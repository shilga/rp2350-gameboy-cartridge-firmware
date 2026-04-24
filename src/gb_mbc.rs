/* RP2350 GameBoy cartridge
 * Copyright (C) 2025 Sebastian Quilitz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

use embassy_rp::pio::{Instance, StateMachineRx};

use core::ptr::{self, NonNull};

pub trait MbcRamControl {
    fn enable_ram_access(&mut self);
    fn enable_ram_read_access(&mut self);
    fn disable_ram_access(&mut self);
    fn enable_rtc_access(&mut self);
    fn enable_huc3_io_access(&mut self, read_ptr: *mut u8, write_ptr: *mut u8);
}

pub trait MbcRtcControl {
    fn process(&mut self);
    fn trigger_latch(&mut self);
    fn activate_register(&mut self, reg_num: u8);
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
    current_ram_bank_pointer: NonNull<*mut u8>,
    gb_ram_memory: &'a mut [u8],
    ram_control: &'a mut dyn MbcRamControl,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc1<'a, 'd, PIO, SM> {
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
                        unsafe {
                            ptr::write_volatile(
                                self.current_ram_bank_pointer.as_ptr(),
                                self.gb_ram_memory
                                    .as_mut_ptr()
                                    .add((data & 0x03) as usize * 0x2000usize),
                            )
                        }
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
    rtc_control: &'a mut dyn MbcRtcControl,
    rom_bank_mask: u8,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc3<'a, 'd, PIO, SM> {
    pub fn new(
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        current_rom_bank: *mut u32,
        ram_bank_pointer: *mut *mut u8,
        gb_ram_memory: &'a mut [u8],
        ram_control: &'a mut dyn MbcRamControl,
        rtc_control: &'a mut dyn MbcRtcControl,
        rom_bank_count: u16,
    ) -> Self {
        rtc_control.process(); /* pre process RTC here */
        let current_rom_bank_pointer = NonNull::new(current_rom_bank).unwrap();
        let current_ram_bank_pointer = NonNull::new(ram_bank_pointer).unwrap();
        let rom_bank_mask = if rom_bank_count > 128 { 0xFF } else { 0x7F };
        Self {
            rx_fifo,
            current_rom_bank_pointer,
            current_ram_bank_pointer,
            gb_ram_memory,
            ram_control,
            rtc_control,
            rom_bank_mask,
        }
    }
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc for Mbc3<'a, 'd, PIO, SM> {
    fn run(&mut self) {
        let mut rom_bank = 1u8;
        let mut rom_bank_new = 1u8;
        let mut ram_bank = 1u8;
        let mut ram_enabled = false;
        let mut rtc_latch = false;

        loop {
            while self.rx_fifo.empty() {
                self.rtc_control.process();
            }
            let addr = self.rx_fifo.pull();
            while self.rx_fifo.empty() {}
            let data = (self.rx_fifo.pull() & 0xFFu32) as u8;

            match addr & 0xE000u32 {
                0x0000u32 => {
                    ram_enabled = (data & 0x0F) == 0x0A;
                    if ram_enabled {
                        if ram_bank & 0x08u8 == 0x08u8 {
                            self.ram_control.enable_rtc_access();
                        } else {
                            self.ram_control.enable_ram_access();
                        }
                    } else {
                        self.ram_control.disable_ram_access();
                    }
                }
                0x2000u32 => {
                    rom_bank_new = data & self.rom_bank_mask;
                    if rom_bank_new == 0x00 {
                        rom_bank_new = 0x01;
                    }
                }
                0x4000u32 => {
                    ram_bank = data;
                    if ram_bank & 0x08u8 == 0x08u8 {
                        self.rtc_control.activate_register(ram_bank & 0x07u8);
                        if ram_enabled {
                            self.ram_control.enable_rtc_access();
                        }
                    } else {
                        unsafe {
                            ptr::write_volatile(
                                self.current_ram_bank_pointer.as_ptr(),
                                self.gb_ram_memory
                                    .as_mut_ptr()
                                    .add((ram_bank & 0x07) as usize * 0x2000usize),
                            )
                        }
                        if ram_enabled {
                            self.ram_control.enable_ram_access();
                        }
                    }
                }
                0x6000u32 => {
                    if data != 0 {
                        if !rtc_latch {
                            rtc_latch = true;
                            self.rtc_control.trigger_latch();
                        }
                    } else {
                        rtc_latch = false;
                    }
                }
                _ => {}
            }

            rom_bank_new &= self.rom_bank_mask;
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

pub struct Mbc5<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
    current_ram_bank_pointer: NonNull<*mut u8>,
    gb_ram_memory: &'a mut [u8],
    ram_control: &'a mut dyn MbcRamControl,
}

pub struct Huc3<'a, 'd, PIO: Instance, const SM: usize> {
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    current_rom_bank_pointer: NonNull<u32>,
    current_ram_bank_pointer: NonNull<*mut u8>,
    gb_ram_memory: &'a mut [u8],
    ram_control: &'a mut dyn MbcRamControl,
    command_arg: u8,
    response: u8,
    semaphore: u8,
    ir: u8,
    rtc_memory: [u8; 256],
    rtc_addr: u8,
}

impl<'a, 'd, PIO: Instance, const SM: usize> Huc3<'a, 'd, PIO, SM> {
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
            command_arg: 0x80,
            response: 0x80,
            semaphore: 0x81,
            ir: 0x80,
            rtc_memory: [0u8; 256],
            rtc_addr: 0,
        }
    }

    fn process_io(&mut self) {
        if self.semaphore & 0x01 == 0 {
            self.execute_command();
            self.semaphore = 0x81;
        }
    }

    fn execute_command(&mut self) {
        let command_arg = self.command_arg & 0x7F;
        let command = (command_arg >> 4) & 0x07;
        let argument = command_arg & 0x0F;
        let mut result = 0u8;

        match command {
            0x01 => {
                result = self.rtc_memory[self.rtc_addr as usize] & 0x0F;
                self.rtc_addr = self.rtc_addr.wrapping_add(1);
            }
            0x03 => {
                self.rtc_memory[self.rtc_addr as usize] = argument;
                self.rtc_addr = self.rtc_addr.wrapping_add(1);
            }
            0x04 => {
                self.rtc_addr = (self.rtc_addr & 0xF0) | argument;
            }
            0x05 => {
                self.rtc_addr = (self.rtc_addr & 0x0F) | (argument << 4);
            }
            0x06 => {
                result = match argument {
                    0x02 => 0x01,
                    _ => 0x00,
                };
            }
            _ => {}
        }

        self.response = 0x80 | (command_arg & 0x70) | result;
    }
}

impl<'a, 'd, PIO: Instance, const SM: usize> Mbc for Huc3<'a, 'd, PIO, SM> {
    fn run(&mut self) {
        let mut rom_bank = 1u8;
        let mut rom_bank_new = 1u8;
        let mut ram_bank = 0u8;
        let mut ram_bank_new = 0u8;

        let rom_bank_mask = 0x7Fu8;
        let ram_bank_mask = 0x03u8;

        loop {
            while self.rx_fifo.empty() {
                self.process_io();
            }
            let addr = self.rx_fifo.pull();
            while self.rx_fifo.empty() {}
            let data = (self.rx_fifo.pull() & 0xFFu32) as u8;

            match addr & 0xE000u32 {
                0x0000u32 => match data & 0x0F {
                    0x00u8 => {
                        self.ram_control.enable_ram_read_access();
                    }
                    0x0Au8 => {
                        self.ram_control.enable_ram_access();
                    }
                    0x0Bu8 => {
                        self.ram_control
                            .enable_huc3_io_access(&mut self.command_arg, &mut self.command_arg);
                    }
                    0x0Cu8 => {
                        self.ram_control
                            .enable_huc3_io_access(&mut self.response, &mut self.response);
                    }
                    0x0Du8 => {
                        self.ram_control
                            .enable_huc3_io_access(&mut self.semaphore, &mut self.semaphore);
                    }
                    0x0Eu8 => {
                        self.ram_control
                            .enable_huc3_io_access(&mut self.ir, &mut self.ir);
                    }
                    _ => {
                        self.ram_control.disable_ram_access();
                    }
                },
                0x2000u32 => {
                    rom_bank_new = data & 0x7F;
                }
                0x4000u32 => {
                    ram_bank_new = data & ram_bank_mask;
                }
                _ => {}
            }
            self.process_io();

            rom_bank_new &= rom_bank_mask;
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
                            .add(ram_bank as usize * 0x2000usize),
                    )
                }
            }
        }
    }
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
