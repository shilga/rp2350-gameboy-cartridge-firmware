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

use embassy_rp::{pac, peripherals, spi, spi::MosiPin, Peri};
use smart_leds::RGB8;

pub trait Ws2812Led {
    fn write(&mut self, color: &RGB8);
}

const TARGET_BAUDRATE: u32 = 2800000u32;

/// Driver for WS2812x RGB LED which uses an RP SPI interface MOSI pin
///
/// Each bit of the RGB color is encoded as 4 bit on the SPI bus.
/// As the there is a FIFO of 8 times 16 bit one RGB state can be transmitted instantly
#[allow(private_bounds)]
pub struct Ws2812Spi<'d, T: Instance> {
    inst: Peri<'d, T>,
    buffer: [u8; 16],
    //    phantom: PhantomData<(&'d mut T, M)>,
}

fn div_roundup(a: u32, b: u32) -> u32 {
    (a + b - 1) / b
}

fn calc_prescs(freq: u32) -> (u8, u8) {
    let clk_peri = crate::clocks::clk_peri_freq();

    // final SPI frequency: spi_freq = clk_peri / presc / postdiv
    // presc must be in 2..=254, and must be even
    // postdiv must be in 1..=256

    // divide extra by 2, so we get rid of the "presc must be even" requirement
    let ratio = div_roundup(clk_peri, freq * 2);
    if ratio > 127 * 256 {
        panic!("Requested too low SPI frequency");
    }

    let presc = div_roundup(ratio, 256);
    let postdiv = if presc == 1 {
        ratio
    } else {
        div_roundup(ratio, presc)
    };

    ((presc * 2) as u8, (postdiv - 1) as u8)
}

unsafe fn fill_buffer(color: u8, buffer_ptr: *mut u8) {
    let mut mask01 = 0x40u8;
    let mut mask10 = 0x80u8;
    let mut mask = 0xc0u8;

    let mut buffer_ptr = buffer_ptr;

    while mask != 0 {
        let masked_color = color & mask;

        if masked_color == mask {
            *buffer_ptr = 0xeeu8;
        } else if masked_color == mask01 {
            *buffer_ptr = 0x8eu8;
        } else if masked_color == mask10 {
            *buffer_ptr = 0xe8u8;
        } else {
            *buffer_ptr = 0x88u8;
        }

        mask01 >>= 2;
        mask10 >>= 2;
        mask >>= 2;

        buffer_ptr = buffer_ptr.offset(1);
    }
}

#[allow(private_bounds)]
impl<'d, T: Instance> Ws2812Spi<'d, T> {
    pub fn new(inst: Peri<'d, T>, mosi: Peri<'d, impl MosiPin<T>>) -> Self {
        let mosi_pin = mosi.pin() as usize;

        let mosi_pad_ctrl = pac::PADS_BANK0.gpio(mosi_pin);
        let mosi_gpio_ctrl = pac::IO_BANK0.gpio(mosi_pin).ctrl();

        mosi_gpio_ctrl.write(|w| w.set_funcsel(1));
        mosi_pad_ctrl.write(|w| {
            w.set_iso(false);
            w.set_schmitt(true);
            w.set_slewfast(false);
            w.set_ie(false);
            w.set_od(false);
            w.set_pue(false);
            w.set_pde(true);
        });

        let p = inst.regs();
        let (presc, postdiv) = calc_prescs(TARGET_BAUDRATE);

        p.cpsr().write(|w| w.set_cpsdvsr(presc));
        p.cr0().write(|w| {
            w.set_dss(0b1111); // 16bit
            w.set_scr(postdiv);
        });

        // finally, enable.
        p.cr1().write(|w| w.set_sse(true));

        Self {
            inst,
            buffer: [0u8; 16],
        }
    }
}

impl<'d, T: Instance> Ws2812Led for Ws2812Spi<'d, T> {
    fn write(&mut self, color: &RGB8) {
        unsafe {
            fill_buffer(color.g, self.buffer[0..4].as_mut_ptr());
            fill_buffer(color.r, self.buffer[4..8].as_mut_ptr());
            fill_buffer(color.b, self.buffer[8..12].as_mut_ptr());
        }

        critical_section::with(|_cs| {
            // This code runs within a critical section.

            for i in (0..self.buffer.len()).step_by(2) {
                self.inst.regs().dr().write_value(pac::spi::regs::Dr(
                    ((self.buffer[i] as u32) << 8) | self.buffer[i + 1] as u32,
                ));
            }
        });
    }
}

trait Instance: spi::Instance {
    fn regs(&self) -> pac::spi::Spi;
}

macro_rules! impl_instance {
    ($type:ident) => {
        impl Instance for peripherals::$type {
            fn regs(&self) -> pac::spi::Spi {
                pac::$type
            }
        }
    };
}

impl_instance!(SPI0);
impl_instance!(SPI1);
