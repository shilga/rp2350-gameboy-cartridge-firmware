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

#![allow(unused)]

use arrayvec::ArrayString;
use core::str;
use defmt::error;

#[derive(Debug)]
pub enum MbcType {
    None,
    Mbc1,
    Mbc2,
    Mbc3,
    Mbc5,
}

pub struct RomInfo {
    pub ram_bank_count: u8,
    pub rom_bank_count: u16,
    pub has_rtc: bool,
    pub mbc: MbcType,
    pub savefile: ArrayString<16>,
}

impl RomInfo {
    pub fn from_rom_bytes(first_bank: &[u8], savefile_str: &str) -> Option<Self> {
        let mut has_rtc = false;
        let mbc_dat = first_bank[0x147];

        let mbc = match mbc_dat {
            0x00u8 => MbcType::None,
            0x01u8..=0x03u8 => MbcType::Mbc1,
            0x05u8..=0x07u8 => MbcType::Mbc2,
            0x0Fu8..=0x10u8 => {
                has_rtc = true;

                MbcType::Mbc3
            }
            0x11u8..=0x13u8 => MbcType::Mbc3,
            0x19u8..=0x1Eu8 => MbcType::Mbc5,
            _ => {
                error!("Don't know how to interpret MBC type {:#x}", mbc_dat);
                return None;
            }
        };

        let lookup = [0u8, 0u8, 1u8, 4u8, 16u8, 8u8];
        let ram_bank_count_value = first_bank[0x149];
        let ram_bank_count = lookup[ram_bank_count_value as usize];

        let rom_bank_count = 1u16 << (first_bank[0x0148] as u16 + 1);

        let mut savefile = ArrayString::<16>::new();
        savefile.push_str(savefile_str);

        Some(Self {
            ram_bank_count,
            rom_bank_count,
            has_rtc,
            mbc,
            savefile,
        })
    }
}
