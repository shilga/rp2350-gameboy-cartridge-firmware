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

use crc16::CCITT_FALSE;
use defmt::error;

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum HardwareRevision {
    Rev0 = 0x00u8,
    Rev1 = 0x01u8,
    Unknown = 0xFFu8,
}

impl HardwareRevision {
    fn from_u8(byte: u8) -> Self {
        match byte {
            0x00u8 => Self::Rev0,
            0x01u8 => Self::Rev1,
            _ => Self::Unknown,
        }
    }
}

#[derive(Debug)]
pub struct ProductionData {
    pub hardware_revision: HardwareRevision,
}

impl ProductionData {
    pub fn from_bytes(raw_data: &[u8; 16]) -> Option<Self> {
        if raw_data[0] != 0x01u8 && raw_data[1] != 0xFEu8 {
            return None;
        }

        let crc_read = u16::from_be_bytes(raw_data[14..16].try_into().unwrap_or_default());
        let crc_calc = crc16::State::<CCITT_FALSE>::calculate(&raw_data[..14]);

        if crc_read != crc_calc {
            error!(
                "production_data crc do not match! crc_read: {:X}, crc_calc: {:X}",
                crc_read, crc_calc
            );
            return None;
        }

        let hardware_revision = HardwareRevision::from_u8(raw_data[3]);

        Some(Self { hardware_revision })
    }
}

impl Default for ProductionData {
    fn default() -> Self {
        Self {
            hardware_revision: HardwareRevision::Rev0,
        }
    }
}
