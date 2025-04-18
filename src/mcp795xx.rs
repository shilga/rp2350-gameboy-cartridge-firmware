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

use embedded_hal_1::spi::{Operation, SpiDevice};
use {defmt_serial as _, panic_probe as _};

pub use rtcc::{DateTimeAccess, Datelike, NaiveDate, NaiveDateTime, Rtcc, Timelike};

pub struct Mcp795xx<SPI> {
    spi: SPI,
}

#[derive(Copy, Clone, Debug)]
pub enum Mcp795xxError<E> {
    Spi(E),
    InvalidInputData,
    CalendarError,
}

#[allow(unused)]
enum Instructions {
    /// Read data from EEPROM array beginning at selected address
    EEREAD = 0b0000_0011,
    /// Write data to EEPROM array beginning at selected address
    EEWRITE = 0b0000_0010,
    /// Reset the write enable latch (disable write operations)
    EEWRDI = 0b0000_0100,
    /// Set the write enable latch (enable write operations)
    EEWREN = 0b0000_0110,
    /// Read STATUS register
    SRREAD = 0b0000_0101,
    /// Write STATUS register
    SRWRITE = 0b0000_0001,
    /// Read data from RTCC/SRAM array beginning at selected address
    READ = 0b0001_0011,
    /// Write data to RTCC/SRAM array beginning at selected address
    WRITE = 0b0001_0010,
    /// Unlock the protected EEPROM block for a write operation
    UNLOCK = 0b0001_0100,
    /// Write data to the protected EEPROM block beginning at selected address
    IDWRITE = 0b0011_0010,
    /// Read data from the protected EEPROM block beginning at the selected address
    IDREAD = 0b0011_0011,
    /// Clear all SRAM data to 0
    CLRRAM = 0b0101_0100,
}

#[allow(unused)]
enum RegisterAdresses {
    RTCHSEC = 0x00,
    RTCSEC = 0x01,
    RTCMIN = 0x02,
    RTCHOUR = 0x03,
    RTCWKDAY = 0x04,
    RTCDATE = 0x05,
    RTCMTH = 0x06,
    RTCYEAR = 0x07,
    CONTROL = 0x08,
    ALM0SEC = 0x0C,
    ALM0MIN = 0x0D,
    ALM0HOUR = 0x0E,
    ALM0WKDAY = 0x0F,
    ALM0DATE = 0x10,
    ALM0MTH = 0x11,
    ALM1HSEC = 0x12,
    ALM1SEC = 0x13,
    ALM1MIN = 0x14,
    ALM1HOUR = 0x15,
    ALM1WKDAY = 0x16,
    ALM1DATE = 0x17,
    PWRDNMIN = 0x18,
    PWRDNHOUR = 0x19,
    PWRDNDATE = 0x1A,
    PWRDNMTH = 0x1B,
    PWRUPMIN = 0x1C,
    PWRUPHOUR = 0x1D,
    PWRUPDATE = 0x1E,
    PWRUPMTH = 0x1F,
}

fn bin2bcd(dec: u8) -> u8 {
    ((dec / 10) << 4) | (dec % 10)
}

pub(crate) fn bcd2bin(bcd: u8) -> u8 {
    (bcd >> 4) * 10 + (bcd & 0xF)
}

impl<SPI, E> Mcp795xx<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn read_register(&mut self, addr: u8) -> Result<u8, Mcp795xxError<E>> {
        let mut read_buf = [0u8; 1];

        self.spi
            .transaction(&mut [
                Operation::Write(&[Instructions::READ as u8, addr]),
                Operation::Read(&mut read_buf),
            ])
            .map_err(Mcp795xxError::Spi)?;

        Ok(read_buf[0])
    }

    pub fn read_registers(&mut self, addr: u8, payload: &mut [u8]) -> Result<(), Mcp795xxError<E>> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[Instructions::READ as u8, addr]),
                Operation::Read(payload),
            ])
            .map_err(Mcp795xxError::Spi)?;

        Ok(())
    }

    pub fn write_register(&mut self, addr: u8, reg_data: u8) -> Result<(), Mcp795xxError<E>> {
        self.spi
            .write(&[Instructions::WRITE as u8, addr, reg_data])
            .map_err(Mcp795xxError::Spi)?;

        Ok(())
    }

    pub fn write_registers(&mut self, addr: u8, payload: &[u8]) -> Result<(), Mcp795xxError<E>> {
        self.spi
            .transaction(&mut [
                Operation::Write(&[Instructions::WRITE as u8, addr]),
                Operation::Write(payload),
            ])
            .map_err(Mcp795xxError::Spi)?;

        Ok(())
    }

    pub fn is_oscillator_running(&mut self) -> Result<bool, Mcp795xxError<E>> {
        let data = self.read_register(RegisterAdresses::RTCWKDAY as u8)?;
        Ok((data & 0x20u8) != 0)
    }

    pub fn enable_oscillator(&mut self) -> Result<(), Mcp795xxError<E>> {
        let sec = self.read_register(RegisterAdresses::RTCSEC as u8)?;
        self.write_register(RegisterAdresses::RTCSEC as u8, sec | 0x80u8)?;
        Ok(())
    }

    pub fn disable_oscillator(&mut self) -> Result<(), Mcp795xxError<E>> {
        let sec = self.read_register(RegisterAdresses::RTCSEC as u8)?;
        self.write_register(RegisterAdresses::RTCSEC as u8, sec & 0x80u8)?;
        Ok(())
    }
}

impl<SPI, E> DateTimeAccess for Mcp795xx<SPI>
where
    SPI: SpiDevice<Error = E>,
{
    type Error = Mcp795xxError<E>;

    fn datetime(&mut self) -> Result<NaiveDateTime, Self::Error> {
        let mut data = [0; 7];
        self.read_registers(RegisterAdresses::RTCSEC as u8, &mut data)?;

        Ok(NaiveDate::from_ymd_opt(
            bcd2bin(data[6]) as i32 + 2000,
            bcd2bin(data[5] & 0x1F) as u32,
            bcd2bin(data[4] & 0x3F) as u32,
        )
        .ok_or(Self::Error::CalendarError)?
        .and_hms_opt(
            bcd2bin(data[2] & 0x3F) as u32,
            bcd2bin(data[1] & 0x7F) as u32,
            bcd2bin(data[0] & 0x7F) as u32,
        )
        .ok_or(Self::Error::CalendarError)?)
    }

    fn set_datetime(&mut self, datetime: &NaiveDateTime) -> Result<(), Self::Error> {
        let mut data = [0; 7];

        if datetime.year() < 2000 || datetime.year() > 2099 {
            return Err(Self::Error::InvalidInputData);
        }

        // read register first to preserve config bits
        self.read_registers(RegisterAdresses::RTCSEC as u8, &mut data)?;

        let is_oscillator_running = data[3] & 20u8 != 0;

        if is_oscillator_running {
            self.disable_oscillator()?;
        }

        data[0] = bin2bcd(datetime.second() as u8); // no need to preserve here, osc always off
        data[1] = (data[1] & 0x80u8) | bin2bcd(datetime.minute() as u8);
        data[2] = bin2bcd(datetime.hour() as u8); // this also sets 24 hour format
        data[3] = (data[3] & 0x08u8) | datetime.weekday().number_from_monday() as u8;
        data[4] = bin2bcd(datetime.day() as u8);
        data[5] = bin2bcd(datetime.month() as u8);
        data[6] = bin2bcd((datetime.year() - 2000) as u8);

        /* Always write the date and month using a separate Write command.
         * This is a workaround for a know silicon issue that some combinations
         * of date and month values may result in the date being reset to 1.
         */
        self.write_registers(RegisterAdresses::RTCSEC as u8, &data[0..5])?;
        self.write_registers(RegisterAdresses::RTCMTH as u8, &data[5..])?;

        if is_oscillator_running {
            // if it was running before enable it again
            self.enable_oscillator()?;
        }

        Ok(())
    }
}
