use embedded_hal_1::spi::{Operation, SpiDevice};

pub struct Mcp795xx<SPI> {
    spi: SPI,
}

#[derive(Copy, Clone, Debug)]
pub enum Mcp795xxError<SPI> {
    Spi(SPI),
    // Add other errors for your driver here.
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

impl<SPI> Mcp795xx<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn read_register(&mut self, addr: u8) -> Result<u8, Mcp795xxError<SPI::Error>> {
        let mut read_buf = [0u8; 1];

        self.spi
            .transaction(&mut [
                Operation::Write(&[Instructions::READ as u8, addr]),
                Operation::Read(&mut read_buf),
            ])
            .map_err(Mcp795xxError::Spi)?;

        Ok(read_buf[0])
    }

    pub fn write_register(
        &mut self,
        addr: u8,
        reg_data: u8,
    ) -> Result<(), Mcp795xxError<SPI::Error>> {
        self.spi
            .write(&[Instructions::WRITE as u8, addr, reg_data])
            .map_err(Mcp795xxError::Spi)?;

        Ok(())
    }
}
