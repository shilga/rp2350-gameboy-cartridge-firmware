use core::num;

use crate::hyperram::WriteBlocking as HyperRamWriteBlocking;
use crate::rom_info::RomInfo;
use arrayvec::ArrayString;
use cstr_core::{c_char, CStr};
use defmt::{info, warn};
use embassy_rp::pio::{Instance, StateMachineRx};
use embassy_time::Instant;
use embedded_hal_1::digital::OutputPin;
use embedded_sdmmc::{BlockDevice, TimeSource, VolumeManager};

pub struct GbBootloader<
    'a,
    'd,
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
    PIO,
    const SM: usize,
    Pin,
    PinError,
> where
    D: BlockDevice,
    T: TimeSource,
    PIO: Instance,
    Pin: OutputPin<Error = PinError>,
{
    volume_mgr: &'a mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    gb_rom_memory: &'a mut [u8],
    gb_ram_memory: &'a mut [u8],
    reset_pin: &'a mut Pin,
    hyperram: &'a mut dyn HyperRamWriteBlocking,
}

impl<
        'a,
        'd,
        D,
        T,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
        PIO,
        const SM: usize,
        Pin,
        PinError,
    > GbBootloader<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, PIO, SM, Pin, PinError>
where
    D: BlockDevice,
    T: TimeSource,
    PIO: Instance,
    Pin: OutputPin<Error = PinError>,
{
    pub fn new(
        volume_mgr: &'a mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        gb_rom_memory: &'a mut [u8],
        gb_ram_memory: &'a mut [u8],
        reset_pin: &'a mut Pin,
        hyperram: &'a mut dyn HyperRamWriteBlocking,
    ) -> GbBootloader<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, PIO, SM, Pin, PinError> {
        Self {
            volume_mgr,
            rx_fifo,
            gb_rom_memory,
            gb_ram_memory,
            reset_pin,
            hyperram,
        }
    }

    pub async fn run(&mut self) -> RomInfo {
        // load the bootloader binary
        let bytes = include_bytes!(concat!(env!("OUT_DIR"), "/bootloader.gb"));
        self.gb_rom_memory[..bytes.len()].copy_from_slice(bytes);

        // Try and access Volume 0 (i.e. the first partition).
        // The volume object holds information about the filesystem on that volume.
        let mut volume0 = self
            .volume_mgr
            .open_volume(embedded_sdmmc::VolumeIdx(0))
            .unwrap();
        info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

        let bootloader_data = &mut self.gb_ram_memory[0..0x1000];
        let mut used_data = 16usize; // offset that makes it compatible to the v1 cartridge bootloader
        let mut num_roms = 0u8;

        // Open the root directory (mutably borrows from the volume).
        let mut root_dir = volume0.open_root_dir().unwrap();
        root_dir
            .iterate_dir(|entry| {
                if entry.name.extension() == b"GB".as_ref()
                    || entry.name.extension() == b"GBC".as_ref()
                {
                    let rom_data = &mut bootloader_data[used_data..used_data + 18];
                    rom_data[0] = 0; // always without RTC for now
                    let filename_data = &mut rom_data[1..];
                    let base_name = entry.name.base_name();
                    let extension = entry.name.extension();
                    let filename_len = base_name.len() + extension.len() + 1;
                    filename_data[..base_name.len()].copy_from_slice(base_name);
                    filename_data[base_name.len()] = b'.';
                    filename_data[base_name.len() + 1..filename_len].copy_from_slice(extension);
                    filename_data[filename_len] = 0; // zero terminate

                    used_data += filename_len + 1 + 1; // + zero termination + status byte
                    num_roms += 1;

                    info!("filename_data: {}", filename_data);

                    info!(
                        "{} {} {}",
                        core::str::from_utf8(&filename_data[0..filename_len]).unwrap(),
                        entry.size,
                        if entry.attributes.is_directory() {
                            "<DIR>"
                        } else {
                            ""
                        }
                    );
                }
            })
            .unwrap();

        bootloader_data[15] = num_roms;

        let _ = self.reset_pin.set_low();

        let game_name_cstr;

        loop {
            let addr = self.rx_fifo.wait_pull().await;
            let data = (self.rx_fifo.wait_pull().await & 0xFFu32) as u8;

            info!("Addr {:#x} data {:#x}", addr, data);

            if addr == 0x6000u32 && data == 42 {
                let game_name_mem = &mut self.gb_ram_memory[0x1000..0x1011];
                info!("game_name_mem {}", game_name_mem);
                game_name_cstr = unsafe { CStr::from_ptr(game_name_mem.as_ptr() as *const c_char) };
                info!("Selected game: {}", game_name_cstr.to_str().unwrap());

                break;
            }
        }

        let _ = self.reset_pin.set_high();

        let ptr = self.gb_rom_memory.as_mut_ptr();
        let bank0 = unsafe { core::slice::from_raw_parts_mut(ptr, 0x4000usize) };
        let temp_bank0 =
            unsafe { core::slice::from_raw_parts_mut(ptr.add(0x4000usize), 0x4000usize) };
        let _temp_bank1 =
            unsafe { core::slice::from_raw_parts_mut(ptr.add(0x8000usize), 0x4000usize) };

        let game_filename = game_name_cstr.to_str().unwrap();
        let game_basename = game_filename.split(".").next().unwrap();
        let mut save_filename = ArrayString::<16>::new();
        save_filename.push_str(game_basename);
        save_filename.push_str(".SAV");

        let read_start_time = Instant::now();

        let mut file = root_dir
            .open_file_in_dir(game_filename, embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        let rom_length = file.length();
        let _ = file.read(bank0).unwrap();

        let rom_info = RomInfo::from_rom_bytes(bank0, save_filename.as_str()).unwrap();

        let mut addr = 0x4000u32;
        while addr < rom_length {
            let _ = file.read(temp_bank0).unwrap();
            self.hyperram.write_blocking(addr, temp_bank0);
            addr += 0x4000u32;
        }

        let read_duration = read_start_time.elapsed();
        info!(
            "Read {} bytes in {} ms",
            rom_length,
            read_duration.as_millis()
        );

        file.close().unwrap();

        match root_dir.open_file_in_dir(save_filename.as_str(), embedded_sdmmc::Mode::ReadOnly) {
            Ok(mut savefile) => {
                info!("Found and opened savefile");

                match savefile.read(self.gb_ram_memory) {
                    Ok(num_read) => {
                        info!("Read {} bytes from savefile", num_read);
                    }
                    Err(error) => {
                        warn!(
                            "Unable to read from savefile {}",
                            defmt::Debug2Format(&error)
                        );
                    }
                };

                savefile.close().unwrap();
            }
            Err(error) => {
                warn!("Unable to open savefile {}", defmt::Debug2Format(&error));
            }
        }

        rom_info
    }
}
