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

use arrayvec::ArrayString;
use core::ptr;
use cstr_core::{c_char, CStr};
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_rp::pio::{Instance, StateMachineRx};
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal_1::digital::{Error as HalDigitalError, OutputPin};
use embedded_sdmmc::{BlockDevice, TimeSource, VolumeManager};
use rtcc::{DateTimeAccess, Datelike, NaiveDate, Timelike};
use smart_leds::RGB8;

use crate::built_info;
use crate::gb_savefile::{GbRtcSaveStateProvider, GbSavefile};
use crate::hyperram::WriteBlocking as HyperRamWriteBlocking;
use crate::rom_info::RomInfo;
use crate::ws2812_spi::Ws2812Led;
use crate::ProductionData;

#[repr(C, packed(1))]
struct SharedGameboyData {
    git_sha1: u32,
    git_status: u8,
    build_type: c_char,
    version_major: u8,
    version_minor: u8,
    version_patch: u8,
    hardware_revision: u8,
    loaded_banks: u16,
    num_banks: u16,
    msg_id_gb_2_rp: u8,
    msg_id_rp_2_gb: u8,
    num_roms: u8,
}

#[repr(C, packed(1))]
struct TimePoint {
    second: u8,
    minute: u8,
    hour: u8,
    day: u8,
    month: u8,
    year: u8, // offset from 1970
}

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
    PinError,
    DTAError,
> where
    D: BlockDevice,
    T: TimeSource,
    PIO: Instance,
    PinError: HalDigitalError,
    DTAError: core::fmt::Debug,
{
    volume_mgr: &'a mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
    gb_rom_memory: &'a mut [u8],
    gb_ram_memory: &'a mut [u8],
    gb_rtc_state_provider: &'a mut dyn GbRtcSaveStateProvider,
    bank0_base_addr_ptr: *mut *mut u8,
    reset_pin: &'a mut dyn OutputPin<Error = PinError>,
    hyperram: &'a mut dyn HyperRamWriteBlocking,
    led: &'a mut dyn Ws2812Led,
    rtc: &'a mut dyn DateTimeAccess<Error = DTAError>,
    production_data: &'a ProductionData,
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
        PinError,
        DTAError,
    > GbBootloader<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, PIO, SM, PinError, DTAError>
where
    D: BlockDevice,
    T: TimeSource,
    PIO: Instance,
    PinError: HalDigitalError,
    DTAError: core::fmt::Debug,
{
    pub fn new(
        volume_mgr: &'a mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
        rx_fifo: &'a mut StateMachineRx<'d, PIO, SM>,
        gb_rom_memory: &'a mut [u8],
        gb_ram_memory: &'a mut [u8],
        gb_rtc_state_provider: &'a mut dyn GbRtcSaveStateProvider,
        bank0_base_addr_ptr: *mut *mut u8,
        reset_pin: &'a mut dyn OutputPin<Error = PinError>,
        hyperram: &'a mut dyn HyperRamWriteBlocking,
        led: &'a mut dyn Ws2812Led,
        rtc: &'a mut dyn DateTimeAccess<Error = DTAError>,
        production_data: &'a ProductionData,
    ) -> GbBootloader<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, PIO, SM, PinError, DTAError>
    {
        Self {
            volume_mgr,
            rx_fifo,
            gb_rom_memory,
            gb_ram_memory,
            gb_rtc_state_provider,
            bank0_base_addr_ptr,
            reset_pin,
            hyperram,
            led,
            rtc,
            production_data,
        }
    }

    pub async fn run(&mut self) -> RomInfo {
        // load the bootloader binary
        let bytes = include_bytes!(concat!(env!("OUT_DIR"), "/bootloader.gb"));
        self.gb_rom_memory[0x4000usize..bytes.len() + 0x4000usize].copy_from_slice(bytes);

        // set the bank0 pointer to the memory we put the bootloader in
        unsafe {
            ptr::write_volatile(
                self.bank0_base_addr_ptr,
                self.gb_rom_memory.as_mut_ptr().offset(0x4000isize),
            )
        };

        // put in some version info
        let bootloader_data = &mut self.gb_ram_memory[0..0x1000];
        let shared_data: &mut SharedGameboyData = unsafe {
            (bootloader_data.as_mut_ptr() as *mut SharedGameboyData)
                .as_mut()
                .unwrap()
        };

        bootloader_data.fill(0u8);

        shared_data.msg_id_rp_2_gb = 0;
        let git_short_hash =
            u32::from_str_radix(built_info::GIT_COMMIT_HASH_SHORT.unwrap_or("0"), 16).unwrap_or(0);
        shared_data.git_sha1 = git_short_hash;
        shared_data.git_status = if built_info::GIT_DIRTY.unwrap_or_default() {
            0x1u8
        } else {
            0x0u8
        };
        shared_data.version_major = u8::from_str_radix(env!("VERSION_MAJOR"), 10).unwrap();
        shared_data.version_minor = u8::from_str_radix(env!("VERSION_MINOR"), 10).unwrap();
        shared_data.version_patch = u8::from_str_radix(env!("VERSION_PATCH"), 10).unwrap();
        shared_data.build_type = env!("RELEASE_TYPE").chars().nth(0).unwrap() as c_char;
        shared_data.hardware_revision = self.production_data.hardware_revision as u8;

        let mut used_data = 17usize; // offset that makes it compatible to the v1 cartridge bootloader
        let mut num_roms = 0u8;

        // Try and access Volume 0 (i.e. the first partition).
        // The volume object holds information about the filesystem on that volume.
        let mut volume0 = self.volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0));
        match volume0 {
            Ok(ref mut volume0) => {
                info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

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
                            filename_data[base_name.len() + 1..filename_len]
                                .copy_from_slice(extension);
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
            }
            Err(ref error) => {
                warn!("unable to open volume0 {}", defmt::Debug2Format(error));
            }
        };

        shared_data.num_roms = num_roms;

        let _ = self.reset_pin.set_low();

        let game_name_cstr;

        let mut ticker = Ticker::every(Duration::from_millis(500));
        let colors = [
            RGB8::default(),
            RGB8::new(0, 0, 16),
            RGB8::new(16, 0, 0),
            RGB8::new(0, 16, 0),
        ];
        let mut current_color = 1;

        loop {
            match select(ticker.next(), self.rx_fifo.wait_pull()).await {
                Either::First(_) => {
                    self.led.write(&colors[current_color]);
                    current_color += 1;
                    if current_color == colors.len() {
                        current_color = 0;
                    }
                }
                Either::Second(addr) => {
                    let data = (self.rx_fifo.wait_pull().await & 0xFFu32) as u8;

                    info!("Addr {:#x} data {:#x}", addr, data);

                    if addr == 0x6000u32 {
                        match data {
                            42 => {
                                let game_name_mem = &mut self.gb_ram_memory[0x1000..0x1011];
                                info!("game_name_mem {}", game_name_mem);
                                game_name_cstr = unsafe {
                                    CStr::from_ptr(game_name_mem.as_ptr() as *const c_char)
                                };
                                info!("Selected game: {}", game_name_cstr.to_str().unwrap());

                                break;
                            }
                            0x0c => {
                                info!("RTC Read command");
                                let dt = self.rtc.datetime().unwrap_or_default();
                                let tp: &mut TimePoint = unsafe {
                                    (self.gb_ram_memory[0x1000..].as_mut_ptr() as *mut TimePoint)
                                        .as_mut()
                                        .unwrap()
                                };
                                tp.second = dt.second() as u8;
                                tp.minute = dt.minute() as u8;
                                tp.hour = dt.hour() as u8;
                                tp.day = dt.day0() as u8;
                                tp.month = dt.month0() as u8;
                                tp.year = (dt.year() - 1970) as u8;

                                shared_data.msg_id_rp_2_gb = shared_data.msg_id_gb_2_rp;
                            }
                            0x0d => {
                                info!("RTC Write command");
                                let tp: &TimePoint = unsafe {
                                    (self.gb_ram_memory[0x1000..].as_mut_ptr() as *mut TimePoint)
                                        .as_ref()
                                        .unwrap()
                                };
                                match match NaiveDate::from_ymd_opt(
                                    tp.year as i32 + 1970,
                                    tp.month as u32 + 1,
                                    tp.day as u32 + 1,
                                ) {
                                    None => None,
                                    Some(nd) => nd.and_hms_opt(tp.hour as u32, tp.minute as u32, 0),
                                } {
                                    None => warn!("Could not parse received TimePoint"),
                                    Some(dt) => {
                                        info!("dt: {}", defmt::Debug2Format(&dt));
                                        self.rtc.set_datetime(&dt).unwrap();
                                    }
                                };

                                shared_data.msg_id_rp_2_gb = shared_data.msg_id_gb_2_rp;
                            }
                            _ => {
                                warn!("Unknown command received");
                            }
                        }
                    }
                }
            }
        }

        let ptr = self.gb_rom_memory.as_mut_ptr();
        let bank0 = unsafe { core::slice::from_raw_parts_mut(ptr, 0x4000usize) };
        let _temp_bank0 =
            unsafe { core::slice::from_raw_parts_mut(ptr.add(0x4000usize), 0x4000usize) };
        let temp_bank1 =
            unsafe { core::slice::from_raw_parts_mut(ptr.add(0x8000usize), 0x4000usize) };

        let game_filename = game_name_cstr.to_str().unwrap();
        let game_basename = game_filename.split(".").next().unwrap();
        let mut save_filename = ArrayString::<16>::new();
        save_filename.push_str(game_basename);
        save_filename.push_str(".SAV");

        let read_start_time = Instant::now();

        let mut volume0 = volume0.unwrap();
        let mut root_dir = volume0.open_root_dir().unwrap();

        let mut file = root_dir
            .open_file_in_dir(game_filename, embedded_sdmmc::Mode::ReadOnly)
            .unwrap();
        let rom_length = file.length();
        let _ = file.read(bank0).unwrap();

        let rom_info = RomInfo::from_rom_bytes(bank0, save_filename.as_str()).unwrap();

        shared_data.num_banks = rom_info.rom_bank_count;

        let mut addr = 0x4000u32;
        shared_data.loaded_banks = 1u16;
        while addr < rom_length {
            let _ = file.read(temp_bank1).unwrap();
            self.hyperram.write_blocking(addr, temp_bank1);
            addr += 0x4000u32;
            shared_data.loaded_banks += 1;
        }

        let read_duration = read_start_time.elapsed();
        info!(
            "Read {} bytes in {} ms",
            rom_length,
            read_duration.as_millis()
        );

        file.close().unwrap();

        let _ = self.reset_pin.set_high(); // reset the GameBoy

        // set the bank0 pointer to the area we have loaded bank0 to
        unsafe { ptr::write_volatile(self.bank0_base_addr_ptr, bank0.as_mut_ptr()) };

        {
            let saveram_memory =
                &mut self.gb_ram_memory[0..(rom_info.ram_bank_count as usize * 0x2000usize)];

            let mut savefile = GbSavefile::new(
                &mut root_dir,
                &rom_info,
                self.rtc,
                self.gb_rtc_state_provider,
            );

            match savefile.load(saveram_memory) {
                Ok(_) => {
                    info!("saveram restored from savefile");
                }
                Err(error) => {
                    warn!(
                        "Unable to read from savefile {}",
                        defmt::Debug2Format(&error)
                    );
                }
            }
        }

        rom_info
    }
}
