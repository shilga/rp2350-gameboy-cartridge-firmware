use defmt::info;
use embassy_rp::pio::{Instance, StateMachineRx};
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
    ) -> GbBootloader<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, PIO, SM, Pin, PinError> {
        Self {
            volume_mgr,
            rx_fifo,
            gb_rom_memory,
            gb_ram_memory,
            reset_pin,
        }
    }

    pub async fn run(&mut self) {
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
            })
            .unwrap();

        bootloader_data[15] = num_roms;

        let _ = self.reset_pin.set_low();

        loop {
            let _addr = self.rx_fifo.wait_pull().await;
            let _data = (self.rx_fifo.wait_pull().await & 0xFFu32) as u8;
        }

        // let mut file = root_dir
        //     .open_file_in_dir("MARIO.GB", embedded_sdmmc::Mode::ReadOnly)
        //     .unwrap();
        // let read_start_time = Instant::now();
        // let num_read = file.read(gb_rom).unwrap();
        // let read_duration = read_start_time.elapsed();
        // info!(
        //     "Read {} bytes in {} ms",
        //     num_read,
        //     read_duration.as_millis()
        // );
    }
}
