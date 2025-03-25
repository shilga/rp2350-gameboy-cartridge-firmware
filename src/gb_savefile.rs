use defmt::debug;
use embassy_time::Instant;
use embedded_sdmmc::{BlockDevice, Directory, Error as SdmmcError, TimeSource};
use rtcc::DateTimeAccess;

use crate::rom_info::RomInfo;

pub trait GbRtcSaveStateProvider {
    fn retrieve_register_state(&self) -> ([u8; 5], [u8; 5]);
    fn restore_register_state(&mut self, regs: ([u8; 5], [u8; 5]));
    fn advance_by_seconds(&mut self, seconds: u64);
}

#[derive(Clone, Debug)]
pub enum GbSavefileError<E1, E2>
where
    E1: ::core::fmt::Debug,
    E2: ::core::fmt::Debug,
{
    Sdmmc(SdmmcError<E1>),
    Timesource(E2),
    SaveFileFromFuture,
}

pub struct GbSavefile<
    'a,
    'd,
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
    DTAError,
> where
    D: BlockDevice,
    T: TimeSource,
    DTAError: core::fmt::Debug,
{
    directory: &'a mut Directory<'d, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    rom_info: &'a RomInfo,
    timesource: &'a mut dyn DateTimeAccess<Error = DTAError>,
    rtc_state_provider: &'a mut dyn GbRtcSaveStateProvider,
}

impl<
        'a,
        'd,
        D,
        T,
        const MAX_DIRS: usize,
        const MAX_FILES: usize,
        const MAX_VOLUMES: usize,
        DTAError,
        E,
    > GbSavefile<'a, 'd, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES, DTAError>
where
    D: BlockDevice<Error = E>,
    T: TimeSource,
    E: ::core::fmt::Debug,
    DTAError: core::fmt::Debug,
{
    pub fn new(
        directory: &'a mut Directory<'d, D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
        rom_info: &'a RomInfo,
        timesource: &'a mut dyn DateTimeAccess<Error = DTAError>,
        rtc_state_provider: &'a mut dyn GbRtcSaveStateProvider,
    ) -> Self {
        Self {
            directory,
            rom_info,
            timesource,
            rtc_state_provider,
        }
    }

    pub fn load(&mut self, saveram_memory: &mut [u8]) -> Result<(), GbSavefileError<E, DTAError>> {
        let saveram_filename = self.rom_info.savefile.as_str();

        let mut savefile = self
            .directory
            .open_file_in_dir(saveram_filename, embedded_sdmmc::Mode::ReadOnly)
            .map_err(GbSavefileError::Sdmmc)?;

        debug!(
            "Found and opened savefile {} with len {}",
            saveram_filename,
            savefile.length()
        );

        if saveram_memory.len() > 0 {
            let len = savefile
                .read(saveram_memory)
                .map_err(GbSavefileError::Sdmmc)?;

            debug!("Read {} bytes for saveram", len);
        }

        if self.rom_info.has_rtc {
            let mut rtcsave = [0u8; 48];

            let len = savefile
                .read(&mut rtcsave)
                .map_err(GbSavefileError::Sdmmc)?;

            debug!("Read {} bytes for rtc", len);

            if len == rtcsave.len() {
                let mut real = [0u8; 5];
                let mut latch = [0u8; 5];

                real[0] = rtcsave[0];
                real[1] = rtcsave[4];
                real[2] = rtcsave[8];
                real[3] = rtcsave[12];
                real[4] = rtcsave[16];

                latch[0] = rtcsave[20];
                latch[1] = rtcsave[24];
                latch[2] = rtcsave[28];
                latch[3] = rtcsave[32];
                latch[4] = rtcsave[36];

                let timestamp_save = u64::from_le_bytes(rtcsave[40..48].try_into().unwrap());

                self.rtc_state_provider
                    .restore_register_state((real, latch));

                let time = self
                    .timesource
                    .datetime()
                    .map_err(GbSavefileError::Timesource)?;

                let seconds_since_boot = Instant::now().as_secs();
                // the RTC will advance for the time since boot, it needs to account for here
                let timestamp_now = time.and_utc().timestamp() as u64 - seconds_since_boot;

                if timestamp_now > timestamp_save {
                    let diff = timestamp_now - timestamp_save;

                    debug!("RTC should advance by {} seconds", diff);

                    self.rtc_state_provider.advance_by_seconds(diff);
                } else {
                    return Err(GbSavefileError::SaveFileFromFuture);
                }
            }
        }

        savefile.close().map_err(GbSavefileError::Sdmmc)?;

        Ok(())
    }

    pub fn store(&mut self, saveram_memory: &[u8]) -> Result<(), GbSavefileError<E, DTAError>> {
        let saveram_filename = self.rom_info.savefile.as_str();

        let mut savefile = self
            .directory
            .open_file_in_dir(
                saveram_filename,
                embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
            )
            .map_err(GbSavefileError::Sdmmc)?;

        debug!("Found and opened savefile {}", saveram_filename);

        savefile
            .write(saveram_memory)
            .map_err(GbSavefileError::Sdmmc)?;

        debug!("Wrote {} bytes for saveram", saveram_memory.len());

        if self.rom_info.has_rtc {
            let mut rtcsave = [0u8; 48];

            let time = self
                .timesource
                .datetime()
                .map_err(GbSavefileError::Timesource)?;
            let regstate = self.rtc_state_provider.retrieve_register_state();

            rtcsave[0..4].copy_from_slice(&(regstate.0[0] as u32).to_le_bytes());
            rtcsave[4..8].copy_from_slice(&(regstate.0[1] as u32).to_le_bytes());
            rtcsave[8..12].copy_from_slice(&(regstate.0[2] as u32).to_le_bytes());
            rtcsave[12..16].copy_from_slice(&(regstate.0[3] as u32).to_le_bytes());
            rtcsave[16..20].copy_from_slice(&(regstate.0[4] as u32).to_le_bytes());

            rtcsave[20..24].copy_from_slice(&(regstate.1[0] as u32).to_le_bytes());
            rtcsave[24..28].copy_from_slice(&(regstate.1[1] as u32).to_le_bytes());
            rtcsave[28..32].copy_from_slice(&(regstate.1[2] as u32).to_le_bytes());
            rtcsave[32..36].copy_from_slice(&(regstate.1[3] as u32).to_le_bytes());
            rtcsave[36..40].copy_from_slice(&(regstate.1[4] as u32).to_le_bytes());

            rtcsave[40..48].copy_from_slice(&(time.and_utc().timestamp() as u64).to_le_bytes());

            savefile.write(&rtcsave).map_err(GbSavefileError::Sdmmc)?;

            debug!("wrote RTC to savefile")
        }

        savefile.close().map_err(GbSavefileError::Sdmmc)?;

        Ok(())
    }
}
