use defmt::debug;
use embedded_sdmmc::{BlockDevice, Directory, Error as SdmmcError, TimeSource};
use rtcc::DateTimeAccess;

use crate::rom_info::RomInfo;

pub trait GbRtcSaveStateProvider {
    fn retrieve_register_state(&self) -> ([u8; 5], [u8; 5]);
}

#[derive(Clone, Debug)]
pub enum GbSavefileError<E1, E2>
where
    E1: ::core::fmt::Debug,
    E2: ::core::fmt::Debug,
{
    Sdmmc(SdmmcError<E1>),
    Timesource(E2),
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
    rom_info: &'static RomInfo,
    saveram_memory: &'static [u8],
    timesource: &'a mut dyn DateTimeAccess<Error = DTAError>,
    rtc_state_provider: &'a dyn GbRtcSaveStateProvider,
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
        rom_info: &'static RomInfo,
        saveram_memory: &'static [u8],
        timesource: &'a mut dyn DateTimeAccess<Error = DTAError>,
        rtc_state_provider: &'a dyn GbRtcSaveStateProvider,
    ) -> Self {
        Self {
            directory,
            rom_info,
            saveram_memory,
            timesource,
            rtc_state_provider,
        }
    }

    pub fn store(&mut self) -> Result<(), GbSavefileError<E, DTAError>> {
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
            .write(self.saveram_memory)
            .map_err(GbSavefileError::Sdmmc)?;

        debug!("Wrote {} bytes for saveram", self.saveram_memory.len());

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
