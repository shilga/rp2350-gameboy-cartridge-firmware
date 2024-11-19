//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! No specific hardware is specified in this example. Only output on pin 0 is tested.
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use embassy_executor::{Executor, Spawner};
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::otp;
use embassy_rp::peripherals::{PIN_46, SPI1, USB};
use embassy_rp::peripherals::{PIO0, PIO1, PIO2, SPI0, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::spi::Blocking;
use embassy_rp::uart::UartTx;
use embassy_rp::uart::{self};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, spi};
use embassy_rp::{clocks, config as rpconfig, pac};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::CriticalSectionMutex;

use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;

use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Config, UsbDevice};

use embassy_time::Timer;

use embedded_io::{ErrorType, Write};

use embedded_sdmmc::sdcard::SdCard;
use embedded_sdmmc::VolumeManager;

use gb_pio::GbRamRead;
use rom_info::{MbcType, RomInfo};
use rp2350_core_voltage::vreg_set_voltage;
use smart_leds::RGB8;

use core::cell::RefCell;
use core::{ptr, str};

use arrayvec::ArrayString;

use defmt::{info, unwrap, warn};
use {defmt_serial as _, panic_probe as _};

use static_cell::StaticCell;

mod ws2812_spi;
use crate::ws2812_spi::{Ws2812Led, Ws2812Spi};

mod picotool_reset;
use crate::picotool_reset::PicotoolReset;

mod gb_bootloader;
use crate::gb_bootloader::GbBootloader;

mod gb_pio;
use gb_pio::{
    GbDataOut, GbMbcCommands, GbPioPins, GbRamWrite, GbRomDetect, GbRomHigher, GbRomLower,
};

mod gb_dma;
use gb_dma::{GbReadDmaConfig, GbReadSniffDmaConfig, GbWriteDmaConfig};

mod gb_mbc;
use gb_mbc::{Mbc, Mbc1, Mbc3, Mbc5, NoMbc};

mod hyperram;
use hyperram::{HyperRam, HyperRamPins, HyperRamReadOnly};

mod dma_helper;

mod rom_info;

mod rp2350_core_voltage;

mod mcp975xx;
use mcp975xx::Mcp795xx;

// Include the generated-file as a separate module
pub mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 3] = [
    embassy_rp::binary_info::rp_program_name!(c"rp2350-gameboy-cartridge"),
    embassy_rp::binary_info::rp_program_description!(c"A GameBoy cartridge based on the RP2350"),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{F5E9F718-9228-4B8F-BE38-E53DE6FDCE2B}"];

static SERIAL: StaticCell<SerialWrapper> = StaticCell::new();

struct SerialWrapper<'a> {
    uart: UartTx<'a, UART0, embassy_rp::uart::Blocking>,
}

impl<'a> ErrorType for SerialWrapper<'a> {
    type Error = uart::Error;
}

impl<'a> Write for SerialWrapper<'a> {
    fn write(&mut self, word: &[u8]) -> Result<usize, uart::Error> {
        self.uart.blocking_write(word).unwrap();
        Ok(word.len())
    }

    fn flush(&mut self) -> Result<(), uart::Error> {
        self.uart.blocking_flush()
    }
}

struct DummyTimesource();
impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

type VolumeManagerType<'d> = VolumeManager<
    SdCard<
        SpiDeviceWithConfig<'d, CriticalSectionRawMutex, spi::Spi<'d, SPI0, Blocking>, Output<'d>>,
        embassy_time::Delay,
    >,
    DummyTimesource,
>;
static mut VOLUME_MANAGER: StaticCell<VolumeManagerType> = StaticCell::new();

static LOADED_ROM_INFO: StaticCell<RomInfo> = StaticCell::new();

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

static SPI_BUS: StaticCell<CriticalSectionMutex<RefCell<spi::Spi<SPI0, Blocking>>>> =
    StaticCell::new();

static WS2812: StaticCell<Ws2812Spi<SPI1>> = StaticCell::new();

extern "C" {
    static mut _s_gb_rom_memory: u8;
    static mut _s_gb_save_ram: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
    let mut rp_config = rpconfig::Config::default();
    let pll_config = rp_config
        .clocks
        .xosc
        .as_mut()
        .unwrap()
        .sys_pll
        .as_mut()
        .unwrap();

    vreg_set_voltage(rp2350_core_voltage::VregVoltage::VregVoltage1_15);

    // 150 MHz config
    // pll_config.fbdiv = 125; // VCO 1500 MHz
    // pll_config.post_div1 = 5;
    // pll_config.post_div2 = 2;

    // 200 MHz config
    // pll_config.fbdiv = 100; // VCO 1200 MHz
    // pll_config.post_div1 = 6;
    // pll_config.post_div2 = 1;

    // 266 MHz config
    pll_config.fbdiv = 133; // VCO 1596 MHz
    pll_config.post_div1 = 6;
    pll_config.post_div2 = 1;

    let p = embassy_rp::init(rp_config);
    let config = uart::Config::default();
    let uart = uart::UartTx::new_blocking(p.UART0, p.PIN_46, config);

    let serialwrapper = SerialWrapper { uart };

    defmt_serial::defmt_serial(SERIAL.init(serialwrapper));

    let sys_freq = clocks::clk_sys_freq();
    info!(
        "Hello defmt-world!, croco-cartridge v2 revision {}",
        built_info::GIT_COMMIT_HASH_SHORT
    );
    info!("running at {} hz", sys_freq);

    let mut reset_pin = Output::new(p.PIN_45, Level::High);
    let mut _gb_bus_en = Output::new(p.PIN_44, Level::High);

    let Pio {
        common: mut pio0,
        sm0: _sm0_0,
        sm1: sm0_1,
        sm2: sm0_2,
        sm3: sm0_3,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let Pio {
        common: mut pio1,
        sm0: sm1_0,
        sm1: sm1_1,
        sm2: sm1_2,
        sm3: sm1_3,
        ..
    } = Pio::new(p.PIO1, Irqs);

    let Pio {
        common: mut pio2,
        sm0: mut sm2_0,
        ..
    } = Pio::new(p.PIO2, Irqs);

    let ws2812 = WS2812.init(Ws2812Spi::new(p.SPI1, p.PIN_47));

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    let serialnum = otp::get_chipid().unwrap();
    let serial = {
        static SERIALBUF: StaticCell<ArrayString<16>> = StaticCell::new();
        SERIALBUF.init(ArrayString::<16>::new())
    };
    core::fmt::write(serial, format_args!("{:X}", serialnum)).unwrap();

    // Create embassy-usb Config
    // Create a USB device RPI Vendor ID and on of these Product ID:
    // https://github.com/raspberrypi/picotool/blob/master/picoboot_connection/picoboot_connection.c#L23-L27
    let mut config = Config::new(0x2e8a, 0x0009);
    config.manufacturer = Some("Croco");
    config.product = Some("Cartridge V2");
    config.serial_number = Some(serial.as_str());
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // // Required for windows compatibility.
    // // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let picotool = {
        static PICOTOOL: StaticCell<PicotoolReset> = StaticCell::new();
        let picotool = PicotoolReset::new();
        PICOTOOL.init(picotool)
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            MSOS_DESCRIPTOR.init([0; 256]), // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // which causes it to use the built-in WinUSB driver automatically, which in turn
    // can be used by libusb/rusb software without needing a custom driver or INF file.
    // In principle you might want to call msos_feature() just on a specific function,
    // if your device also has other functions that still use standard class drivers.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    picotool.configure(&mut builder);

    // Build the builder.
    let usb = builder.build();

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(usb_task(usb)).unwrap();

    #[rustfmt::skip]
    let gb_pio_pins = GbPioPins::new(
        &mut pio1,
        p.PIN_17, p.PIN_18, p.PIN_19,
        p.PIN_20, p.PIN_21, p.PIN_22, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_26, p.PIN_27,
        p.PIN_28, p.PIN_29, p.PIN_30, p.PIN_31, p.PIN_32, p.PIN_33, p.PIN_34, p.PIN_35,
        p.PIN_36, p.PIN_37, p.PIN_38, p.PIN_39, p.PIN_40, p.PIN_41, p.PIN_42, p.PIN_43,
        None::<PIN_46>
    );
    let mut gb_rom_detect_pio = GbRomDetect::new(&mut pio1, &pac::PIO1, sm1_2);
    let mut gb_rom_lower_pio = GbRomLower::new(&mut pio1, &pac::PIO1, sm1_0, &gb_pio_pins);
    let mut gb_rom_higher_pio = GbRomHigher::new(&mut pio1, &pac::PIO1, sm1_1, &gb_pio_pins);
    let mut gb_data_out_pio = GbDataOut::new(&mut pio1, &pac::PIO1, sm1_3, &gb_pio_pins);

    let mut gb_mbc_commands_pio = GbMbcCommands::new(&mut pio0, &pac::PIO0, sm0_1);
    let mut gb_ram_read_pio = GbRamRead::new(&mut pio0, &pac::PIO0, sm0_2);
    let mut gb_ram_write_pio = GbRamWrite::new(&mut pio0, &pac::PIO0, sm0_3);

    info!("gpiobase: {}", pac::PIO1.gpiobase().read().gpiobase());

    let gb_rom = unsafe {
        core::slice::from_raw_parts_mut(ptr::addr_of!(_s_gb_rom_memory) as *mut u8, 0x10000)
    };

    let gb_save_ram = unsafe {
        core::slice::from_raw_parts_mut(ptr::addr_of!(_s_gb_save_ram) as *mut u8, 0x20000)
    };

    let mut gb_rom_ptr = unsafe { ptr::addr_of_mut!(_s_gb_rom_memory) };
    let mut gb_ram_ptr = unsafe { ptr::addr_of_mut!(_s_gb_save_ram) };
    let mut current_higher_base_addr: u32 = 0x4000u32;

    let _read_dma_lower = GbReadDmaConfig::new(
        p.DMA_CH0,
        p.DMA_CH1,
        p.DMA_CH2,
        ptr::addr_of_mut!(gb_rom_ptr),
        &gb_rom_lower_pio,
        &gb_data_out_pio,
    );

    let _read_dma_saveram = GbReadDmaConfig::new(
        p.DMA_CH3,
        p.DMA_CH4,
        p.DMA_CH5,
        ptr::addr_of_mut!(gb_ram_ptr),
        &gb_ram_read_pio,
        &gb_data_out_pio,
    );

    let _write_dma_saveram = GbWriteDmaConfig::new(
        p.DMA_CH6,
        p.DMA_CH7,
        p.DMA_CH8,
        ptr::addr_of_mut!(gb_ram_ptr),
        &gb_ram_write_pio,
    );

    gb_rom_lower_pio.start();
    gb_data_out_pio.start();
    gb_mbc_commands_pio.start();
    gb_rom_detect_pio.start();
    gb_ram_read_pio.start();
    gb_ram_write_pio.start();

    // SPI clock needs to be running at <= 400kHz during initialization of sd card
    let mut config = spi::Config::default();
    config.frequency = 400_000;
    let spi = spi::Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, config);

    let spi_bus = CriticalSectionMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);

    let cs_sd_pin = Output::new(p.PIN_1, Level::High);
    let cs_rtc_pin = Output::new(p.PIN_5, Level::High);

    // pre-initialize with 74 clock cycles according to SD-card spec
    spi_bus.lock(|bus| {
        bus.borrow_mut().blocking_write(&[0xFF; 10]).unwrap();
    });

    let mut config = spi::Config::default();
    config.frequency = 400_000;
    let spi_dev_sd = SpiDeviceWithConfig::new(spi_bus, cs_sd_pin, config);

    let mut config = spi::Config::default();
    config.frequency = 1_000_000;
    let spi_dev_rtc = SpiDeviceWithConfig::new(spi_bus, cs_rtc_pin, config);

    // open the sd card and fully initialize it
    let sdcard = SdCard::new(spi_dev_sd, embassy_time::Delay);
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

    // Now that the card is initialized, the SPI clock can go faster
    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
    sdcard.spi(|dev| dev.set_config(config));

    // Now let's look for volumes (also known as partitions) on our block device.
    // To do this we need a Volume Manager. It will take ownership of the block device.
    let volume_mgr = unsafe {
        VOLUME_MANAGER.init(embedded_sdmmc::VolumeManager::new(
            sdcard,
            DummyTimesource(),
        ))
    };

    let mut rtc = Mcp795xx::new(spi_dev_rtc);

    let control = rtc.read_register(8).unwrap();
    info!("control {:#x}", control);
    if control != 0x40 {
        info!("writing control");
        rtc.write_register(8, 0x40).unwrap();
    }

    let weekd_status = rtc.read_register(4).unwrap();
    info!("weekd_status {:#x}", weekd_status);
    if weekd_status & 0x08 == 0 {
        info!("enabling VBat");
        rtc.write_register(4, 0x09u8).unwrap();

        let weekd_status2 = rtc.read_register(4).unwrap();
        info!("weekd_status2 {:#x}", weekd_status2);
    }

    let sec_status = rtc.read_register(1).unwrap();
    info!("sec_status {:#x}", sec_status);
    if sec_status & 0x80 == 0 {
        info!("enabling RTC oscillator");
        rtc.write_register(1, 0x80u8).unwrap();
    }

    let hyperrampins = HyperRamPins::new(
        &mut pio2, p.PIN_6, p.PIN_7, p.PIN_8, p.PIN_9, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13,
        p.PIN_14, p.PIN_15, p.PIN_16,
    );

    let rom_info = LOADED_ROM_INFO.init({
        let mut hyperram = HyperRam::new(&mut pio2, &mut sm2_0, &hyperrampins);

        hyperram.init();

        let id0 = hyperram.read_cfg_blocking(hyperram::ID0);
        let id1 = hyperram.read_cfg_blocking(hyperram::ID1);
        let cfg0 = hyperram.read_cfg_blocking(hyperram::CFG0);
        let cfg1 = hyperram.read_cfg_blocking(hyperram::CFG1);
        info!(
            "ID0 {:#x}, ID1 {:#x}, CFG0 {:#x}, CFG1 {:#x}",
            id0, id1, cfg0, cfg1
        );

        let mut gb_bootloader = GbBootloader::new(
            volume_mgr,
            gb_mbc_commands_pio.rx_fifo(),
            gb_rom,
            gb_save_ram,
            &mut reset_pin,
            &mut hyperram,
            ws2812,
        );

        gb_bootloader.run().await
    });

    ws2812.write(&RGB8::default());

    spi_bus.lock(|bus| {
        bus.borrow_mut().blocking_write(&[0xFF; 2]).unwrap();
    });

    let mut hyperram = HyperRamReadOnly::new(&mut pio2, &pac::PIO2, sm2_0, hyperrampins);
    let dat = hyperram.read_blocking(0x4100u32);
    let dat2 = hyperram.read_blocking(0x4105u32);
    let dat3 = hyperram.read_blocking(0x5208u32);
    let dat4 = hyperram.read_blocking(0x5209u32);
    let dat5 = hyperram.read_blocking(0x6707u32);
    info!(
        "dat {:#x} dat2 {:#x} dat3 {:#x} dat4 {:#x} dat5 {:#x}",
        dat, dat2, dat3, dat4, dat5
    );

    let mbc: &mut dyn Mbc = match rom_info.mbc {
        MbcType::None => &mut NoMbc {},
        MbcType::Mbc1 => &mut Mbc1::new(
            gb_mbc_commands_pio.rx_fifo(),
            ptr::addr_of_mut!(current_higher_base_addr),
        ),
        MbcType::Mbc3 => &mut Mbc3::new(
            gb_mbc_commands_pio.rx_fifo(),
            ptr::addr_of_mut!(current_higher_base_addr),
            ptr::addr_of_mut!(gb_ram_ptr),
            gb_save_ram,
        ),
        MbcType::Mbc5 => &mut Mbc5::new(
            gb_mbc_commands_pio.rx_fifo(),
            ptr::addr_of_mut!(current_higher_base_addr),
            ptr::addr_of_mut!(gb_ram_ptr),
            gb_save_ram,
        ),
        _ => {
            panic!("unimplemented MBC");
        }
    };

    if rom_info.ram_bank_count > 0 {
        spawn_core1(
            p.CORE1,
            unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
            move || {
                let executor1 = EXECUTOR1.init(Executor::new());
                let gb_save_ram = unsafe {
                    core::slice::from_raw_parts(
                        ptr::addr_of!(_s_gb_save_ram) as *mut u8,
                        rom_info.ram_bank_count as usize * 0x2000usize,
                    )
                };
                executor1.run(|spawner| {
                    unwrap!(spawner.spawn(core1_task(
                        p.PIN_4.into(),
                        ws2812,
                        volume_mgr,
                        rom_info,
                        gb_save_ram
                    )))
                });
            },
        );
    }

    let _hyperram_gb_dma = GbReadSniffDmaConfig::new(
        p.DMA_CH9,
        p.DMA_CH10,
        p.DMA_CH11,
        p.DMA_CH12,
        &gb_rom_higher_pio,
        &hyperram,
        &hyperram,
        &gb_data_out_pio,
        ptr::addr_of_mut!(current_higher_base_addr),
    );

    gb_rom_higher_pio.start();

    cortex_m::interrupt::disable();

    reset_pin.set_low();

    mbc.run();

    loop {
        // defmt::info!("hello there!");
        Timer::after_millis(1500).await;
        //reset_pin.toggle();
    }
}

// Declare async tasks
type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn core1_task(
    button_pin: AnyPin,
    led: &'static mut dyn Ws2812Led,
    volume_mgr: &'static mut VolumeManagerType<'_>,
    rom_info: &'static RomInfo,
    saveram_memory: &'static [u8],
) {
    let mut async_input = Input::new(button_pin, Pull::Up);
    let saveram_filename = rom_info.savefile.as_str();

    info!("Core1 got saveram_filename {}", saveram_filename);

    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    let mut root_dir = volume0.open_root_dir().unwrap();

    loop {
        info!("Hello from core 1");
        async_input.wait_for_high().await;
        async_input.wait_for_low().await;

        led.write(&RGB8::new(16, 0, 0));

        match root_dir.open_file_in_dir(
            saveram_filename,
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        ) {
            Ok(mut savefile) => {
                info!("Found and opened savefile");

                match savefile.write(saveram_memory) {
                    Ok(_) => {
                        info!("Saved saveram to {}", saveram_filename);
                    }
                    Err(error) => {
                        warn!(
                            "Unable to write to {}: {}",
                            saveram_filename,
                            defmt::Debug2Format(&error)
                        );
                    }
                };

                savefile.close().unwrap();
            }
            Err(error) => {
                warn!(
                    "Unable to open savefile for saving {}",
                    defmt::Debug2Format(&error)
                );
            }
        };

        led.write(&RGB8::default());
    }
}
