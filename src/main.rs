//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! No specific hardware is specified in this example. Only output on pin 0 is tested.
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::peripherals::{PIO0, PIO1, PIO2, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::uart::{self};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, spi};
use embassy_rp::{block::ImageDef, uart::UartTx};
use embassy_rp::{clocks, config as rpconfig, pac};
use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_embedded_hal::SetConfig;

use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Config, UsbDevice};

use embassy_time::{Instant, Timer};

use embedded_io::{ErrorType, Write};

use embedded_sdmmc::sdcard::SdCard;

use smart_leds::RGB8;

use core::mem::MaybeUninit;
use core::ptr;

use defmt::info;
use {defmt_serial as _, panic_probe as _};

use static_cell::StaticCell;

mod ws2812;
use crate::ws2812::Ws2812;

mod picotool_reset;
use crate::picotool_reset::PicotoolReset;

mod gb_pio;
use crate::gb_pio::{GbDataOut, GbMbcCommands, GbPioPins, GbRomDetect, GbRomHigher, GbRomLower};

mod gb_dma;
use crate::gb_dma::{GbReadDmaConfig, GbReadSniffDmaConfig};

mod gb_mbc;
use crate::gb_mbc::Mbc1;

mod hyperram;
use crate::hyperram::{HyperRam, HyperRamPins, HyperRamReadOnly};

mod dma_helper;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"example"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_description!(c"Blinky"),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// This is a randomly generated GUID to allow clients on Windows to find our device
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{AFB9A6FB-30BA-44BC-9232-806CFC875321}"];

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

#[link_section = ".gb_rom_memory"]
#[used]
pub static mut GB_ROM_MEMORY: MaybeUninit<[u8; 4 * 0x4000]> = MaybeUninit::uninit();

extern "C" {
    static mut _s_gb_rom_memory: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    embassy_rp::pac::SIO.spinlock(31).write_value(1);
    let mut rp_config = rpconfig::Config::default();
    rp_config
        .clocks
        .xosc
        .as_mut()
        .unwrap()
        .sys_pll
        .as_mut()
        .unwrap()
        .post_div1 = 5;
    let p = embassy_rp::init(rp_config);
    // let config = uart::Config::default();
    // let uart = uart::UartTx::new_blocking(p.UART0, p.PIN_46, config);

    // let serialwrapper = SerialWrapper { uart };

    // defmt_serial::defmt_serial(SERIAL.init(serialwrapper));

    let sys_freq = clocks::clk_sys_freq();
    info!("Hello defmt-world!, running at {} hz", sys_freq);

    let mut reset_pin = Output::new(p.PIN_45, Level::High);
    let mut _gb_bus_en = Output::new(p.PIN_44, Level::High);

    let Pio {
        common: mut pio0,
        sm0: sm0_0,
        sm1: sm0_1,
        sm2: _sm0_2,
        sm3: _sm0_3,
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

    let ws2812 = Ws2812::new(&mut pio0, sm0_0, p.PIN_47);

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    // Create a USB device RPI Vendor ID and on of these Product ID:
    // https://github.com/raspberrypi/picotool/blob/master/picoboot_connection/picoboot_connection.c#L23-L27
    let mut config = Config::new(0x2e8a, 0x0009);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB raw example");
    config.serial_number = Some("12345678");
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

    #[rustfmt::skip]
    let gb_pio_pins = GbPioPins::new(
        &mut pio1,
        p.PIN_17, p.PIN_18, p.PIN_19,
        p.PIN_20, p.PIN_21, p.PIN_22, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_26, p.PIN_27,
        p.PIN_28, p.PIN_29, p.PIN_30, p.PIN_31, p.PIN_32, p.PIN_33, p.PIN_34, p.PIN_35,
        p.PIN_36, p.PIN_37, p.PIN_38, p.PIN_39, p.PIN_40, p.PIN_41, p.PIN_42, p.PIN_43,
        Some(p.PIN_46)
    );
    let mut gb_rom_detect_pio = GbRomDetect::new(&mut pio1, &pac::PIO1, sm1_2);
    let mut gb_rom_lower_pio = GbRomLower::new(&mut pio1, &pac::PIO1, sm1_0, &gb_pio_pins);
    let mut gb_rom_higher_pio = GbRomHigher::new(&mut pio1, &pac::PIO1, sm1_1, &gb_pio_pins);
    let mut gb_data_out_pio = GbDataOut::new(&mut pio1, &pac::PIO1, sm1_3, &gb_pio_pins);

    info!("gpiobase: {}", pac::PIO1.gpiobase().read().gpiobase());

    let gb_rom = unsafe {
        core::slice::from_raw_parts_mut(ptr::addr_of!(_s_gb_rom_memory) as *mut u8, 0x10000)
    };

    let mut testvarptr = unsafe { ptr::addr_of_mut!(_s_gb_rom_memory) };

    // let bytes = include_bytes!("bootloader.gb");
    // gb_rom[..bytes.len()].copy_from_slice(bytes);

    let _read_dma_lower = GbReadDmaConfig::new(
        p.DMA_CH0,
        p.DMA_CH1,
        p.DMA_CH2,
        ptr::addr_of_mut!(testvarptr),
        &gb_rom_lower_pio,
        &gb_data_out_pio,
    );

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(blink(ws2812)).unwrap();
    spawner.spawn(usb_task(usb)).unwrap();

    gb_rom_lower_pio.start();
    gb_rom_higher_pio.start();
    gb_data_out_pio.start();
    gb_rom_detect_pio.start();

    // SPI clock needs to be running at <= 400kHz during initialization
    let mut config = spi::Config::default();
    config.frequency = 400_000;
    let mut spi = spi::Spi::new_blocking(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, config);

    let cs_sd_pin = Output::new(p.PIN_1, Level::High);
    let _cs_rtc_pin = Output::new(p.PIN_5, Level::High);

    // pre-initialize with 74 clock cycles according to SD-card spec
    spi.blocking_write(&[0xFF; 10]).unwrap();

    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs_sd_pin);

    // open the sd card and fully initialize it
    let sdcard = SdCard::new(spi_dev, embassy_time::Delay);
    info!("Card size is {} bytes", sdcard.num_bytes().unwrap());

    // Now that the card is initialized, the SPI clock can go faster
    let mut config = spi::Config::default();
    config.frequency = 16_000_000;
    sdcard.spi(|dev| dev.bus_mut().set_config(&config)).ok();

    // Now let's look for volumes (also known as partitions) on our block device.
    // To do this we need a Volume Manager. It will take ownership of the block device.
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, DummyTimesource());

    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
    let mut volume0 = volume_mgr
        .open_volume(embedded_sdmmc::VolumeIdx(0))
        .unwrap();
    info!("Volume 0: {:?}", defmt::Debug2Format(&volume0));

    // Open the root directory (mutably borrows from the volume).
    let mut root_dir = volume0.open_root_dir().unwrap();
    root_dir
        .iterate_dir(|entry| {
            info!(
                "{} {} {}",
                defmt::Debug2Format(&entry.name),
                entry.size,
                if entry.attributes.is_directory() {
                    "<DIR>"
                } else {
                    ""
                }
            );
        })
        .unwrap();

    let mut file = root_dir
        .open_file_in_dir("MARIO.GB", embedded_sdmmc::Mode::ReadOnly)
        .unwrap();
    let read_start_time = Instant::now();
    let num_read = file.read(gb_rom).unwrap();
    let read_duration = read_start_time.elapsed();
    info!(
        "Read {} bytes in {} ms",
        num_read,
        read_duration.as_millis()
    );

    let hyperrampins = HyperRamPins::new(
        &mut pio2, p.PIN_6, p.PIN_7, p.PIN_8, p.PIN_9, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13,
        p.PIN_14, p.PIN_15, p.PIN_16,
    );

    {
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

        let write_start_time = Instant::now();
        hyperram.write_blocking(0x4000, &gb_rom[0x4000..0x8000]);
        let write_duration = write_start_time.elapsed();
        info!("Writing took {}", write_duration);
        let mut test_read: [u8; 16] = [0; 16];
        hyperram.read_blocking(0x100u32, &mut test_read);
        info!("test_read: {}", test_read);
        let mut test_read2: [u8; 16] = [0; 16];
        hyperram.read_blocking(0x5700u32, &mut test_read2);
        info!("test_read2: {}", test_read2);

        hyperram.write_blocking(0x8000, &gb_rom[0x8000..0xC000]);
        hyperram.write_blocking(0xC000, &gb_rom[0xC000..]);
    }

    let mut hyperram = HyperRamReadOnly::new(&mut pio2, &pac::PIO2, sm2_0, hyperrampins);
    let dat = hyperram.read_blocking(0x100u32);
    let dat2 = hyperram.read_blocking(0x105u32);
    let dat3 = hyperram.read_blocking(0x1208u32);
    let dat4 = hyperram.read_blocking(0x1209u32);
    let dat5 = hyperram.read_blocking(0x6707u32);
    info!(
        "dat {:#x} dat2 {:#x} dat3 {:#x} dat4 {:#x} dat5 {:#x}",
        dat, dat2, dat3, dat4, dat5
    );

    let mut current_higher_base_addr: u32 = 0x4000u32;

    let _hyperram_gb_dma = GbReadSniffDmaConfig::new(
        p.DMA_CH3,
        p.DMA_CH4,
        p.DMA_CH5,
        p.DMA_CH6,
        &gb_rom_higher_pio,
        &hyperram,
        &hyperram,
        &gb_data_out_pio,
        ptr::addr_of_mut!(current_higher_base_addr),
    );

    let mut gb_mbc_commands_pio = GbMbcCommands::new(&mut pio0, &pac::PIO0, sm0_1);

    gb_mbc_commands_pio.start();

    let mut mbc = Mbc1::new(
        gb_mbc_commands_pio.rx_fifo(),
        ptr::addr_of_mut!(current_higher_base_addr),
    );

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
#[embassy_executor::task]
async fn blink(mut led: Ws2812<'static, PIO0, 0>) {
    {
        loop {
            let off = RGB8::default();
            let blue = RGB8::new(0, 0, 16);
            let red = RGB8::new(16, 0, 0);
            let green = RGB8::new(0, 16, 0);
            led.write(&off);
            Timer::after_millis(500).await;
            led.write(&blue);
            Timer::after_millis(500).await;
            led.write(&red);
            Timer::after_millis(500).await;
            led.write(&green);
            Timer::after_millis(500).await;
        }
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
