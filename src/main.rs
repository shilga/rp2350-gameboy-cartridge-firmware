//! This example shows how to use UART (Universal asynchronous receiver-transmitter) in the RP2040 chip.
//!
//! No specific hardware is specified in this example. Only output on pin 0 is tested.
//! The Raspberry Pi Debug Probe (https://www.raspberrypi.com/products/debug-probe/) could be used
//! with its UART port.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pac;
use embassy_rp::peripherals::USB;
use embassy_rp::peripherals::{PIO0, PIO1, UART0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio, PioPin};
use embassy_rp::uart::{self};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{block::ImageDef, uart::UartTx};

use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Config, UsbDevice};

use embassy_time::Timer;

use embedded_io::{ErrorType, Write};
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
use crate::gb_pio::{GbDataOut, GbRomDetect, GbRomLower};

mod gb_dma;
use crate::gb_dma::GbReadDmaConfig;

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

#[link_section = ".gb_rom_memory"]
#[used]
pub static mut GB_ROM_MEMORY: MaybeUninit<[u8; 4 * 0x4000]> = MaybeUninit::uninit();

extern "C" {
    static mut _s_gb_rom_memory: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let config = uart::Config::default();
    // let uart = uart::UartTx::new_blocking(p.UART0, p.PIN_46, config);

    // let serialwrapper = SerialWrapper { uart };

    // defmt_serial::defmt_serial(SERIAL.init(serialwrapper));

    info!("Hello defmt-world!");

    let mut reset_pin = Output::new(p.PIN_45, Level::High);
    let mut _gb_bus_en = Output::new(p.PIN_44, Level::High);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let ws2812 = Ws2812::new(&mut common, sm0, p.PIN_47);

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

    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let mut gb_rom_detect_pio = GbRomDetect::new(&mut common, &pac::PIO1, sm2, p.PIN_46);
    let mut gb_rom_lower_pio = GbRomLower::new(&mut common, &pac::PIO1, sm0);
    let mut gb_data_out_pio = GbDataOut::new(&mut common, &pac::PIO1, sm3);

    info!("gpiobase: {}", pac::PIO1.gpiobase().read().gpiobase());

    let gb_pio_pins = [
        common.make_pio_pin(p.PIN_17),
        common.make_pio_pin(p.PIN_18),
        common.make_pio_pin(p.PIN_19),
        // addr pins
        common.make_pio_pin(p.PIN_20),
        common.make_pio_pin(p.PIN_21),
        common.make_pio_pin(p.PIN_22),
        common.make_pio_pin(p.PIN_23),
        common.make_pio_pin(p.PIN_24),
        common.make_pio_pin(p.PIN_25),
        common.make_pio_pin(p.PIN_26),
        common.make_pio_pin(p.PIN_27),
        common.make_pio_pin(p.PIN_28),
        common.make_pio_pin(p.PIN_29),
        common.make_pio_pin(p.PIN_30),
        common.make_pio_pin(p.PIN_31),
        common.make_pio_pin(p.PIN_32),
        common.make_pio_pin(p.PIN_33),
        common.make_pio_pin(p.PIN_34),
        common.make_pio_pin(p.PIN_35),
        //data pins
        common.make_pio_pin(p.PIN_36),
        common.make_pio_pin(p.PIN_37),
        common.make_pio_pin(p.PIN_38),
        common.make_pio_pin(p.PIN_39),
        common.make_pio_pin(p.PIN_40),
        common.make_pio_pin(p.PIN_41),
        common.make_pio_pin(p.PIN_42),
        common.make_pio_pin(p.PIN_43),   
    ];

    for mut pin in gb_pio_pins {
        // pin.set_pull(gpio::Pull::None);
        pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w|{
            w.set_ie(true);
        })
    }

    let gb_rom = unsafe {
        core::slice::from_raw_parts_mut(ptr::addr_of!(_s_gb_rom_memory) as *mut u8, 0x4000)
    };

    unsafe {
        info!("pos0 before: {}", _s_gb_rom_memory);
    }

    let mut testvarptr = unsafe { ptr::addr_of_mut!(_s_gb_rom_memory) };
    info!("testvarptr {}", testvarptr);

    let bytes = include_bytes!("bootloader.gb");
    gb_rom[..bytes.len()].copy_from_slice(bytes);

    unsafe {
        info!("pos 0x30 after: {}", *(testvarptr.add(0x30usize)) );
    }

    let _read_dma_lower = GbReadDmaConfig::new(
        p.DMA_CH0,
        p.DMA_CH1,
        p.DMA_CH2,
        ptr::addr_of_mut!(testvarptr),
        gb_rom_lower_pio.get_rx_reg(),
        gb_data_out_pio.get_tx_reg(),
        gb_rom_lower_pio.get_rx_dreq(),
    );

    // Spawned tasks run in the background, concurrently.
    spawner.spawn(blink(ws2812)).unwrap();
    spawner.spawn(usb_task(usb)).unwrap();

    gb_rom_lower_pio.start();
    gb_data_out_pio.start();
    gb_rom_detect_pio.start();

    reset_pin.set_low();

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
