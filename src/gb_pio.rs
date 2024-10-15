use crate::dma_helper::{DmaReadTarget, DmaWriteTarget};
use embassy_rp::{
    pac,
    pio::{
        Common, Config, Direction, ExecConfig, Instance, Pin, PinConfig, PioPin, ShiftConfig,
        ShiftDirection, StateMachine, StateMachineRx,
    },
};

pub struct GbPioPins<'d, P: Instance> {
    pub _addr_pins: [Pin<'d, P>; 16],
    pub _data_pins: [Pin<'d, P>; 8],
    pub _ctrl_pins: [Pin<'d, P>; 3],
    pub debug_pin: Option<Pin<'d, P>>,
}
impl<'d, P: Instance> GbPioPins<'d, P> {
    pub fn new(
        pio: &mut Common<'d, P>,
        clk_pin: impl PioPin,
        rd_pin: impl PioPin,
        cs_pin: impl PioPin,
        a0_pin: impl PioPin,
        a1_pin: impl PioPin,
        a2_pin: impl PioPin,
        a3_pin: impl PioPin,
        a4_pin: impl PioPin,
        a5_pin: impl PioPin,
        a6_pin: impl PioPin,
        a7_pin: impl PioPin,
        a8_pin: impl PioPin,
        a9_pin: impl PioPin,
        a10_pin: impl PioPin,
        a11_pin: impl PioPin,
        a12_pin: impl PioPin,
        a13_pin: impl PioPin,
        a14_pin: impl PioPin,
        a15_pin: impl PioPin,
        d0_pin: impl PioPin,
        d1_pin: impl PioPin,
        d2_pin: impl PioPin,
        d3_pin: impl PioPin,
        d4_pin: impl PioPin,
        d5_pin: impl PioPin,
        d6_pin: impl PioPin,
        d7_pin: impl PioPin,
        debug_pin: Option<impl PioPin>,
    ) -> Self {
        let mut ctrl_pins: [embassy_rp::pio::Pin<'_, P>; 3] = [
            pio.make_pio_pin(clk_pin),
            pio.make_pio_pin(rd_pin),
            pio.make_pio_pin(cs_pin),
        ];

        let mut addr_pins: [embassy_rp::pio::Pin<'_, P>; 16] = [
            pio.make_pio_pin(a0_pin),
            pio.make_pio_pin(a1_pin),
            pio.make_pio_pin(a2_pin),
            pio.make_pio_pin(a3_pin),
            pio.make_pio_pin(a4_pin),
            pio.make_pio_pin(a5_pin),
            pio.make_pio_pin(a6_pin),
            pio.make_pio_pin(a7_pin),
            pio.make_pio_pin(a8_pin),
            pio.make_pio_pin(a9_pin),
            pio.make_pio_pin(a10_pin),
            pio.make_pio_pin(a11_pin),
            pio.make_pio_pin(a12_pin),
            pio.make_pio_pin(a13_pin),
            pio.make_pio_pin(a14_pin),
            pio.make_pio_pin(a15_pin),
        ];

        let mut data_pins = [
            pio.make_pio_pin(d0_pin),
            pio.make_pio_pin(d1_pin),
            pio.make_pio_pin(d2_pin),
            pio.make_pio_pin(d3_pin),
            pio.make_pio_pin(d4_pin),
            pio.make_pio_pin(d5_pin),
            pio.make_pio_pin(d6_pin),
            pio.make_pio_pin(d7_pin),
        ];

        for pin in &mut ctrl_pins {
            pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w| {
                w.set_ie(true);
            })
        }

        for pin in &mut addr_pins {
            pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w| {
                w.set_ie(true);
            })
        }

        for pin in &mut data_pins {
            pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w| {
                w.set_ie(true);
            })
        }

        let debug_pin = match debug_pin {
            Some(pin) => {
                let pin = pio.make_pio_pin(pin);
                Some(pin)
            }
            None => None,
        };

        Self {
            _ctrl_pins: ctrl_pins,
            _addr_pins: addr_pins,
            _data_pins: data_pins,
            debug_pin,
        }
    }
}

pub struct GbDataOut<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}

impl<'d, P: Instance, const S: usize> GbDataOut<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
        pins: &GbPioPins<'d, P>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_write_to_data"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_clk as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        cfg.shift_out = ShiftConfig {
            auto_fill: false,
            threshold: 8,
            direction: ShiftDirection::Right,
        };
        cfg.use_program(&pio.load_program(&program.program), &[]);

        sm.set_config(&cfg);

        match &pins.debug_pin {
            Some(pin) => {
                sm.set_pin_dirs(Direction::Out, &[&pin]);
            }
            _ => {}
        }

        Self { sm, p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }
}

unsafe impl<'d, P: Instance, const S: usize> DmaWriteTarget for GbDataOut<'d, P, S> {
    type TransmittedWord = u8;

    fn tx_treq(&self) -> Option<u8> {
        let pio_num: u8 = ((self.p.as_ptr() as u32 - pac::PIO0.as_ptr() as u32) / 0x10_0000u32)
            .try_into()
            .unwrap();

        Some(pio_num * 8 + S as u8)
    }

    fn tx_address_count(&self) -> (u32, u32) {
        let ptr = self.p.txf(S).as_ptr();
        (ptr as u32, 1u32)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

pub struct GbRomDetect<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> GbRomDetect<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        _p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_detect_a15_low_a14_irqs"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_a15 as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_a14 as u8 - 16; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        let mut lprogram = pio.load_program(&program.program);
        lprogram.origin += program.public_defines.entry_point as u8; // offset the start
        cfg.use_program(&lprogram, &[]);

        sm.set_config(&cfg);

        Self { sm }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }
}

pub struct GbRomLower<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}
impl<'d, P: Instance, const S: usize> GbRomLower<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
        _pins: &GbPioPins<'d, P>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_rom_low"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_ad_base as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_rd as u8 - 16; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 14,
            direction: ShiftDirection::Left,
        };
        cfg.use_program(&pio.load_program(&program.program), &[]);

        sm.set_config(&cfg);

        Self { sm, p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }
}

unsafe impl<'d, P: Instance, const S: usize> DmaReadTarget for GbRomLower<'d, P, S> {
    type ReceivedWord = u32;

    fn rx_treq(&self) -> Option<u8> {
        let pio_num: u8 = ((self.p.as_ptr() as u32 - pac::PIO0.as_ptr() as u32) / 0x10_0000u32)
            .try_into()
            .unwrap();

        Some(pio_num * 8 + 4 + S as u8)
    }

    fn rx_address_count(&self) -> (u32, u32) {
        let ptr = self.p.rxf(S).as_ptr();
        (ptr as u32, 1u32)
    }

    fn rx_increment(&self) -> bool {
        false
    }
}

pub struct GbRomHigher<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}
impl<'d, P: Instance, const S: usize> GbRomHigher<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
        _pins: &GbPioPins<'d, P>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_rom_high"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_ad_base as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_rd as u8 - 16; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 14,
            direction: ShiftDirection::Left,
        };
        cfg.use_program(&pio.load_program(&program.program), &[]);

        sm.set_config(&cfg);

        Self { sm, p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }
}

unsafe impl<'d, P: Instance, const S: usize> DmaReadTarget for GbRomHigher<'d, P, S> {
    type ReceivedWord = u32;

    fn rx_treq(&self) -> Option<u8> {
        let pio_num: u8 = ((self.p.as_ptr() as u32 - pac::PIO0.as_ptr() as u32) / 0x10_0000u32)
            .try_into()
            .unwrap();

        Some(pio_num * 8 + 4 + S as u8)
    }

    fn rx_address_count(&self) -> (u32, u32) {
        let ptr = self.p.rxf(S).as_ptr();
        (ptr as u32, 1u32)
    }

    fn rx_increment(&self) -> bool {
        false
    }
}

pub struct GbRamRead<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}
impl<'d, P: Instance, const S: usize> GbRamRead<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_saveram_read"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_ad_base as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_rd as u8 - 16; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Right,
        };

        let mut lprogram = pio.load_program(&program.program);
        lprogram.origin += program.public_defines.entry_point as u8; // offset the start
        cfg.use_program(&lprogram, &[]);

        sm.set_config(&cfg);

        Self { sm, p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }
}

unsafe impl<'d, P: Instance, const S: usize> DmaReadTarget for GbRamRead<'d, P, S> {
    type ReceivedWord = u32;

    fn rx_treq(&self) -> Option<u8> {
        let pio_num: u8 = ((self.p.as_ptr() as u32 - pac::PIO0.as_ptr() as u32) / 0x10_0000u32)
            .try_into()
            .unwrap();

        Some(pio_num * 8 + 4 + S as u8)
    }

    fn rx_address_count(&self) -> (u32, u32) {
        let ptr = self.p.rxf(S).as_ptr();
        (ptr as u32, 1u32)
    }

    fn rx_increment(&self) -> bool {
        false
    }
}

pub struct GbMbcCommands<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    _p: &'static pac::pio::Pio,
}
impl<'d, P: Instance, const S: usize> GbMbcCommands<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_mbc_commands"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_ad_base as u8;
        pincfg.out_base = program.public_defines.pin_data_base as u8;
        pincfg.out_count = 8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_rd as u8 - 16; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };

        let mut lprogram = pio.load_program(&program.program);
        lprogram.origin += program.public_defines.entry_point as u8; // offset the start
        cfg.use_program(&lprogram, &[]);

        sm.set_config(&cfg);

        Self { sm, _p: p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    pub fn rx_fifo(&mut self) -> &mut StateMachineRx<'d, P, S> {
        self.sm.rx()
    }
}
