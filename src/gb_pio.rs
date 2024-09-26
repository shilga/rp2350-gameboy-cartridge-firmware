use embassy_rp::{
    pac,
    pio::{
        Common, Config, ExecConfig, Instance, PinConfig, PioPin, ShiftConfig, ShiftDirection, Direction,
        StateMachine,
    },
    Peripherals,
};

pub struct GbDataOut<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}

impl<'d, P: Instance, const S: usize> GbDataOut<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
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

        Self { sm, p }
    }

    pub fn start(&mut self) {
        self.sm.set_enable(true);
    }

    pub fn get_tx_reg(&self) -> *mut u32 {
        self.p.txf(S).as_ptr()
    }
}

pub struct GbRomDetect<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    p: &'static pac::pio::Pio,
}

impl<'d, P: Instance, const S: usize> GbRomDetect<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        p: &'static pac::pio::Pio,
        mut sm: StateMachine<'d, P, S>,
        pin: impl PioPin
    ) -> Self {
        let debug_pin = pio.make_pio_pin(pin);

        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_detect_a15_low_a14_irqs"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_a15 as u8; // todo: offset should be calculated automatically
        unsafe {
            cfg.set_pins(pincfg);
        }
        cfg.set_set_pins(&[&debug_pin]); 
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_a14 as u8 - 16;  // todo: offset should be calculated automatically
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
        cfg.use_program(&lprogram, &[&debug_pin]);

        // program.program.set_origin(origin)

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&debug_pin]);

        Self { sm, p }
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

    pub fn get_rx_reg(&self) -> *mut u32 {
        self.p.rxf(S).as_ptr()
    }

    pub fn get_rx_dreq(&self) -> pac::dma::vals::TreqSel {
        let pio_num: u8 = ((self.p.as_ptr() as u32 - pac::PIO0.as_ptr() as u32) / 0x10_0000u32)
            .try_into()
            .unwrap();
        pac::dma::vals::TreqSel::from(pio_num * 8 + 4 + S as u8)
    }
}
