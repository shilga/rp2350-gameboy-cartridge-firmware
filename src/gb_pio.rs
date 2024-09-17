use embassy_rp::{
    pac,
    pio::{
        Common, Config, ExecConfig, Instance, PinConfig, PioPin, ShiftConfig, ShiftDirection,
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
        pincfg.out_base = program.public_defines.pin_data_base as u8;
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
    ) -> Self {
        let program = pio_proc::pio_file!(
            "./pio/gameboy_bus.pio",
            select_program("gameboy_bus_detect_a15_low_a14_irqs"),
            options(max_program_size = 32) // Optional, defaults to 32
        );
        let mut cfg = Config::default();
        let mut pincfg = PinConfig::default();
        pincfg.in_base = program.public_defines.pin_a15 as u8;
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_a14 as u8;
        unsafe {
            cfg.set_exec(execcfg);
        }
        cfg.shift_in = ShiftConfig {
            auto_fill: false,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.use_program(&pio.load_program(&program.program), &[]);

        sm.set_config(&cfg);

        Self { sm, p }
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
        unsafe {
            cfg.set_pins(pincfg);
        }
        let mut execcfg = ExecConfig::default();
        execcfg.jmp_pin = program.public_defines.pin_rd as u8;
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

    pub fn get_rx_reg(&self) -> *mut u32 {
        self.p.txf(S).as_ptr()
    }
}

pub struct GbPio<
    'd,
    P: Instance,
    const S0_0: usize,
    const S0_1: usize,
    const S0_2: usize,
    const S0_3: usize,
> {
    sm_lower_rom: StateMachine<'d, P, S0_0>,
    sm_upper_rom: StateMachine<'d, P, S0_1>,
    sm_rom_detect: StateMachine<'d, P, S0_2>,
    sm_data_out: StateMachine<'d, P, S0_3>,
}

impl<
        'd,
        P: Instance,
        const S0_0: usize,
        const S0_1: usize,
        const S0_2: usize,
        const S0_3: usize,
    > GbPio<'d, P, S0_0, S0_1, S0_2, S0_3>
{
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm_lower_rom: StateMachine<'d, P, S0_0>,
        mut sm_upper_rom: StateMachine<'d, P, S0_1>,
        mut sm_rom_detect: StateMachine<'d, P, S0_2>,
        mut sm_data_out: StateMachine<'d, P, S0_3>,
        p: Peripherals,
    ) -> Self {
        let pio_data_pins = [
            &pio.make_pio_pin(p.PIN_36),
            &pio.make_pio_pin(p.PIN_37),
            &pio.make_pio_pin(p.PIN_38),
            &pio.make_pio_pin(p.PIN_39),
            &pio.make_pio_pin(p.PIN_40),
            &pio.make_pio_pin(p.PIN_41),
            &pio.make_pio_pin(p.PIN_42),
            &pio.make_pio_pin(p.PIN_43),
        ];

        let pio_addr_pins = [
            &pio.make_pio_pin(p.PIN_20),
            &pio.make_pio_pin(p.PIN_21),
            &pio.make_pio_pin(p.PIN_22),
            &pio.make_pio_pin(p.PIN_23),
            &pio.make_pio_pin(p.PIN_24),
            &pio.make_pio_pin(p.PIN_25),
            &pio.make_pio_pin(p.PIN_26),
            &pio.make_pio_pin(p.PIN_27),
            &pio.make_pio_pin(p.PIN_28),
            &pio.make_pio_pin(p.PIN_29),
            &pio.make_pio_pin(p.PIN_30),
            &pio.make_pio_pin(p.PIN_31),
            &pio.make_pio_pin(p.PIN_32),
            &pio.make_pio_pin(p.PIN_33),
            &pio.make_pio_pin(p.PIN_34),
            &pio.make_pio_pin(p.PIN_35),
        ];

        let pio_clk_pin = pio.make_pio_pin(p.PIN_17);
        let pio_rd_pin = pio.make_pio_pin(p.PIN_18);
        let pio_cs_pin = pio.make_pio_pin(p.PIN_19);

        Self {
            sm_lower_rom,
            sm_upper_rom,
            sm_rom_detect,
            sm_data_out,
        }
    }

    pub fn get_tx_reg() -> *mut u32 {
        pac::PIO0.txf(S0_3).as_ptr()
    }
}
