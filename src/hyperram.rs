use embassy_futures::block_on;
use embassy_rp::{
    gpio::{Drive, SlewRate},
    pac,
    pio::{Common, Config, Instance, PioPin, ShiftConfig, ShiftDirection, StateMachine},
};

pub const ID0: u32 = 0u32 << 12 | 0u32 << 1;
pub const ID1: u32 = 0u32 << 12 | 1u32 << 1;
pub const CFG0: u32 = 1u32 << 12 | 0u32 << 1;
pub const CFG1: u32 = 1u32 << 12 | 1u32 << 1;

#[derive(Debug, Clone, Copy)]
enum CmdFlags {
    Read = 0xa0,
    Write = 0x20,
    RegWrite = 0x40,
    RegRead = 0xC0,
}

pub struct HyperRam<'d, P: Instance, const S: usize> {
    sm: StateMachine<'d, P, S>,
    pos_r_lat: u8,
    pos_finish: u8,
}

impl<'d, P: Instance, const S: usize> HyperRam<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        rwds_pin: impl PioPin,
        clk_pin: impl PioPin,
        cs_pin: impl PioPin,
        dq0_pin: impl PioPin,
        dq1_pin: impl PioPin,
        dq2_pin: impl PioPin,
        dq3_pin: impl PioPin,
        dq4_pin: impl PioPin,
        dq5_pin: impl PioPin,
        dq6_pin: impl PioPin,
        dq7_pin: impl PioPin,
    ) -> Self {
        let program = pio_proc::pio_file!("./pio/hyperram.pio");

        let mut ctrl_pins = [
            pio.make_pio_pin(rwds_pin),
            pio.make_pio_pin(clk_pin),
            pio.make_pio_pin(cs_pin),
        ];

        let mut dq_pins = [
            pio.make_pio_pin(dq0_pin),
            pio.make_pio_pin(dq1_pin),
            pio.make_pio_pin(dq2_pin),
            pio.make_pio_pin(dq3_pin),
            pio.make_pio_pin(dq4_pin),
            pio.make_pio_pin(dq5_pin),
            pio.make_pio_pin(dq6_pin),
            pio.make_pio_pin(dq7_pin),
        ];

        for pin in &mut ctrl_pins {
            pin.set_drive_strength(Drive::_8mA);
            pin.set_slew_rate(SlewRate::Fast);
            pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w| {
                w.set_ie(true);
            })
        }

        for pin in &mut dq_pins {
            pin.set_drive_strength(Drive::_8mA);
            pin.set_slew_rate(SlewRate::Fast);
            pac::PADS_BANK0.gpio(pin.pin() as usize).modify(|w| {
                w.set_ie(true);
            })
        }

        let mut cfg = Config::default();

        cfg.set_set_pins(&ctrl_pins.each_ref());
        cfg.set_in_pins(&dq_pins.each_ref());
        cfg.set_out_pins(&dq_pins.each_ref());
        cfg.set_jmp_pin(&ctrl_pins[0]);

        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };

        let lprogram = pio.load_program(&program.program);

        cfg.use_program(&lprogram, &[&ctrl_pins[1]]);
        let pos_r_lat = lprogram.origin + program.public_defines.r_lat as u8;
        let pos_finish = lprogram.origin + program.public_defines.finish as u8;

        sm.set_config(&cfg);

        sm.set_enable(true);

        Self {
            sm,
            pos_r_lat,
            pos_finish,
        }
    }

    fn create_cmd(&self, cmd_flags: CmdFlags, addr: u32, len: usize) -> [u32; 3] {
        let mut cmd = [0; 3];

        // Only halfword alignment
        let addr = addr >> 1;
        let addr_l = addr & 0x7u32;
        // Add flags to addr_h upper bits for Command/Address 0
        let addr_h = (addr >> 3) | ((cmd_flags as u32) << 24);

        // Convert word len to uint16 len - 1
        let len = (len * 2) - 1;

        match cmd_flags {
            CmdFlags::Read | CmdFlags::RegRead => {
                cmd[0] =
                    ((addr_h >> 16) & 0xff) << 24 | ((addr_h >> 24) & 0xff) << 16 | 0xff << 8 | 2;
                let cmd1_be = (addr_h << 16) | addr_l;
                cmd[1] = ((cmd1_be >> 24) & 0xff)
                    | (((cmd1_be >> 16) & 0xff) << 8)
                    | (((cmd1_be >> 8) & 0xff) << 16)
                    | (((cmd1_be >> 0) & 0xff) << 24);
                let next_pc = self.pos_r_lat as u32;
                cmd[2] = next_pc << 24 | 0x00 << 16 | len as u32;
            }
            _ => {}
        }

        cmd
    }

    fn create_cmd_reg_write(&self, addr: u32, reg: u16) -> [u32; 4] {
        let mut cmd: [u32; 4] = [0; 4];

        // Only halfword alignment
        let addr = addr >> 1;
        let addr_l = addr & 0x7u32;
        // Add flags to addr_h upper bits for Command/Address 0
        let addr_h = (addr >> 3) | ((CmdFlags::RegWrite as u32) << 24);

        /*
         * reg writes are sepecial as they do not use the initial latency. That is why the register
         * value is transfered as part of the addr and the length is set to 0.
         * The reads and write parts are skipped then.
         */

        cmd[0] = ((addr_h >> 16) & 0xff) << 24 | ((addr_h >> 24) & 0xff) << 16 | 0xff << 8 | 4;
        let cmd1_be = (addr_h << 16) | addr_l;
        cmd[1] = ((cmd1_be >> 24) & 0xff)
            | (((cmd1_be >> 16) & 0xff) << 8)
            | (((cmd1_be >> 8) & 0xff) << 16)
            | (((cmd1_be >> 0) & 0xff) << 24);

        let wdata = (reg << 8 | ((reg >> 8) & 0xff)) as u32;
        cmd[2] = wdata << 16 | wdata;

        let next_pc = self.pos_finish as u32;
        cmd[3] = next_pc << 24;

        cmd
    }

    pub fn read_cfg_blocking(&mut self, addr: u32) -> u16 {
        let cmd = self.create_cmd(CmdFlags::RegRead, addr, 1);

        block_on(self.sm.tx().wait_push(cmd[0]));
        block_on(self.sm.tx().wait_push(cmd[1]));
        block_on(self.sm.tx().wait_push(cmd[2]));

        let data_be = block_on(self.sm.rx().wait_pull());
        // Byte swap register data - they are big endian
        let data = ((data_be & 0xff) << 8 | ((data_be >> 8) & 0xff)) as u16;

        data
    }

    pub fn write_cfg_blocking(&mut self, addr: u32, reg_data: u16) {
        let cmds = self.create_cmd_reg_write(addr, reg_data);

        for cmd in cmds {
            block_on(self.sm.tx().wait_push(cmd));
        }
    }

    pub fn init(&mut self) {
        let cfgreg_init = (0x1u16 << 15) | // Do not enter power down
            //(0x1u << 12) | // 115R drive strength
            //(0x0u << 12) | // Default drive strength (34R)
            //(0x5u << 12) | // 27R drive strength
            (0x7u16 << 12) | // 19R drive strength
            (0xeu16 << 4)  | // 3 latency cycles (in bias -5 format)
            //(0xfu << 4)  | // 4 latency cycles (in bias -5 format)
            //(0x0u << 4)  | // 5 latency cycles (in bias -5 format)
            //(0x1u << 4)  | // 6 latency cycles (in bias -5 format)
            //(0x2u << 4)  | // Default 7 latency cycles (in bias -5 format)
            //(0x1u << 3);   // Fixed 2x latency mode
            (0x0u16 << 3); // Variable latency mode

        self.write_cfg_blocking(CFG0, cfgreg_init);
    }
}
