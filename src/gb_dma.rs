/* RP2350 GameBoy cartridge
 * Copyright (C) 2025 Sebastian Quilitz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

use core::ptr;

use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::pac;
use embassy_rp::Peri;

use defmt::info;

use crate::dma_helper::{DmaReadTarget, DmaWriteTarget};
use crate::gb_mbc::MbcRamControl;

const REG_ALIAS_SET_BITS: u32 = 0x2u32 << 12u32;
static mut DEV_NULL: u32 = 0;
static mut DEV_NULL_PTR: *mut u32 = ptr::addr_of_mut!(DEV_NULL);

pub struct GbReadDmaConfig<'d> {
    _dma_ch0: Peri<'d, AnyChannel>,
    _dma_ch1: Peri<'d, AnyChannel>,
    _dma_ch2: Peri<'d, AnyChannel>,
}

impl<'d> GbReadDmaConfig<'d> {
    pub fn new(
        dma0: Peri<'d, AnyChannel>,
        dma1: Peri<'d, AnyChannel>,
        dma2: Peri<'d, AnyChannel>,
        read_base_addr_ptr: *mut *mut u8,
        addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        write_target: &dyn DmaWriteTarget<TransmittedWord = u8>,
    ) -> Self {
        info!("read_base_addr_ptr: {:#010x}", read_base_addr_ptr as u32);
        info!(
            "read_addr_rx_fifo: {:#010x}",
            addr_read_target.rx_address_count().0
        );
        info!(
            "write_to_data_tx_fifo: {:#010x}",
            write_target.tx_address_count().0
        );
        info!("dreq: {:x}", addr_read_target.rx_treq().unwrap() as u32);

        let p0 = dma0.regs();
        let p1 = dma1.regs();
        let p2 = dma2.regs();

        // setup channel 1, which reads from buffer and writes to tx fifo, do not trigger it, channel 2 will trigger it
        p1.trans_count().write(|w| {
            w.set_count(1);
        });
        p1.write_addr()
            .write_value(write_target.tx_address_count().0);
        let mut dma1_cfg = pac::dma::regs::CtrlTrig(0);
        dma1_cfg.set_incr_read(false);
        dma1_cfg.set_incr_write(false);
        dma1_cfg.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        dma1_cfg.set_chain_to(dma0.number());
        dma1_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_BYTE);
        dma1_cfg.set_en(true);
        p1.al1_ctrl().write_value(dma1_cfg.0);

        /* setup channel 0, which reads from the RX FIFO on DREQ, and writes the addr to the read addr register of channel 1. Trigger it, so it will start on DREQ */
        p0.trans_count().write(|w| {
            w.set_count(1);
        });
        p0.read_addr()
            .write_value(addr_read_target.rx_address_count().0);
        p0.write_addr().write_value(p1.read_addr().as_ptr() as u32);
        p0.ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_treq_sel(pac::dma::vals::TreqSel::from(
                addr_read_target
                    .rx_treq()
                    .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
            ));
            w.set_chain_to(dma2.number());
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_en(true);
        });

        /* setup channel 2, which reads the base addr and writes it to the read addr of channel 1, while triggering it. The write happens through the set-shadow register, to combine it with the addr written before from channel 0*/
        p2.trans_count().write(|w| {
            w.set_count(1);
        });
        p2.read_addr().write_value(read_base_addr_ptr as u32);
        p2.write_addr()
            .write_value((p1.al3_read_addr_trig().as_ptr() as u32) | REG_ALIAS_SET_BITS);
        p2.ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
            w.set_chain_to(dma2.number()); // chain to itself -> disable
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_en(true);
        });

        Self {
            _dma_ch0: dma0,
            _dma_ch1: dma1,
            _dma_ch2: dma2,
        }
    }
}

pub struct GbReadSniffDmaConfig<'d> {
    _dma_ch0: Peri<'d, AnyChannel>,
    _dma_ch1: Peri<'d, AnyChannel>,
    _dma_ch2: Peri<'d, AnyChannel>,
    _dma_ch3: Peri<'d, AnyChannel>,
}

impl<'d> GbReadSniffDmaConfig<'d> {
    pub fn new(
        dma0: Peri<'d, AnyChannel>,
        dma1: Peri<'d, AnyChannel>,
        dma2: Peri<'d, AnyChannel>,
        dma3: Peri<'d, AnyChannel>,
        addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        data_fetcher_write_target: &dyn DmaWriteTarget<TransmittedWord = u32>,
        data_fetcher_read_target: &dyn DmaReadTarget<ReceivedWord = u8>,
        write_target: &dyn DmaWriteTarget<TransmittedWord = u8>,
        base_addr_ptr: *const u32,
    ) -> Self {
        let p0 = dma0.regs();
        let p1 = dma1.regs();
        let p2 = dma2.regs();
        let p3 = dma3.regs();

        // setup channel 0, which reads the addr from the the gb-pio and writes it into the sum register to begin calculation
        p0.trans_count().write(|w| {
            w.set_count(1);
        });
        p0.write_addr()
            .write_value(pac::DMA.sniff_data().as_ptr() as u32);
        p0.read_addr()
            .write_value(addr_read_target.rx_address_count().0);
        let mut dma0_cfg = pac::dma::regs::CtrlTrig(0);
        dma0_cfg.set_incr_read(false);
        dma0_cfg.set_incr_write(false);
        dma0_cfg.set_treq_sel(pac::dma::vals::TreqSel::from(
            addr_read_target
                .rx_treq()
                .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
        ));
        dma0_cfg.set_chain_to(dma1.number());
        dma0_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        dma0_cfg.set_en(true);
        p0.al1_ctrl().write_value(dma0_cfg.0);

        /* setup channel 1, which reads the base addr and does a dummy transfer. This will be catched by the sniffer. It will be triggered by channel 0 */
        p1.trans_count().write(|w| {
            w.set_count(1);
        });
        p1.read_addr().write_value(base_addr_ptr as u32);
        p1.write_addr()
            .write_value(ptr::addr_of_mut!(DEV_NULL) as u32);
        let mut dma1_cfg = pac::dma::regs::CtrlTrig(0);
        dma1_cfg.set_incr_read(false);
        dma1_cfg.set_incr_write(false);
        dma1_cfg.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        dma1_cfg.set_chain_to(dma2.number());
        dma1_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        dma1_cfg.set_sniff_en(true); // allow sniffing of this channel
        dma1_cfg.set_en(true);
        p1.al1_ctrl().write_value(dma1_cfg.0);

        /* setup channel 2, which reads the calculated addr and writes it to the data fetcher. It will be triggered by channel 1 */
        p2.trans_count().write(|w| {
            w.set_count(1);
        });
        p2.read_addr()
            .write_value(pac::DMA.sniff_data().as_ptr() as u32);
        p2.write_addr()
            .write_value(data_fetcher_write_target.tx_address_count().0 as u32);
        let mut dma2_cfg = pac::dma::regs::CtrlTrig(0);
        dma2_cfg.set_incr_read(false);
        dma2_cfg.set_incr_write(false);
        dma2_cfg.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        dma2_cfg.set_chain_to(dma3.number());
        dma2_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        dma2_cfg.set_en(true);
        p2.al1_ctrl().write_value(dma2_cfg.0);

        /* setup channel 3, which reads the data after it is collected and writes it to the final destination. It will be triggered by channel 2 */
        p3.trans_count().write(|w| {
            w.set_count(1);
        });
        p3.read_addr()
            .write_value(data_fetcher_read_target.rx_address_count().0 as u32);
        p3.write_addr()
            .write_value(write_target.tx_address_count().0 as u32);
        let mut dma3_cfg = pac::dma::regs::CtrlTrig(0);
        dma3_cfg.set_incr_read(false);
        dma3_cfg.set_incr_write(false);
        dma3_cfg.set_treq_sel(pac::dma::vals::TreqSel::from(
            data_fetcher_read_target
                .rx_treq()
                .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
        ));
        dma3_cfg.set_chain_to(dma0.number()); // trigger channel 0 again, so it can wait on the next request
        dma3_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_BYTE);
        dma3_cfg.set_en(true);
        p3.al1_ctrl().write_value(dma3_cfg.0);

        // setup the sniffer to sniff on channel 1 and use calculation method sum
        pac::DMA.sniff_ctrl().write(|w| {
            w.set_dmach(dma1.number());
            w.set_calc(pac::dma::vals::Calc::SUM);
            w.set_en(true);
        });

        // finally trigger channel 0
        pac::DMA.multi_chan_trigger().write(|w| {
            w.set_multi_chan_trigger(1 << dma0.number());
        });

        Self {
            _dma_ch0: dma0,
            _dma_ch1: dma1,
            _dma_ch2: dma2,
            _dma_ch3: dma3,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(C, packed(1))]
struct DmaCommand {
    read_addr: *const u32,
    write_addr: *const u32,
}

pub struct GbDmaCommandMachine<'d> {
    dma_ch_cmd_executor: Peri<'d, AnyChannel>,
    dma_ch_cmd_loader: Peri<'d, AnyChannel>,
    dma_ch_mem_accessor: Peri<'d, AnyChannel>,
    dma_ch_ram_read_requestor: Peri<'d, AnyChannel>,
    dma_ch_ram_write_requestor: Peri<'d, AnyChannel>,
    write_target_addr: u32,
    saveram_write_addr_read_target_addr: u32,
    ram_read_ctrl_reg: u32,
    ram_write_ctrl_reg: u32,
    ram_disabled_read_commands: [DmaCommand; 5],
    ram_read_commands: [DmaCommand; 5],
    rtc_read_commands: [DmaCommand; 5],
    ram_write_commands: [DmaCommand; 5],
    ram_disabled_write_commands: [DmaCommand; 5],
    current_ram_read_command: *const DmaCommand,
    current_ram_write_commands: *const DmaCommand,
    rtc_write_commands: [DmaCommand; 5],
}

impl<'d> GbDmaCommandMachine<'d> {
    pub fn new(
        dma_cmd_executor: Peri<'d, AnyChannel>,
        dma_cmd_loader: Peri<'d, AnyChannel>,
        dma_mem_accessor: Peri<'d, AnyChannel>,
        dma_ram_read_requestor: Peri<'d, AnyChannel>,
        dma_ram_write_requestor: Peri<'d, AnyChannel>,
    ) -> Self {
        Self {
            dma_ch_cmd_executor: dma_cmd_executor,
            dma_ch_cmd_loader: dma_cmd_loader,
            dma_ch_mem_accessor: dma_mem_accessor,
            dma_ch_ram_read_requestor: dma_ram_read_requestor,
            dma_ch_ram_write_requestor: dma_ram_write_requestor,
            write_target_addr: 0,
            saveram_write_addr_read_target_addr: 0,
            ram_read_ctrl_reg: 0,
            ram_write_ctrl_reg: 0,
            ram_read_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            ram_write_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            rtc_read_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            rtc_write_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            ram_disabled_read_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            ram_disabled_write_commands: [DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            }; 5],
            current_ram_read_command: core::ptr::null(),
            current_ram_write_commands: core::ptr::null(),
        }
    }

    pub fn init(
        &mut self,
        ram_read_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        ram_write_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        write_target: &dyn DmaWriteTarget<TransmittedWord = u8>,
        ram_base_addr_ptr: *const *mut u8,
        rtc_latch_ptr: *const *mut u8,
        rtc_real_ptr: *const *mut u8,
    ) {
        let cmd_executor_regs = self.dma_ch_cmd_executor.regs();
        let cmd_loader_regs = self.dma_ch_cmd_loader.regs();
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();
        let saveram_read_requestor_regs = self.dma_ch_ram_read_requestor.regs();
        let saveram_write_requestor_regs = self.dma_ch_ram_write_requestor.regs();

        self.write_target_addr = write_target.tx_address_count().0;
        self.saveram_write_addr_read_target_addr = ram_write_addr_read_target.rx_address_count().0;

        let mut ram_read_ctrl_reg = pac::dma::regs::CtrlTrig(0);
        ram_read_ctrl_reg.set_chain_to(self.dma_ch_ram_read_requestor.number());
        ram_read_ctrl_reg.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        ram_read_ctrl_reg.set_en(true);
        self.ram_read_ctrl_reg = ram_read_ctrl_reg.0;

        let mut ram_write_ctrl_reg = pac::dma::regs::CtrlTrig(0);
        ram_write_ctrl_reg.set_chain_to(self.dma_ch_ram_write_requestor.number());
        ram_write_ctrl_reg.set_treq_sel(pac::dma::vals::TreqSel::from(
            ram_write_addr_read_target
                .rx_treq()
                .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
        ));
        ram_write_ctrl_reg.set_en(true);
        self.ram_write_ctrl_reg = ram_write_ctrl_reg.0;

        self.ram_disabled_read_commands = self
            .create_ram_disabled_read_command_blocks(ram_read_addr_read_target, ram_base_addr_ptr);
        self.ram_read_commands =
            self.create_ram_read_command_blocks(ram_read_addr_read_target, ram_base_addr_ptr);
        self.rtc_read_commands =
            self.create_rtc_read_command_blocks(ram_read_addr_read_target, rtc_latch_ptr);
        self.current_ram_read_command = self.ram_read_commands.as_ptr();

        self.ram_disabled_write_commands =
            self.create_ram_disabled_write_command_blocks(ram_write_addr_read_target);
        self.ram_write_commands =
            self.create_ram_write_command_blocks(ram_write_addr_read_target, ram_base_addr_ptr);
        self.rtc_write_commands =
            self.create_rtc_write_command_blocks(ram_write_addr_read_target, rtc_real_ptr);
        self.current_ram_write_commands = self.ram_write_commands.as_ptr();

        /*
         * Setup the DMA which acts as the CMD_LOADER. It will be initially triggered
         * by one of the REQUESTOR DMAs. They are triggered by the PIO-SM DREQ and
         * will load the DmaCommand for the transaction into the the read_register of
         * this DMA. This DMA loads the current command list into the CMD_EXECUTOR and
         * triggers it. It will be triggered again by the CMD_EXECUTOR until all
         * commands are executed (NULL-entry).
         */
        cmd_loader_regs.trans_count().write(|w| {
            w.set_count(2); // Halt after each control block
        });
        cmd_loader_regs
            .write_addr()
            .write_value(cmd_executor_regs.al2_read_addr().as_ptr() as u32);
        let mut loader_ctrl = pac::dma::regs::CtrlTrig(0);
        loader_ctrl.set_incr_read(true);
        loader_ctrl.set_incr_write(true);
        loader_ctrl.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        loader_ctrl.set_ring_sel(true); // wrap on write addr
        loader_ctrl.set_ring_size(3); // wrap every 2 words (after each command)
        loader_ctrl.set_chain_to(self.dma_ch_cmd_loader.number()); // disable chain by setting it to itself
        loader_ctrl.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        loader_ctrl.set_en(true);
        cmd_loader_regs.al1_ctrl().write_value(loader_ctrl.0);

        /*
         * Setup the DMA which acts as the CMD_EXECUTOR: IT will receive it's commands
         * and triggers from CMD_LOADER. Only it's read and write address changes with
         * each command. Those are always 32-bit wide and only one word long.
         * CMD_EXECUTOR chains to CMD_LOADER to trigger the loading of the next
         * command.
         */
        cmd_executor_regs.trans_count().write(|w| {
            w.set_count(1);
        });
        let mut executor_ctrl = pac::dma::regs::CtrlTrig(0);
        executor_ctrl.set_incr_read(false);
        executor_ctrl.set_incr_write(false);
        executor_ctrl.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        executor_ctrl.set_chain_to(self.dma_ch_cmd_loader.number());
        executor_ctrl.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        executor_ctrl.set_en(true);
        cmd_executor_regs.al1_ctrl().write_value(executor_ctrl.0);

        /*
         * The DMA wich acts as MEMORY_ACCESOR is completely setup and trigger by the
         * CMD_EXECUTOR based on the command chain received from CMD_LOADER. It's only
         * necessary the transfer count here, as everything else is done by the
         * commands.
         */
        mem_accessor_regs.trans_count().write(|w| {
            w.set_count(1);
        });

        /*
         * Setup all the REQUESTOR DMAs for each PIO-SM the DMAs above serve. Each
         * REQUSTOR DMA is setup to wait for a DREQ from a PIO state machine. After it
         * gets it's DREQ it loads the command chain needed to serve this request into
         * the CMD_LOADER and triggers it. This starts the whole chain of commands
         * which will eventually lead to the serving of the necessary memory transfer
         * from and into the FIFOs of the state machine.
         */
        saveram_read_requestor_regs.trans_count().write(|w| {
            w.set_count(1);
        });
        saveram_read_requestor_regs
            .read_addr()
            .write_value(ptr::addr_of!(self.current_ram_read_command) as u32);
        saveram_read_requestor_regs
            .write_addr()
            .write_value(cmd_loader_regs.al3_read_addr_trig().as_ptr() as u32);
        saveram_read_requestor_regs.ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_treq_sel(pac::dma::vals::TreqSel::from(
                ram_read_addr_read_target
                    .rx_treq()
                    .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
            ));
            w.set_chain_to(self.dma_ch_ram_read_requestor.number()); // chain to self, to disable chain
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_en(true);
        });

        saveram_write_requestor_regs.trans_count().write(|w| {
            w.set_count(1);
        });
        saveram_write_requestor_regs
            .read_addr()
            .write_value(ptr::addr_of!(self.current_ram_write_commands) as u32);
        saveram_write_requestor_regs
            .write_addr()
            .write_value(cmd_loader_regs.al3_read_addr_trig().as_ptr() as u32);
        saveram_write_requestor_regs.ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_treq_sel(pac::dma::vals::TreqSel::from(
                ram_write_addr_read_target
                    .rx_treq()
                    .unwrap_or(pac::dma::vals::TreqSel::PERMANENT as u8),
            ));
            w.set_chain_to(self.dma_ch_ram_write_requestor.number()); // chain to self, to disable chain
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_en(true);
        });
    }

    fn create_ram_read_command_blocks(
        &self,
        ram_read_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        ram_base_addr_ptr: *const *mut u8,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // load the settings for this transfer into the MEMORY_ACCESOR_DMA control register
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_read_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // load the addr from the rx-fifo of the PIO-SM triggering this transfer into the write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: ptr::addr_of!(self.write_target_addr),
                write_addr: mem_accessor_regs.write_addr().as_ptr(),
            },
            // load the addr from the rx-fifo of the PIO-SM triggering this transfer
            DmaCommand {
                read_addr: ram_read_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: mem_accessor_regs.read_addr().as_ptr(),
            },
            // load the base addr, write it into the read-addr of the MEMORY_ACCESSOR_DMA, or-ing it with the addr received and trigger the MEMORY_ACCESSOR_DMA transfer
            DmaCommand {
                read_addr: ram_base_addr_ptr as *const u32,
                write_addr: (mem_accessor_regs.al3_read_addr_trig().as_ptr() as u32
                    | REG_ALIAS_SET_BITS) as *const u32,
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }

    fn create_ram_disabled_read_command_blocks(
        &self,
        ram_read_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        ram_base_addr_ptr: *const *mut u8,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // load the settings for this transfer into the MEMORY_ACCESOR_DMA control register
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_read_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // load the addr of devnull into the write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: ptr::addr_of!(DEV_NULL_PTR) as *const u32,
                write_addr: mem_accessor_regs.write_addr().as_ptr(),
            },
            // load the addr from the rx-fifo of the PIO-SM triggering this transfer
            DmaCommand {
                read_addr: ram_read_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: mem_accessor_regs.read_addr().as_ptr(),
            },
            // load the base addr, write it into the read-addr of the MEMORY_ACCESSOR_DMA, or-ing it with the addr received and trigger the MEMORY_ACCESSOR_DMA transfer
            DmaCommand {
                read_addr: ram_base_addr_ptr as *const u32,
                write_addr: (mem_accessor_regs.al3_read_addr_trig().as_ptr() as u32
                    | REG_ALIAS_SET_BITS) as *const u32,
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }

    fn create_ram_write_command_blocks(
        &self,
        ram_write_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        ram_base_addr_ptr: *const *mut u8,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // setup MEMORY_ACCESSOR_DMA for this write to RAM transaction
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_write_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // load the addr from the rx-fifo of the PIO-SM triggering this transfer into the write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: ram_write_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: mem_accessor_regs.write_addr().as_ptr(),
            },
            // load the base addr into the write addr of MEMORY_ACCESSOR_DMA, or-ing it with the addr already there (received from rx-fifo)
            DmaCommand {
                read_addr: ram_base_addr_ptr as *const u32,
                write_addr: (mem_accessor_regs.write_addr().as_ptr() as u32 | REG_ALIAS_SET_BITS)
                    as *const u32,
            },
            // load the addr of the rx-fifo which will have the data to be written to RAM into the read register of MEMORY_ACCESSOR_DMA and trigger it's transfer
            DmaCommand {
                read_addr: ptr::addr_of!(self.saveram_write_addr_read_target_addr),
                write_addr: mem_accessor_regs.al3_read_addr_trig().as_ptr(),
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }

    fn create_ram_disabled_write_command_blocks(
        &self,
        ram_write_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // setup MEMORY_ACCESSOR_DMA for this write to RAM transaction
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_write_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // dummy load the addr from the rx-fifo of the PIO-SM triggering this transfer
            DmaCommand {
                read_addr: ram_write_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: ptr::addr_of!(DEV_NULL) as *const u32,
            },
            // load the addr of _devNull into write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: ptr::addr_of!(DEV_NULL_PTR) as *const u32,
                write_addr: (mem_accessor_regs.write_addr().as_ptr() as u32 | REG_ALIAS_SET_BITS)
                    as *const u32,
            },
            // load the addr of the rx-fifo which will have the data to be written to RAM into the read register of MEMORY_ACCESSOR_DMA and trigger it's transfer
            DmaCommand {
                read_addr: ptr::addr_of!(self.saveram_write_addr_read_target_addr),
                write_addr: mem_accessor_regs.al3_read_addr_trig().as_ptr(),
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }

    fn create_rtc_read_command_blocks(
        &self,
        ram_read_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        rtc_latch_ptr: *const *mut u8,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // load the settings for this transfer into the MEMORY_ACCESOR_DMA control register
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_read_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // load the addr from the rx-fifo of the PIO-SM triggering this transfer into the write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: ptr::addr_of!(self.write_target_addr),
                write_addr: mem_accessor_regs.write_addr().as_ptr(),
            },
            // dummy read the addr from the rx-fifo of the PIO-SM triggering this transfer
            DmaCommand {
                read_addr: ram_read_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: ptr::addr_of!(DEV_NULL),
            },
            // load the base addr, write it into the read-addr of the MEMORY_ACCESSOR_DMA, or-ing it with the addr received and trigger the MEMORY_ACCESSOR_DMA transfer
            DmaCommand {
                read_addr: rtc_latch_ptr as *const u32,
                write_addr: mem_accessor_regs.al3_read_addr_trig().as_ptr(),
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }

    fn create_rtc_write_command_blocks(
        &self,
        ram_write_addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        rtc_real_ptr: *const *mut u8,
    ) -> [DmaCommand; 5] {
        let mem_accessor_regs = self.dma_ch_mem_accessor.regs();

        [
            // setup MEMORY_ACCESSOR_DMA for this write to RAM transaction
            DmaCommand {
                read_addr: ptr::addr_of!(self.ram_write_ctrl_reg),
                write_addr: mem_accessor_regs.al1_ctrl().as_ptr(),
            },
            // dummy load the addr from the rx-fifo of the PIO-SM triggering this transfer
            DmaCommand {
                read_addr: ram_write_addr_read_target.rx_address_count().0 as *const u32,
                write_addr: ptr::addr_of!(DEV_NULL) as *const u32,
            },
            // load the current rtc register addr into the write addr of MEMORY_ACCESSOR_DMA
            DmaCommand {
                read_addr: rtc_real_ptr as *const u32,
                write_addr: mem_accessor_regs.write_addr().as_ptr(),
            },
            // load the addr of the rx-fifo which will have the data to be written to RAM into the read register of MEMORY_ACCESSOR_DMA and trigger it's transfer
            DmaCommand {
                read_addr: ptr::addr_of!(self.saveram_write_addr_read_target_addr),
                write_addr: mem_accessor_regs.al3_read_addr_trig().as_ptr(),
            },
            DmaCommand {
                read_addr: core::ptr::null(),
                write_addr: core::ptr::null(),
            },
        ]
    }
}

impl<'d> MbcRamControl for GbDmaCommandMachine<'d> {
    fn enable_ram_access(&mut self) {
        self.current_ram_read_command = self.ram_read_commands.as_ptr();
        self.current_ram_write_commands = self.ram_write_commands.as_ptr();
    }

    fn disable_ram_access(&mut self) {
        self.current_ram_read_command = self.ram_disabled_read_commands.as_ptr();
        self.current_ram_write_commands = self.ram_disabled_write_commands.as_ptr();
    }

    fn enable_rtc_access(&mut self) {
        self.current_ram_read_command = self.rtc_read_commands.as_ptr();
        self.current_ram_write_commands = self.rtc_write_commands.as_ptr();
    }
}
