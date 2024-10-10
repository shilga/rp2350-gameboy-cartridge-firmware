use core::ptr;

use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::pac;
use embassy_rp::{into_ref, Peripheral, PeripheralRef};

use defmt::info;

use crate::dma_helper::{DmaReadTarget, DmaWriteTarget};

const REG_ALIAS_SET_BITS: u32 = 0x2u32 << 12u32;
static mut DEV_NULL: u32 = 0;

pub struct GbReadDmaConfig<'d> {
    _dma_ch0: PeripheralRef<'d, AnyChannel>,
    _dma_ch1: PeripheralRef<'d, AnyChannel>,
    _dma_ch2: PeripheralRef<'d, AnyChannel>,
}

impl<'d> GbReadDmaConfig<'d> {
    pub fn new(
        dma0: impl Peripheral<P = impl Channel> + 'd,
        dma1: impl Peripheral<P = impl Channel> + 'd,
        dma2: impl Peripheral<P = impl Channel> + 'd,
        read_base_addr_ptr: *mut *mut u8,
        addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        write_target: &dyn DmaWriteTarget<TransmittedWord = u8>,
    ) -> Self {
        into_ref!(dma0);
        into_ref!(dma1);
        into_ref!(dma2);

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

        let dma_ch0: PeripheralRef<'d, AnyChannel> = dma0.map_into();
        let dma_ch1: PeripheralRef<'d, AnyChannel> = dma1.map_into();
        let dma_ch2: PeripheralRef<'d, AnyChannel> = dma2.map_into();

        let p0 = dma_ch0.regs();
        let p1 = dma_ch1.regs();
        let p2 = dma_ch2.regs();

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
        dma1_cfg.set_chain_to(dma_ch0.number());
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
            w.set_chain_to(dma_ch2.number());
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
            w.set_chain_to(dma_ch2.number()); // chain to itself -> disable
            w.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
            w.set_en(true);
        });

        Self {
            _dma_ch0: dma_ch0,
            _dma_ch1: dma_ch1,
            _dma_ch2: dma_ch2,
        }
    }
}

pub struct GbReadSniffDmaConfig<'d> {
    _dma_ch0: PeripheralRef<'d, AnyChannel>,
    _dma_ch1: PeripheralRef<'d, AnyChannel>,
    _dma_ch2: PeripheralRef<'d, AnyChannel>,
    _dma_ch3: PeripheralRef<'d, AnyChannel>,
}

impl<'d> GbReadSniffDmaConfig<'d> {
    pub fn new(
        dma0: impl Peripheral<P = impl Channel> + 'd,
        dma1: impl Peripheral<P = impl Channel> + 'd,
        dma2: impl Peripheral<P = impl Channel> + 'd,
        dma3: impl Peripheral<P = impl Channel> + 'd,
        addr_read_target: &dyn DmaReadTarget<ReceivedWord = u32>,
        data_fetcher_write_target: &dyn DmaWriteTarget<TransmittedWord = u32>,
        data_fetcher_read_target: &dyn DmaReadTarget<ReceivedWord = u8>,
        write_target: &dyn DmaWriteTarget<TransmittedWord = u8>,
        base_addr_ptr: *const u32,
    ) -> Self {
        into_ref!(dma0);
        into_ref!(dma1);
        into_ref!(dma2);
        into_ref!(dma3);

        let dma_ch0: PeripheralRef<'d, AnyChannel> = dma0.map_into();
        let dma_ch1: PeripheralRef<'d, AnyChannel> = dma1.map_into();
        let dma_ch2: PeripheralRef<'d, AnyChannel> = dma2.map_into();
        let dma_ch3: PeripheralRef<'d, AnyChannel> = dma3.map_into();

        let p0 = dma_ch0.regs();
        let p1 = dma_ch1.regs();
        let p2 = dma_ch2.regs();
        let p3 = dma_ch3.regs();

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
        dma0_cfg.set_chain_to(dma_ch1.number());
        dma0_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_WORD);
        dma0_cfg.set_en(true);
        p0.al1_ctrl().write_value(dma0_cfg.0);

        /* setup channel 1, which reads the base addr and does a dummy transfer. This will be catched by the sniffer. It will be triggered by channel 0 */
        p1.trans_count().write(|w| {
            w.set_count(1);
        });
        p1.read_addr().write_value(base_addr_ptr as u32);
        p1.write_addr()
            .write_value(unsafe { ptr::addr_of_mut!(DEV_NULL) } as u32);
        let mut dma1_cfg = pac::dma::regs::CtrlTrig(0);
        dma1_cfg.set_incr_read(false);
        dma1_cfg.set_incr_write(false);
        dma1_cfg.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT);
        dma1_cfg.set_chain_to(dma_ch2.number());
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
        dma2_cfg.set_chain_to(dma_ch3.number());
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
        dma3_cfg.set_chain_to(dma_ch0.number()); // trigger channel 0 again, so it can wait on the next request
        dma3_cfg.set_data_size(pac::dma::vals::DataSize::SIZE_BYTE);
        dma3_cfg.set_en(true);
        p3.al1_ctrl().write_value(dma3_cfg.0);

        // setup the sniffer to sniff on channel 1 and use calculation method sum
        pac::DMA.sniff_ctrl().write(|w| {
            w.set_dmach(dma_ch1.number());
            w.set_calc(pac::dma::vals::Calc::SUM);
            w.set_en(true);
        });

        // finally trigger channel 0
        pac::DMA.multi_chan_trigger().write(|w| {
            w.set_multi_chan_trigger(1 << dma_ch0.number());
        });

        Self {
            _dma_ch0: dma_ch0,
            _dma_ch1: dma_ch1,
            _dma_ch2: dma_ch2,
            _dma_ch3: dma_ch3,
        }
    }
}
