use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::pac;
use embassy_rp::{into_ref, Peripheral, PeripheralRef};

const REG_ALIAS_SET_BITS: u32 = 0x2u32 << 12u32;

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
        read_base_addr_ptr: *mut *mut u32,
        read_addr_rx_fifo: *mut u32,
        write_to_data_tx_fifo: *mut u32,
    ) -> Self {
        into_ref!(dma0);
        into_ref!(dma1);
        into_ref!(dma2);

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
        p1.write_addr().write_value(write_to_data_tx_fifo as u32);
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
        p0.read_addr().write_value(read_addr_rx_fifo as u32);
        p0.write_addr().write_value(p1.read_addr().as_ptr() as u32);
        p0.ctrl_trig().write(|w: &mut pac::dma::regs::CtrlTrig| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_treq_sel(pac::dma::vals::TreqSel::PERMANENT); // TODO
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
        p2.ctrl_trig().write(|w: &mut pac::dma::regs::CtrlTrig| {
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
