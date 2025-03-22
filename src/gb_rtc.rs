use core::ptr;
use embassy_time::{Duration, Instant};

use crate::gb_mbc::MbcRtcControl;

static REGISTER_MASKS: [u8; 5] = [0x3fu8, 0x3fu8, 0x1fu8, 0xffu8, 0xc1u8];

#[derive(Clone, Copy)]
#[repr(C, packed(1))]
struct RtcRegs {
    seconds: u8,
    minutes: u8,
    hours: u8,
    days: u8,
    status: u8,
}

impl RtcRegs {
    fn is_halt(&self) -> bool {
        self.status & 0x40u8 == 0x40u8
    }

    fn is_days_high(&self) -> bool {
        self.status & 0x01u8 == 0x01u8
    }

    fn toggle_days_high(&mut self) {
        self.status ^= 0x01u8;
    }

    fn set_days_carry(&mut self) {
        self.status |= 0x80u8;
    }
}

union GbRtcRegisters {
    regs: RtcRegs,
    as_array: [u8; 5],
}

pub struct GbRtc {
    real: GbRtcRegisters,
    latch: GbRtcRegisters,
    last_milli: Instant,
    millies: u32,
    old_halt: bool,
    latch_ptr: *mut u8,
    real_ptr: *mut u8,
}

impl GbRtc {
    pub fn new() -> Self {
        GbRtc {
            real: GbRtcRegisters { as_array: [0u8; 5] },
            latch: GbRtcRegisters { as_array: [0u8; 5] },
            last_milli: Instant::from_micros(0),
            millies: 0,
            old_halt: false,
            latch_ptr: core::ptr::null::<u8>() as *mut u8,
            real_ptr: core::ptr::null::<u8>() as *mut u8,
        }
    }

    fn process_tick(&mut self) {
        {
            let regs = unsafe { &mut self.real.regs };

            regs.seconds += 1;

            if regs.seconds == 60 {
                regs.seconds = 0;
                regs.minutes += 1;

                if regs.minutes == 60 {
                    regs.minutes = 0;
                    regs.hours += 1;

                    if regs.hours == 24 {
                        regs.hours = 0;
                        regs.days += 1;

                        if regs.days == 0 {
                            if regs.is_days_high() {
                                regs.set_days_carry();
                            }
                            regs.toggle_days_high();
                        }
                    }
                }
            }
        }
    }

    pub fn get_real_ptr(&mut self) -> *const *mut u8 {
        if self.real_ptr.is_null() {
            self.real_ptr = unsafe { self.real.as_array }.as_mut_ptr();
        }
        ptr::addr_of!(self.real_ptr)
    }

    pub fn get_latch_ptr(&mut self) -> *const *mut u8 {
        if self.latch_ptr.is_null() {
            self.latch_ptr = unsafe { self.latch.as_array }.as_mut_ptr();
        }
        ptr::addr_of!(self.latch_ptr)
    }
}

impl MbcRtcControl for GbRtc {
    fn process(&mut self) {
        let now = Instant::now();

        if self.old_halt && !unsafe { self.real.regs }.is_halt() {
            self.last_milli = now;
        }
        self.old_halt = unsafe { self.real.regs }.is_halt();

        if !unsafe { self.real.regs }.is_halt() {
            if now.duration_since(self.last_milli).as_micros() > 1000u64 {
                self.last_milli = self
                    .last_milli
                    .checked_add(Duration::from_micros(1000u64))
                    .unwrap();
                self.millies += 1;
            }

            if self.millies >= 1000u32 {
                self.millies = 0;
                self.process_tick();
            }
        }

        // todo: should we really process this constantly?
        let reg_array = unsafe { &mut self.real.as_array };
        for n in 0..REGISTER_MASKS.len() {
            reg_array[n] &= REGISTER_MASKS[n];
        }
    }

    fn trigger_latch(&mut self) {
        unsafe {
            self.latch.regs = self.real.regs;
        }
    }

    fn set_register(&mut self, reg_num: u8) {
        let reg_num: usize = reg_num as usize;
        if reg_num < REGISTER_MASKS.len() {
            unsafe {
                ptr::write_volatile(
                    ptr::addr_of_mut!(self.latch_ptr),
                    self.latch.as_array.as_mut_ptr().add(reg_num),
                );
                ptr::write_volatile(
                    ptr::addr_of_mut!(self.real_ptr),
                    self.real.as_array.as_mut_ptr().add(reg_num),
                );
            }
        }
    }
}
