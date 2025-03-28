use core::cell::RefCell;
use core::ptr;
use embassy_sync::blocking_mutex::{raw::RawMutex, Mutex};
use embassy_time::{Duration, Instant};

use crate::gb_mbc::MbcRtcControl;
use crate::gb_savefile::GbRtcSaveStateProvider;

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

union GbRtcRegisterInstance {
    regs: RtcRegs,
    as_array: [u8; 5],
}

pub struct GbcRtcRegisters {
    real: GbRtcRegisterInstance,
    latch: GbRtcRegisterInstance,
}

impl GbcRtcRegisters {
    pub fn new() -> Self {
        GbcRtcRegisters {
            real: GbRtcRegisterInstance { as_array: [0u8; 5] },
            latch: GbRtcRegisterInstance { as_array: [0u8; 5] },
        }
    }
}

pub struct GbRtc<'a, M>
where
    M: RawMutex,
{
    registers: &'a Mutex<M, RefCell<GbcRtcRegisters>>,
    last_milli: Instant,
    millies: u32,
    old_halt: bool,
    latch_ptr: *mut u8,
    real_ptr: *mut u8,
}

impl<'a, M> GbRtc<'a, M>
where
    M: RawMutex,
{
    pub fn new(registers: &'a Mutex<M, RefCell<GbcRtcRegisters>>) -> Self {
        GbRtc {
            registers,
            last_milli: Instant::from_micros(0),
            millies: 0,
            old_halt: false,
            latch_ptr: core::ptr::null::<u8>() as *mut u8,
            real_ptr: core::ptr::null::<u8>() as *mut u8,
        }
    }

    fn process_tick(regs: &mut RtcRegs) {
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

    pub fn get_real_ptr(&mut self) -> *const *mut u8 {
        if self.real_ptr.is_null() {
            self.registers.lock(|registers| {
                let registers = registers.borrow_mut();
                let regs = &mut unsafe { registers.real.as_array };
                self.real_ptr = regs.as_mut_ptr();
            });
        }
        ptr::addr_of!(self.real_ptr)
    }

    pub fn get_latch_ptr(&mut self) -> *const *mut u8 {
        if self.latch_ptr.is_null() {
            self.registers.lock(|registers| {
                let registers = registers.borrow_mut();
                let regs = &mut unsafe { registers.latch.as_array };
                self.real_ptr = regs.as_mut_ptr();
            });
        }
        ptr::addr_of!(self.latch_ptr)
    }
}

impl<'a, M> MbcRtcControl for GbRtc<'a, M>
where
    M: RawMutex,
{
    fn process(&mut self) {
        // lock for the full processing time. It's important this is done without interruption.
        // The only other consumer is the savegame storing process. That can wait until this is done.
        self.registers.lock(|registers| {
            let mut registers = registers.borrow_mut();
            let regs = unsafe { &mut registers.real.regs };

            let now = Instant::now();

            if self.old_halt && !regs.is_halt() {
                self.last_milli = now;
            }
            self.old_halt = regs.is_halt();

            if !regs.is_halt() {
                if now.duration_since(self.last_milli).as_micros() > 1000u64 {
                    self.last_milli = self
                        .last_milli
                        .checked_add(Duration::from_micros(1000u64))
                        .unwrap();
                    self.millies += 1;
                }

                if self.millies >= 1000u32 {
                    self.millies = 0;
                    GbRtc::<M>::process_tick(regs);
                }
            }

            // todo: should this really be processed constantly?
            let reg_array = unsafe { &mut registers.real.as_array };
            for n in 0..REGISTER_MASKS.len() {
                reg_array[n] &= REGISTER_MASKS[n];
            }
        });
    }

    fn trigger_latch(&mut self) {
        self.registers.lock(|registers| {
            let mut registers = registers.borrow_mut();

            unsafe {
                registers.latch.regs = registers.real.regs;
            }
        });
    }

    fn activate_register(&mut self, reg_num: u8) {
        let reg_num: usize = reg_num as usize;
        if reg_num < REGISTER_MASKS.len() {
            self.registers.lock(|registers| {
                let mut registers = registers.borrow_mut();

                unsafe {
                    ptr::write_volatile(
                        ptr::addr_of_mut!(self.latch_ptr),
                        registers.latch.as_array.as_mut_ptr().add(reg_num),
                    );
                    ptr::write_volatile(
                        ptr::addr_of_mut!(self.real_ptr),
                        registers.real.as_array.as_mut_ptr().add(reg_num),
                    );
                }
            });
        }
    }
}

pub struct GbRtcStateProvider<'a, M>
where
    M: RawMutex,
{
    registers: &'a Mutex<M, RefCell<GbcRtcRegisters>>,
}

impl<'a, M> GbRtcStateProvider<'a, M>
where
    M: RawMutex,
{
    pub fn new(registers: &'a Mutex<M, RefCell<GbcRtcRegisters>>) -> Self {
        Self { registers }
    }
}

impl<'a, M> GbRtcSaveStateProvider for GbRtcStateProvider<'a, M>
where
    M: RawMutex,
{
    fn retrieve_register_state(&self) -> ([u8; 5], [u8; 5]) {
        let mut real = [0u8; 5];
        let mut latch = [0u8; 5];

        self.registers.lock(|registers| {
            let registers = registers.borrow();
            real.copy_from_slice(unsafe { registers.real.as_array }.as_slice());
            latch.copy_from_slice(unsafe { registers.latch.as_array }.as_slice());
        });

        (real, latch)
    }

    fn restore_register_state(&mut self, regs: ([u8; 5], [u8; 5])) {
        self.registers.lock(|registers| {
            let mut registers = registers.borrow_mut();
            unsafe { &mut registers.real.as_array }.copy_from_slice(&regs.0);
            unsafe { &mut registers.latch.as_array }.copy_from_slice(&regs.1);
        });
    }

    fn advance_by_seconds(&mut self, seconds: u64) {
        let mut seconds = seconds;
        self.registers.lock(|registers| {
            let mut registers = registers.borrow_mut();
            let regs = unsafe { &mut registers.real.regs };
            if !regs.is_halt() {
                while seconds > 0 {
                    GbRtc::<M>::process_tick(regs);
                    seconds -= 1;
                }
            }
        });
    }
}
