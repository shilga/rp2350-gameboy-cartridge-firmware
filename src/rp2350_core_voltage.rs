#![allow(unused)]

use embassy_rp::pac;

pub enum VregVoltage {
    VregVoltage0_55 = 0b00000,
    VregVoltage0_60 = 0b00001,
    VregVoltage0_65 = 0b00010,
    VregVoltage0_70 = 0b00011,
    VregVoltage0_75 = 0b00100,
    VregVoltage0_80 = 0b00101,
    VregVoltage0_85 = 0b00110,
    VregVoltage0_90 = 0b00111,
    VregVoltage0_95 = 0b01000,
    VregVoltage1_00 = 0b01001,
    VregVoltage1_05 = 0b01010,
    VregVoltage1_10 = 0b01011,
    VregVoltage1_15 = 0b01100,
    VregVoltage1_20 = 0b01101,
    VregVoltage1_25 = 0b01110,
    VregVoltage1_30 = 0b01111,
}

const POWMAN_PASSWORD_BITS: u32 = 0x5afe0000u32;

pub fn vreg_set_voltage(voltage: VregVoltage) {
    pac::POWMAN.vreg_ctrl().modify(|w| {
        w.0 |= POWMAN_PASSWORD_BITS;
        w.set_unlock(true);
    });

    while pac::POWMAN.vreg().read().update_in_progress() {}

    pac::POWMAN.vreg().modify(|w| {
        w.0 |= POWMAN_PASSWORD_BITS;
        w.set_vsel(voltage as u8);
    });

    while pac::POWMAN.vreg().read().update_in_progress() {}
}
