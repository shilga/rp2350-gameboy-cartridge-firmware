//! Set up linker scripts for the rp235x-hal examples

use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    // Put the linker script somewhere the linker can find it
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    println!("cargo:rustc-link-search={}", out.display());

    let work_dir = PathBuf::from(std::env::current_dir().unwrap());
    let gb_bootloader_dir = work_dir.join("gb-bootloader");

    let mut gbdk_lcc = PathBuf::from(std::env::var_os("GBDK_PATH").unwrap());
    gbdk_lcc.push("bin/lcc");

    let lcc_status = Command::new(gbdk_lcc)
        .args(&[
            "-Wa-l", "-Wl-m", "-Wl-j", "-Wm-p", "-Wm-yc", "-Wm-yt2", "-Wm-ya1", "-o",
        ])
        .arg(out.join("bootloader.gb"))
        .arg(gb_bootloader_dir.join("bootloader.c"))
        .arg(gb_bootloader_dir.join("giraffe_4color_data.c"))
        .arg(gb_bootloader_dir.join("giraffe_4color_map.c"))
        .status()
        .expect("failed to execute lcc");

    assert!(lcc_status.success());

    println!("cargo:rerun-if-changed=gb-bootloader/bootloader.c");

    // The file `memory.x` is loaded by cortex-m-rt's `link.x` script, which
    // is what we specify in `.cargo/config.toml` for Arm builds
    let memory_x = include_bytes!("memory.x");
    let mut f = File::create(out.join("memory.x")).unwrap();
    f.write_all(memory_x).unwrap();
    println!("cargo:rerun-if-changed=memory.x");

    // File::create(out.join("mylink.x"))
    //     .unwrap()
    //     .write_all(include_bytes!("mylink.x"))
    //     .unwrap();
    // println!("cargo:rustc-link-search={}", out.display());
    // println!("cargo:rerun-if-changed=build.rs");
    // println!("cargo:rerun-if-changed=mylink.x");

    // // The file `rp235x_riscv.x` is what we specify in `.cargo/config.toml` for
    // // RISC-V builds
    // let rp235x_riscv_x = include_bytes!("rp235x_riscv.x");
    // let mut f = File::create(out.join("rp235x_riscv.x")).unwrap();
    // f.write_all(rp235x_riscv_x).unwrap();
    // println!("cargo:rerun-if-changed=rp235x_riscv.x");

    println!("cargo:rerun-if-changed=build.rs");
}
