//! Set up linker scripts for the rp235x-hal examples

use std::fs::{self, File};
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

    // check if env vars for version are correct
    u8::from_str_radix(std::env::var("VERSION_MAJOR").unwrap().as_str(), 10).expect("VERSION_MAJOR is not an u8");
    u8::from_str_radix(std::env::var("VERSION_MINOR").unwrap().as_str(), 10).expect("VERSION_MAJOR is not an u8");
    u8::from_str_radix(std::env::var("VERSION_PATCH").unwrap().as_str(), 10).expect("VERSION_MAJOR is not an u8");
    assert!(std::env::var("RELEASE_TYPE").unwrap().len() == 1);

    println!("cargo::rerun-if-env-changed=VERSION_MAJOR");
    println!("cargo::rerun-if-env-changed=VERSION_MINOR");
    println!("cargo::rerun-if-env-changed=VERSION_PATCH");
    println!("cargo::rerun-if-env-changed=RELEASE_TYPE");


    built::write_built_file().expect("Failed to acquire build-time information");

    // Make sure we get rerun when the git commit changes.
    // We want to watch two files: HEAD, which tracks which branch we are on,
    // and the file for that branch that tracks which commit is is on.
    let git_head_file = PathBuf::from(".git/HEAD");
    if git_head_file.exists() {
        println!("cargo::rerun-if-changed={}", git_head_file.display());

        let git_head_ref = fs::read_to_string(git_head_file).expect("Unable to read HEAD ref");

        let v: Vec<&str> = git_head_ref.trim().split("ref: ").collect();
        if v[0] == "" {
            // this is only true if HEAD begins with "ref:", if false HEAD is detached
            let git_head_ref = v[1];

            let git_head_ref_file = PathBuf::from(".git").join(git_head_ref);
            if git_head_ref_file.exists() {
                println!("cargo::rerun-if-changed={}", git_head_ref_file.display());
            }
        }
    }
}
