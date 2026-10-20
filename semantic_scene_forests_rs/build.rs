use std::{env, fs, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    // cargo-ament-build installs Rust sources in share/<package>/rust, native files beside it.
    let native_source = if manifest.join("cpp").is_dir() {
        manifest.clone()
    } else {
        manifest.parent().unwrap().to_path_buf()
    };
    cxx_build::bridge("src/ffi.rs")
        .include(native_source.join("include"))
        .std("c++23")
        .compile("ssf_rs_glue");

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let native = cmake::Config::new(native_source.join("cpp"))
        .define("CXXBRIDGE_INCLUDE_DIR", out.join("cxxbridge/include"))
        .define("CMAKE_EXPORT_COMPILE_COMMANDS", "ON")
        .build();
    println!("cargo:rustc-link-search=native={}/lib", native.display());
    println!("cargo:rustc-link-lib=static=ssf_rs_adapter");
    print!(
        "{}",
        fs::read_to_string(native.join("cargo-links.txt")).unwrap()
    );

    for path in [
        manifest.join("src/ffi.rs"),
        native_source.join("include/semantic_scene_forests_rs/bridge.hpp"),
        native_source.join("cpp"),
    ] {
        println!("cargo:rerun-if-changed={}", path.display());
    }
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
}
