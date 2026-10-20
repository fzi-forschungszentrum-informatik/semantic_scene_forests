use std::{env, fs, path::PathBuf};

fn main() {
    cxx_build::bridge("src/ffi.rs")
        .include("include")
        .std("c++23")
        .compile("ssf_rs_glue");

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let native = cmake::Config::new("cpp")
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
        "src/ffi.rs",
        "include/semantic_scene_forests_rs/bridge.hpp",
        "cpp",
    ] {
        println!("cargo:rerun-if-changed={path}");
    }
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
}
