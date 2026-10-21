// Copyright (c) 2026 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
