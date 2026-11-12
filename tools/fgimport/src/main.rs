//! fgimport — convert a FlightGear aircraft model into Skyward's runtime
//! aircraft asset (`model.skac` + textures + `fdm.json`).
//!
//!     fgimport <config.json> <source-root> <out-dir>
//!
//! See `tools/fgimport/README.md` for the config format and the pipeline.

mod ac3d;
mod expr;
mod fdm;
mod scene;
mod xml;

use std::path::PathBuf;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 3 {
        eprintln!("usage: fgimport <config.json> <source-root> <out-dir>");
        std::process::exit(2);
    }
    let cfg_text = std::fs::read_to_string(&args[0]).expect("config");
    let cfg: serde_json::Value = serde_json::from_str(&cfg_text).expect("config json");
    let src = PathBuf::from(&args[1]);
    let out = PathBuf::from(&args[2]);
    std::fs::create_dir_all(&out).expect("out dir");
    if let Err(e) = scene::run(&cfg, &src, &out) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}
