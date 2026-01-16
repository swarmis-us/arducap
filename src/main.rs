use std::env;

use anyhow::Result;
use arducap::pipeline::process_ardupilot_file;

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: {} <logfile.bin>... ", args[0]);
        return Ok(());
    }

    for filename in &args[1..] {
        process_ardupilot_file(filename)?;
    }

    Ok(())
}
