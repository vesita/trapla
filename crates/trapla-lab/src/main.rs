//! `trapla-lab` 可执行入口。

use std::process::ExitCode;

fn main() -> ExitCode {
    trapla_lab::cli::main()
}
