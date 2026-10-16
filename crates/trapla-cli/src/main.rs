//! trapla 命令行入口。
//!
//! ```text
//! trapla plan --map <csv> --start <x,y> --goal <x,y> --out <csv> [--seed <u64>] [--metrics <json>] [--repeat N]
//! trapla analyze --path <csv> --goal <x,y> [--out <json>]
//! trapla version
//! ```
//!
//! 约定：**结果摘要走 stdout，进度与诊断走 stderr**；退出码见 [`exit_code_table`]，
//! 由 `--help`（顶层与 `plan`）与 [`EXIT_INPUT`] 等常量**同源**给出。

mod analyze;
mod plan;
mod report;

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{CommandFactory, FromArgMatches, Parser, Subcommand};

/// 退出码：成功。
pub(crate) const EXIT_OK: u8 = 0;
/// 退出码：输入文件不可读或格式错（也含命令行参数错误）。
pub(crate) const EXIT_INPUT: u8 = 1;
/// 退出码：规划失败（无路径）。
pub(crate) const EXIT_NO_PATH: u8 = 2;
/// 退出码：其它内部错误（含输出文件写不出）。
pub(crate) const EXIT_INTERNAL: u8 = 3;

/// `--seed` 未给出时的缺省种子。
pub(crate) const DEFAULT_SEED: u64 = 0;

/// 退出码约定文本：`--help` 的 `after_help` 与代码里的常量共用这一份。
fn exit_code_table() -> String {
    format!(
        "退出码:\n  \
         {EXIT_OK} 成功\n  \
         {EXIT_INPUT} 输入文件不可读或格式错（参数错误同此码）\n  \
         {EXIT_NO_PATH} 规划失败（无路径）\n  \
         {EXIT_INTERNAL} 其它内部错误"
    )
}

#[derive(Debug, Parser)]
#[command(name = "trapla", about = "双足机器人落足点规划器（Rust 重写）")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// 规划一条落足路径并写出点位 CSV
    Plan(PlanArgs),
    /// 分析已有路径的猫步指标
    Analyze(AnalyzeArgs),
    /// 打印版本号
    Version,
}

#[derive(Debug, clap::Args)]
struct PlanArgs {
    /// 输入地图 CSV（无表头纯数值；显式路径，不按 exe 位置拼接）
    #[arg(long, value_name = "csv")]
    map: PathBuf,
    /// 起点，格式 x,y
    #[arg(long, value_name = "x,y")]
    start: Xy,
    /// 终点，格式 x,y
    #[arg(long, value_name = "x,y")]
    goal: Xy,
    /// 输出点位 CSV（表头 "x,y"，坐标固定 6 位小数）
    #[arg(long, value_name = "csv")]
    out: PathBuf,
    /// 随机种子（缺省 0）；相同 seed 的输出逐字节可复现
    #[arg(long, value_name = "u64")]
    seed: Option<u64>,
    /// 指标 JSON 输出路径；N=1 写对象，N>1 写数组（每项带 seed）
    #[arg(long, value_name = "json")]
    metrics: Option<PathBuf>,
    /// 重复规划次数 N（缺省 1）：依次用 seed, seed+1, ..., seed+N-1 各跑一次；
    /// --out 写第一次的路径，--metrics 在 N>1 时写长度 N 的数组
    #[arg(long, value_name = "N")]
    repeat: Option<usize>,
}

#[derive(Debug, clap::Args)]
struct AnalyzeArgs {
    /// 待分析的路径 CSV（表头 "x,y"）
    #[arg(long, value_name = "csv")]
    path: PathBuf,
    /// 终点，格式 x,y（用于 goal_error）
    #[arg(long, value_name = "x,y")]
    goal: Xy,
    /// 指标 JSON 输出路径；缺省只把摘要打到 stdout
    #[arg(long, value_name = "json")]
    out: Option<PathBuf>,
}

/// 命令行上的 `x,y` 点。
#[derive(Debug, Clone, Copy)]
struct Xy {
    x: f64,
    y: f64,
}

impl std::str::FromStr for Xy {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (x, y) = s
            .split_once(',')
            .ok_or_else(|| format!("期望 `x,y` 格式，收到 `{s}`"))?;
        let x = x
            .trim()
            .parse::<f64>()
            .map_err(|e| format!("x 解析失败: {e}"))?;
        let y = y
            .trim()
            .parse::<f64>()
            .map_err(|e| format!("y 解析失败: {e}"))?;
        Ok(Self { x, y })
    }
}

fn main() -> ExitCode {
    let help = exit_code_table();
    let command = Cli::command()
        .after_help(help.clone())
        .mut_subcommand("plan", |plan| plan.after_help(help));

    let matches = match command.try_get_matches() {
        Ok(matches) => matches,
        Err(error) => {
            // clap 自己处理 --help/--version（走 stdout，退 0）；其余用法错误按输入错误退 1。
            let code = if error.use_stderr() {
                EXIT_INPUT
            } else {
                EXIT_OK
            };
            let _ = error.print();
            return ExitCode::from(code);
        }
    };

    let cli = match Cli::from_arg_matches(&matches) {
        Ok(cli) => cli,
        Err(error) => {
            let _ = error.print();
            return ExitCode::from(EXIT_INPUT);
        }
    };

    match cli.command {
        Command::Plan(args) => plan::run(&args),
        Command::Analyze(args) => analyze::run(&args),
        Command::Version => {
            println!("trapla {}", env!("CARGO_PKG_VERSION"));
            ExitCode::from(EXIT_OK)
        }
    }
}
