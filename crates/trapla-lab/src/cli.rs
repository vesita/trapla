//! 命令行入口：`trapla-lab {scenario, validate, bench, list}`。
//!
//! 退出码：`0` 通过、`1` 输入错、`2` 硬约束越界 / 门禁不过、`3` 内部错。

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{CommandFactory, FromArgMatches, Parser, Subcommand};
use trapla_core::config::RobotConfig;
use trapla_core::geom::Dot;
use trapla_core::terrain::Terrain;

use crate::bench::{BenchConfig, BenchError, BenchReport};
use crate::planner::{NaiveStraightPlanner, Planner, RobotPlanner};
use crate::report;
use crate::scenario;
use crate::validate::{
    Limits, RzConvention, SensitivityReport, ValidateOptions, ValidationReport, validate_path,
};

/// 成功。
pub const EXIT_OK: u8 = 0;
/// 输入错（文件读不了 / 参数非法 / 坐标越界）。
pub const EXIT_INPUT: u8 = 1;
/// 硬约束越界（`validate`）或回归门禁不过（`bench`）。
pub const EXIT_VIOLATION: u8 = 2;
/// 其它内部错（写文件失败等）。
pub const EXIT_INTERNAL: u8 = 3;

/// `--help` 与代码共用的退出码说明。
fn exit_code_table() -> String {
    format!(
        "退出码:\n  \
         {EXIT_OK} 通过\n  \
         {EXIT_INPUT} 输入错（参数非法 / 文件读不了 / 坐标越界）\n  \
         {EXIT_VIOLATION} 硬约束越界或回归门禁不过\n  \
         {EXIT_INTERNAL} 其它内部错"
    )
}

#[derive(Debug, Parser)]
#[command(name = "trapla-lab", about = "trapla 无头算法测试台（场景 / 独立约束校验 / 批量跑分）")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// 生成内置场景的地图 CSV 与 manifest
    Scenario(ScenarioArgs),
    /// 用指定规划器在地图上跑一次（写出路径 + 证据 JSON）
    Plan(PlanArgs),
    /// 独立校验一条点位序列的硬约束
    Validate(ValidateArgs),
    /// 场景 × seed 批量跑分、基线对比与回归门禁
    Bench(BenchArgs),
    /// 列出内置场景
    List(ListArgs),
}

#[derive(Debug, clap::Args)]
struct ScenarioArgs {
    /// 输出目录：每场景一个 `<id>.csv`，外加 `manifest.json`
    #[arg(long, value_name = "dir")]
    out: PathBuf,
    /// 地图边长（行=列，缺省 200）
    #[arg(long, value_name = "N")]
    size: Option<usize>,
    /// 覆盖所有场景的 seed（影响随机障碍场）
    #[arg(long, value_name = "u64")]
    seed: Option<u64>,
    /// 只生成这些场景（逗号分隔）
    #[arg(long, value_name = "a,b")]
    scenarios: Option<String>,
}

#[derive(Debug, clap::Args)]
struct PlanArgs {
    /// 输入地图 CSV（无表头纯数值）
    #[arg(long, value_name = "csv")]
    map: PathBuf,
    /// 起点（双足中心），格式 x,y
    #[arg(long, value_name = "x,y")]
    start: Xy,
    /// 终点，格式 x,y
    #[arg(long, value_name = "x,y")]
    goal: Xy,
    /// 随机种子（缺省 0）
    #[arg(long, value_name = "u64")]
    seed: Option<u64>,
    /// 规划器：robot（缺省）或 naive_straight
    #[arg(long, value_name = "name")]
    planner: Option<String>,
    /// 输出点位 CSV
    #[arg(long, value_name = "csv")]
    out: Option<PathBuf>,
    /// 证据 JSON 输出路径（含奇偶脚映射、真实末态、可选校验报告）
    #[arg(long, value_name = "json")]
    json: Option<PathBuf>,
    /// 顺带用独立校验器判这条路径，有硬越界就退 2
    #[arg(long)]
    validate: bool,
    /// rz 重建约定：smoothed（缺省）/ raw / sticky
    #[arg(long, value_name = "smoothed|raw|sticky")]
    rz: Option<String>,
    /// 输入坐标的量化步长（缺省 5e-7）
    #[arg(long, value_name = "f64")]
    coord_quantum: Option<f64>,
}

#[derive(Debug, clap::Args)]
struct ValidateArgs {
    /// 待校验的点位 CSV（表头 `x,y`，偶数下标为左脚）
    #[arg(long, value_name = "csv")]
    path: PathBuf,
    /// 起点（双足中心），格式 x,y —— 点位序列不含初始足位，缺了无法重建第一步
    #[arg(long, value_name = "x,y")]
    start: Xy,
    /// 地图 CSV；给了才检查足覆盖是否全在图内与法向角
    #[arg(long, value_name = "csv")]
    map: Option<PathBuf>,
    /// 终点，格式 x,y（只用于报告 goal_error）
    #[arg(long, value_name = "x,y")]
    goal: Option<Xy>,
    /// 约束覆盖 JSON，例如 {"max_stride": 41.0}；未知键直接报错
    #[arg(long, value_name = "json")]
    overrides: Option<PathBuf>,
    /// rz 重建约定：smoothed（缺省，复现提交语义）/ raw / sticky
    #[arg(long, value_name = "smoothed|raw|sticky")]
    rz: Option<String>,
    /// 偶数下标是左脚还是右脚：left（缺省）或 right
    #[arg(long, value_name = "left|right")]
    first_moved: Option<String>,
    /// 输入坐标的量化步长（缺省 5e-7；0 = 严格判据）
    #[arg(long, value_name = "f64")]
    coord_quantum: Option<f64>,
    /// 报告 JSON 输出路径
    #[arg(long, value_name = "json")]
    json: Option<PathBuf>,
}

#[derive(Debug, clap::Args)]
struct BenchArgs {
    /// 报告 JSON 输出路径
    #[arg(long, value_name = "json")]
    out: Option<PathBuf>,
    /// 明细 CSV 输出路径
    #[arg(long, value_name = "csv")]
    csv: Option<PathBuf>,
    /// 地图落盘目录（每场景一个 `<id>.csv`）
    #[arg(long, value_name = "dir")]
    dump_maps: Option<PathBuf>,
    /// 路径落盘目录（每 run 一个 `<scenario>-<planner>-<seed>.csv`）
    #[arg(long, value_name = "dir")]
    dump_paths: Option<PathBuf>,
    /// 地图边长（行=列，缺省 200）
    #[arg(long, value_name = "N")]
    size: Option<usize>,
    /// seed 列表（逗号分隔，缺省 1,2）
    #[arg(long, value_name = "a,b")]
    seeds: Option<String>,
    /// 场景列表（逗号分隔，缺省全部）
    #[arg(long, value_name = "a,b")]
    scenarios: Option<String>,
    /// 规划器列表（逗号分隔：robot,naive_straight；缺省全部）
    #[arg(long, value_name = "a,b")]
    planners: Option<String>,
    /// 基线上报 JSON（上一次 `bench --out` 的内容）
    #[arg(long, value_name = "json")]
    baseline: Option<PathBuf>,
    /// 约束覆盖 JSON（同时作用于校验器与参考规划器）
    #[arg(long, value_name = "json")]
    overrides: Option<PathBuf>,
    /// rz 重建约定：smoothed（缺省，复现提交语义）/ raw / sticky
    #[arg(long, value_name = "smoothed|raw|sticky")]
    rz: Option<String>,
    /// 输入坐标的量化步长（缺省 5e-7）
    #[arg(long, value_name = "f64")]
    coord_quantum: Option<f64>,
    /// 步数允许的相对增长（缺省 0.10）
    #[arg(long, value_name = "f64")]
    max_step_regression: Option<f64>,
    /// proper_crossings 允许的绝对增长（缺省 0）
    #[arg(long, value_name = "N")]
    proper_slack: Option<usize>,
    /// 横向间距 min 允许的下降量（缺省 0.0）
    #[arg(long, value_name = "f64")]
    lateral_slack: Option<f64>,
}

#[derive(Debug, clap::Args)]
struct ListArgs {
    /// 地图边长（行=列，缺省 200）
    #[arg(long, value_name = "N")]
    size: Option<usize>,
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

impl Xy {
    fn dot(self) -> Dot {
        Dot::new(self.x, self.y)
    }
}

/// 进程入口。
pub fn main() -> ExitCode {
    let help = exit_code_table();
    let command = Cli::command()
        .after_help(help.clone())
        .mut_subcommand("plan", |sub| sub.after_help(help.clone()))
        .mut_subcommand("validate", |sub| sub.after_help(help.clone()))
        .mut_subcommand("bench", |sub| sub.after_help(help));
    let matches = match command.try_get_matches() {
        Ok(matches) => matches,
        Err(error) => {
            let code = if error.use_stderr() { EXIT_INPUT } else { EXIT_OK };
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
        Command::Scenario(args) => run_scenario(&args),
        Command::Plan(args) => run_plan(&args),
        Command::Validate(args) => run_validate(&args),
        Command::Bench(args) => run_bench(&args),
        Command::List(args) => {
            let size = args.size.unwrap_or(200);
            print!("{}", report::scenario_table(&scenario::builtin(size)));
            ExitCode::from(EXIT_OK)
        }
    }
}

/// 逗号分隔列表。
fn split_list(raw: &str) -> Vec<String> {
    raw.split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(str::to_string)
        .collect()
}

fn parse_rz(raw: Option<&String>) -> Result<RzConvention, String> {
    match raw.map(String::as_str) {
        None | Some("smoothed") => Ok(RzConvention::Smoothed),
        Some("raw") => Ok(RzConvention::Raw),
        Some("sticky") => Ok(RzConvention::Sticky),
        Some(other) => Err(format!(
            "未知的 rz 约定 `{other}`（可用 smoothed / raw / sticky）"
        )),
    }
}

fn parse_first_moved(raw: Option<&String>) -> Result<bool, String> {
    match raw.map(String::as_str) {
        None | Some("left") => Ok(true),
        Some("right") => Ok(false),
        Some(other) => Err(format!(
            "未知的 --first-moved `{other}`（可用 left / right）"
        )),
    }
}

fn read_overrides(path: Option<&PathBuf>) -> Result<Option<serde_json::Value>, String> {
    let Some(path) = path else {
        return Ok(None);
    };
    let text = fs::read_to_string(path).map_err(|e| format!("读不到 {}: {e}", path.display()))?;
    let value: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("{} 不是合法 JSON: {e}", path.display()))?;
    Ok(Some(value))
}

fn write_text(path: &Path, text: &str) -> Result<(), String> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent).map_err(|e| format!("建目录 {} 失败: {e}", parent.display()))?;
    }
    fs::write(path, text).map_err(|e| format!("写 {} 失败: {e}", path.display()))
}

fn run_scenario(args: &ScenarioArgs) -> ExitCode {
    let size = args.size.unwrap_or(200);
    let ids = args.scenarios.as_deref().map(split_list);
    let mut scenarios = match scenario::select(size, ids.as_deref()) {
        Ok(scenarios) => scenarios,
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    if let Some(seed) = args.seed {
        for scenario in &mut scenarios {
            scenario.seed = seed;
        }
    }

    if let Err(error) = fs::create_dir_all(&args.out) {
        eprintln!("错误: 建目录 {} 失败: {error}", args.out.display());
        return ExitCode::from(EXIT_INTERNAL);
    }
    for scenario in &scenarios {
        let path = args.out.join(format!("{}.csv", scenario.id));
        if let Err(error) = scenario::write_map_csv(&path, &scenario.grid()) {
            eprintln!("错误: 写 {} 失败: {error}", path.display());
            return ExitCode::from(EXIT_INTERNAL);
        }
    }
    let manifest = match scenario::manifest_json(&scenarios) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("错误: manifest 序列化失败: {error}");
            return ExitCode::from(EXIT_INTERNAL);
        }
    };
    let manifest_path = args.out.join("manifest.json");
    if let Err(detail) = write_text(&manifest_path, &manifest) {
        eprintln!("错误: {detail}");
        return ExitCode::from(EXIT_INTERNAL);
    }

    print!("{}", report::scenario_table(&scenarios));
    eprintln!("场景已写入 {}（含 manifest.json）", args.out.display());
    ExitCode::from(EXIT_OK)
}

/// `plan` 的 JSON 载荷：规划结果 + 机器人真实末态 + （可选）校验报告。
#[derive(serde::Serialize)]
struct PlanOutput<'a> {
    planner: &'a str,
    status: &'a str,
    detail: &'a str,
    points: usize,
    parity0_is_left: bool,
    parity_evidence: crate::planner::ParityEvidence,
    path_sha256: String,
    final_feet: Option<[(f64, f64, f64); 2]>,
    validation: Option<&'a ValidationReport>,
}

fn run_plan(args: &PlanArgs) -> ExitCode {
    let grid = match trapla_io::csv::read_matrix(&args.map) {
        Ok(grid) => grid,
        Err(error) => {
            eprintln!("错误: 读地图 {} 失败: {error}", args.map.display());
            return ExitCode::from(EXIT_INPUT);
        }
    };
    let terrain = Terrain::from_grid(grid);
    let name = args.planner.as_deref().unwrap_or("robot");
    let Some(planner) = planner_by_name(name) else {
        eprintln!(
            "错误: 未知规划器 `{name}`；可用：{}",
            PLANNER_NAMES.join(", ")
        );
        return ExitCode::from(EXIT_INPUT);
    };
    let config = RobotConfig::default();
    let request = crate::planner::PlanRequest {
        terrain: &terrain,
        start: args.start.dot(),
        goal: args.goal.dot(),
        config,
        seed: args.seed.unwrap_or(0),
    };
    let started = std::time::Instant::now();
    let result = planner.plan(&request);
    let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
    println!(
        "规划器={} 状态={} 点={} 末端证据={:?} 耗时={elapsed_ms:.3}ms",
        planner.name(),
        result.status.name(),
        result.path.len(),
        result.parity_evidence
    );
    if !result.detail.is_empty() {
        println!("备注: {}", result.detail);
    }

    if let Some(out) = &args.out
        && !result.path.is_empty()
        && let Err(error) = trapla_io::csv::write_xy(out, &result.path)
    {
        eprintln!("错误: 写 {} 失败: {error}", out.display());
        return ExitCode::from(EXIT_INTERNAL);
    }

    let rz = match parse_rz(args.rz.as_ref()) {
        Ok(rz) => rz,
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    let report = if args.validate && !result.path.is_empty() {
        let options = ValidateOptions {
            limits: Limits::canonical(),
            rz,
            parity0_is_left: result.parity0_is_left,
            coord_quantum: args.coord_quantum.unwrap_or(5e-7),
        };
        let report = validate_path(
            &result.path,
            request.start,
            Some(request.goal),
            Some(&terrain),
            &options,
        );
        println!(
            "自校验: 严格越界步数={} 超量化界步数={} | {}",
            report.strict_violation_steps,
            report.hard_violation_steps,
            if report.passed() { "通过" } else { "不通过" }
        );
        Some(report)
    } else {
        None
    };

    if let Some(json_path) = &args.json {
        let final_feet = result.final_feet.map(|feet| {
            [
                (feet[0].0.x(), feet[0].0.y(), feet[0].1),
                (feet[1].0.x(), feet[1].0.y(), feet[1].1),
            ]
        });
        let payload = PlanOutput {
            planner: planner.name(),
            status: result.status.name(),
            detail: &result.detail,
            points: result.path.len(),
            parity0_is_left: result.parity0_is_left,
            parity_evidence: result.parity_evidence,
            path_sha256: crate::bench::path_sha256(&result.path),
            final_feet,
            validation: report.as_ref(),
        };
        match serde_json::to_string_pretty(&payload) {
            Ok(text) => {
                if let Err(detail) = write_text(json_path, &format!("{text}\n")) {
                    eprintln!("错误: {detail}");
                    return ExitCode::from(EXIT_INTERNAL);
                }
                eprintln!("证据已写入 {}", json_path.display());
            }
            Err(error) => {
                eprintln!("错误: 序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        }
    }

    if let Some(report) = &report
        && !report.passed()
    {
        eprintln!("错误: 硬约束越界 {} 步", report.hard_violation_steps);
        return ExitCode::from(EXIT_VIOLATION);
    }
    ExitCode::from(EXIT_OK)
}

/// `validate` 的 JSON 载荷：主报告 + 敏感性探针。
#[derive(serde::Serialize)]
struct ValidateOutput<'a> {
    report: &'a ValidationReport,
    sensitivity: &'a SensitivityReport,
}

fn run_validate(args: &ValidateArgs) -> ExitCode {
    let path = match trapla_io::csv::read_xy(&args.path) {
        Ok(path) => path,
        Err(error) => {
            eprintln!("错误: 读 {} 失败: {error}", args.path.display());
            return ExitCode::from(EXIT_INPUT);
        }
    };

    let terrain = match &args.map {
        None => None,
        Some(map_path) => match trapla_io::csv::read_matrix(map_path) {
            Ok(grid) => Some(Terrain::from_grid(grid)),
            Err(error) => {
                eprintln!("错误: 读地图 {} 失败: {error}", map_path.display());
                return ExitCode::from(EXIT_INPUT);
            }
        },
    };

    let mut limits = Limits::canonical();
    match read_overrides(args.overrides.as_ref()) {
        Ok(Some(value)) => match limits.apply_overrides(&value) {
            Ok(applied) => {
                for line in &applied {
                    println!("约束覆盖: {line}");
                }
            }
            Err(detail) => {
                eprintln!("错误: {detail}");
                return ExitCode::from(EXIT_INPUT);
            }
        },
        Ok(None) => {}
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    }

    let rz = match parse_rz(args.rz.as_ref()) {
        Ok(rz) => rz,
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    let parity0_is_left = match parse_first_moved(args.first_moved.as_ref()) {
        Ok(flag) => flag,
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    };

    let start = args.start.dot();
    let goal = args.goal.map(Xy::dot);
    let options = ValidateOptions {
        limits,
        rz,
        parity0_is_left,
        coord_quantum: args.coord_quantum.unwrap_or(5e-7),
    };

    let report = validate_path(&path, start, goal, terrain.as_ref(), &options);
    let sensitivity = crate::validate::sensitivity(
        &path,
        start,
        goal,
        terrain.as_ref(),
        &options,
    );

    print!("{}", report::validation_table(&report));
    println!(
        "敏感性: {} -> hard={} strict={} 结论翻转={}",
        sensitivity.rz_convention_flip.change,
        sensitivity.rz_convention_flip.hard_violation_steps,
        sensitivity.rz_convention_flip.strict_violation_steps,
        if sensitivity.rz_convention_flip.verdict_flips {
            "是"
        } else {
            "否"
        }
    );
    println!(
        "敏感性: {} -> hard={} strict={} 结论翻转={}",
        sensitivity.parity_flip.change,
        sensitivity.parity_flip.hard_violation_steps,
        sensitivity.parity_flip.strict_violation_steps,
        if sensitivity.parity_flip.verdict_flips {
            "是"
        } else {
            "否"
        }
    );

    if let Some(json_path) = &args.json {
        let payload = ValidateOutput {
            report: &report,
            sensitivity: &sensitivity,
        };
        match serde_json::to_string_pretty(&payload) {
            Ok(text) => {
                if let Err(detail) = write_text(json_path, &format!("{text}\n")) {
                    eprintln!("错误: {detail}");
                    return ExitCode::from(EXIT_INTERNAL);
                }
                eprintln!("报告已写入 {}", json_path.display());
            }
            Err(error) => {
                eprintln!("错误: 报告序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        }
    }

    if report.passed() {
        ExitCode::from(EXIT_OK)
    } else {
        eprintln!(
            "错误: 硬约束越界：{} 步（严格判据 {} 步）",
            report.hard_violation_steps, report.strict_violation_steps
        );
        ExitCode::from(EXIT_VIOLATION)
    }
}

/// 按名字建一个规划器。
fn planner_by_name(name: &str) -> Option<Box<dyn Planner>> {
    match name {
        "robot" => Some(Box::new(RobotPlanner)),
        "naive_straight" => Some(Box::new(NaiveStraightPlanner)),
        _ => None,
    }
}

/// 内置规划器名字（`--planners` 的可选值）。
const PLANNER_NAMES: [&str; 2] = ["robot", "naive_straight"];

/// 按名字挑规划器；顺序沿用调用方给的顺序，重复名字只登记一次。
fn select_planners(names: Option<&[String]>) -> Result<Vec<Box<dyn Planner>>, String> {
    let requested: Vec<String> = match names {
        None => PLANNER_NAMES.iter().map(|name| name.to_string()).collect(),
        Some(names) => names.to_vec(),
    };
    let mut chosen: Vec<Box<dyn Planner>> = Vec::with_capacity(requested.len());
    for name in requested {
        let Some(planner) = planner_by_name(&name) else {
            return Err(format!(
                "未知规划器 `{name}`；可用：{}",
                PLANNER_NAMES.join(", ")
            ));
        };
        if !chosen
            .iter()
            .any(|existing| existing.name() == planner.name())
        {
            chosen.push(planner);
        }
    }
    Ok(chosen)
}

fn run_bench(args: &BenchArgs) -> ExitCode {
    let size = args.size.unwrap_or(200);
    let mut config = BenchConfig {
        rows: size,
        cols: size,
        dump_paths_dir: args.dump_paths.clone(),
        ..BenchConfig::default()
    };
    if let Some(seeds) = &args.seeds {
        let parsed: Result<Vec<u64>, _> = split_list(seeds)
            .iter()
            .map(|item| item.parse::<u64>())
            .collect();
        match parsed {
            Ok(seeds) if !seeds.is_empty() => config.seeds = seeds,
            Ok(_) => {
                eprintln!("错误: --seeds 不能为空");
                return ExitCode::from(EXIT_INPUT);
            }
            Err(error) => {
                eprintln!("错误: --seeds 解析失败: {error}");
                return ExitCode::from(EXIT_INPUT);
            }
        }
    }
    config.scenario_ids = args.scenarios.as_deref().map(split_list);

    let mut limits = Limits::canonical();
    match read_overrides(args.overrides.as_ref()) {
        Ok(Some(value)) => match limits.apply_overrides(&value) {
            Ok(applied) => {
                for line in &applied {
                    eprintln!("约束覆盖: {line}");
                }
            }
            Err(detail) => {
                eprintln!("错误: {detail}");
                return ExitCode::from(EXIT_INPUT);
            }
        },
        Ok(None) => {}
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    }
    config.base_config = limits.to_config();
    if let Some(raw) = &args.rz {
        match parse_rz(Some(raw)) {
            Ok(rz) => config.rz = rz,
            Err(detail) => {
                eprintln!("错误: {detail}");
                return ExitCode::from(EXIT_INPUT);
            }
        }
    }
    if let Some(quantum) = args.coord_quantum {
        config.coord_quantum = quantum;
    }
    if let Some(value) = args.max_step_regression {
        config.max_step_regression = value;
    }
    if let Some(value) = args.proper_slack {
        config.proper_slack = value;
    }
    if let Some(value) = args.lateral_slack {
        config.lateral_slack = value;
    }
    if let Some(baseline_path) = &args.baseline {
        config.baseline = match load_baseline(baseline_path) {
            Ok(report) => Some(report),
            Err(detail) => {
                eprintln!("错误: {detail}");
                return ExitCode::from(EXIT_INPUT);
            }
        };
    }

    let planner_names = args.planners.as_deref().map(split_list);
    let planners = match select_planners(planner_names.as_deref()) {
        Ok(planners) => planners,
        Err(detail) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    if planners.is_empty() {
        eprintln!("错误: 没有可用的规划器");
        return ExitCode::from(EXIT_INPUT);
    }
    let planner_refs: Vec<&dyn Planner> = planners.iter().map(|planner| planner.as_ref()).collect();

    let report = match crate::bench::run(&config, &planner_refs) {
        Ok(report) => report,
        Err(BenchError::Input(detail)) => {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INPUT);
        }
        Err(error) => {
            eprintln!("错误: {error}");
            return ExitCode::from(EXIT_INTERNAL);
        }
    };

    print!("{}", report::bench_table(&report));

    if let Some(dir) = &args.dump_maps {
        let ids = config.scenario_ids.clone();
        if let Err(detail) = dump_maps(config.rows, ids.as_deref(), dir) {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INTERNAL);
        }
        eprintln!("地图已写入 {}", dir.display());
    }
    if let Some(csv_path) = &args.csv {
        if let Err(detail) = write_text(csv_path, &report::bench_csv(&report)) {
            eprintln!("错误: {detail}");
            return ExitCode::from(EXIT_INTERNAL);
        }
        eprintln!("明细已写入 {}", csv_path.display());
    }
    if let Some(json_path) = &args.out {
        match serde_json::to_string_pretty(&report) {
            Ok(text) => {
                if let Err(detail) = write_text(json_path, &format!("{text}\n")) {
                    eprintln!("错误: {detail}");
                    return ExitCode::from(EXIT_INTERNAL);
                }
                eprintln!("报告已写入 {}", json_path.display());
            }
            Err(error) => {
                eprintln!("错误: 报告序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        }
    }

    if report.gate.passed {
        ExitCode::from(EXIT_OK)
    } else {
        eprintln!("错误: 门禁不通过（{} 条）", report.gate.failures.len());
        ExitCode::from(EXIT_VIOLATION)
    }
}

fn load_baseline(path: &Path) -> Result<BenchReport, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("读不到 {}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("{} 不是合法的 bench 报告: {e}", path.display()))
}

fn dump_maps(rows: usize, ids: Option<&[String]>, dir: &Path) -> Result<(), String> {
    let scenarios = scenario::select(rows, ids)?;
    fs::create_dir_all(dir).map_err(|e| format!("建目录 {} 失败: {e}", dir.display()))?;
    for scenario in &scenarios {
        let path = dir.join(format!("{}.csv", scenario.id));
        scenario::write_map_csv(&path, &scenario.grid())
            .map_err(|e| format!("写 {} 失败: {e}", path.display()))?;
    }
    Ok(())
}
