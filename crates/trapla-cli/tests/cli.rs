//! `trapla` CLI 契约：`--help`、退出码、stdout/stderr 分工、analyze 的 JSON 输出。

use std::path::PathBuf;
use std::process::{Command, Output};

use trapla_core::geom::Dot;
use trapla_io::csv::write_xy;

fn trapla(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_trapla"))
        .args(args)
        .output()
        .expect("启动 trapla 二进制")
}

fn code(output: &Output) -> i32 {
    output
        .status
        .code()
        .expect("进程被信号终止（应为正常退出）")
}

fn stdout(output: &Output) -> String {
    String::from_utf8_lossy(&output.stdout).into_owned()
}

fn stderr(output: &Output) -> String {
    String::from_utf8_lossy(&output.stderr).into_owned()
}

fn tmp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("trapla_cli_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("创建临时目录");
    dir
}

#[test]
fn help_exits_zero_and_documents_exit_codes() {
    let output = trapla(&["--help"]);
    assert_eq!(code(&output), 0, "顶层 --help 应退 0");
    let text = stdout(&output);
    assert!(text.contains("退出码"), "help 要写退出码约定：{text}");
    assert!(
        text.contains("1 输入文件不可读"),
        "help 要写退出码 1：{text}"
    );
    assert!(text.contains("3 其它内部错误"), "help 要写退出码 3：{text}");

    let output = trapla(&["plan", "--help"]);
    assert_eq!(code(&output), 0, "plan --help 应退 0");
    let text = stdout(&output);
    assert!(text.contains("退出码"), "plan help 也要写退出码：{text}");
    assert!(text.contains("--repeat"), "plan help 要写 --repeat：{text}");
    assert!(text.contains("--seed"), "plan help 要写 --seed：{text}");
    assert!(
        text.contains("--metrics"),
        "plan help 要写 --metrics：{text}"
    );
}

#[test]
fn plan_missing_args_exits_nonzero() {
    let output = trapla(&["plan"]);
    assert_ne!(code(&output), 0, "缺参数必须退非 0");
    assert!(!stderr(&output).is_empty(), "参数错误要写 stderr");
}

#[test]
fn plan_missing_map_exits_1() {
    let dir = tmp_dir("missing_map");
    let out = dir.join("out.csv");
    let output = trapla(&[
        "plan",
        "--map",
        dir.join("no_such_map.csv").to_str().expect("utf8 路径"),
        "--start",
        "0,0",
        "--goal",
        "5,5",
        "--out",
        out.to_str().expect("utf8 路径"),
        "--seed",
        "1",
    ]);

    assert_eq!(code(&output), 1, "输入文件不存在应退 1");
    assert!(
        stderr(&output).contains("错误"),
        "缺少地图要在 stderr 说明：{}",
        stderr(&output)
    );
}

#[test]
fn plan_bad_matrix_exits_1_with_position() {
    let dir = tmp_dir("bad_matrix");
    let map = dir.join("map.csv");
    std::fs::write(&map, "1,2\n3,oops\n").expect("写坏地图");
    let out = dir.join("out.csv");

    let output = trapla(&[
        "plan",
        "--map",
        map.to_str().expect("utf8 路径"),
        "--start",
        "0,0",
        "--goal",
        "1,1",
        "--out",
        out.to_str().expect("utf8 路径"),
        "--seed",
        "1",
    ]);

    assert_eq!(code(&output), 1, "格式错应退 1");
    let err = stderr(&output);
    assert!(err.contains("第 2 行"), "报错要带行号：{err}");
    assert!(err.contains("oops"), "报错要带原文：{err}");
}

#[test]
fn analyze_writes_json_and_summary_goes_to_stdout() {
    let dir = tmp_dir("analyze");
    let path = dir.join("path.csv");
    write_xy(
        &path,
        &[Dot::new(0.0, 0.0), Dot::new(10.0, 0.0), Dot::new(20.0, 0.0)],
    )
    .expect("写路径夹具");
    let json = dir.join("metrics.json");

    let output = trapla(&[
        "analyze",
        "--path",
        path.to_str().expect("utf8 路径"),
        "--goal",
        "20,0",
        "--out",
        json.to_str().expect("utf8 路径"),
    ]);

    assert_eq!(code(&output), 0, "analyze 应退 0：{}", stderr(&output));
    let out = stdout(&output);
    assert!(out.contains("steps: 3"), "摘要要走 stdout：{out}");
    assert!(
        out.contains("total_length: 20.000000"),
        "摘要要走 stdout：{out}"
    );
    assert!(!out.contains("已读入"), "进度不应混进 stdout：{out}");

    let text = std::fs::read_to_string(&json).expect("读指标 JSON");
    let value: serde_json::Value = serde_json::from_str(&text).expect("指标 JSON 可解析");
    assert_eq!(value["steps"], 3);
    assert_eq!(value["proper_crossings"], 0);
}

#[test]
fn analyze_missing_path_exits_1() {
    let dir = tmp_dir("analyze_missing");
    let output = trapla(&[
        "analyze",
        "--path",
        dir.join("nope.csv").to_str().expect("utf8 路径"),
        "--goal",
        "0,0",
    ]);

    assert_eq!(code(&output), 1, "缺路径应退 1");
    assert!(stderr(&output).contains("错误"), "要在 stderr 说明");
}

/// 仓库里的真实地图（1000 行 × 3000 列）。
fn real_map() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/csv/map.csv")
}

#[test]
fn plan_out_of_bounds_goal_exits_1() {
    let dir = tmp_dir("oob_goal");
    let out = dir.join("out.csv");
    let map = real_map();

    let output = trapla(&[
        "plan",
        "--map",
        map.to_str().expect("utf8 路径"),
        "--start",
        "0,0",
        "--goal",
        "1500,1500",
        "--out",
        out.to_str().expect("utf8 路径"),
        "--seed",
        "0",
    ]);

    assert_eq!(
        code(&output),
        1,
        "越界目标点是输入错，应退 1 而不是 2：{}",
        stderr(&output)
    );
    let err = stderr(&output);
    assert!(
        err.contains("目标点 (1500.000000,1500.000000)"),
        "stderr 要给出实际值：{err}"
    );
    assert!(err.contains("超出地图范围"), "stderr 要说明越界：{err}");
}

#[test]
fn plan_out_of_bounds_start_exits_1() {
    let dir = tmp_dir("oob_start");
    let out = dir.join("out.csv");
    let map = real_map();

    let output = trapla(&[
        "plan",
        "--map",
        map.to_str().expect("utf8 路径"),
        "--start",
        "5000,0",
        "--goal",
        "10,10",
        "--out",
        out.to_str().expect("utf8 路径"),
        "--seed",
        "0",
    ]);

    assert_eq!(
        code(&output),
        1,
        "越界起点是输入错，应退 1：{}",
        stderr(&output)
    );
    assert!(
        stderr(&output).contains("起点 (5000.000000,0.000000)"),
        "stderr 要给出实际值：{}",
        stderr(&output)
    );
}

#[test]
fn plan_in_map_unreachable_exits_2() {
    let dir = tmp_dir("unreachable");
    let map = dir.join("wall.csv");
    // 400×400 全 1，列 200..220 整列 `inf`：20 列宽的墙横贯全图。
    // 聚合后至少一整个粗网格列成为障碍 ⇒ 分居两侧的起点/目标无路径；
    // 但 `--goal 350,350` 在 400×400 图内（不是越界），所以必须仍是 exit=2。
    let mut text = String::new();
    for _ in 0..400 {
        for j in 0..400 {
            text.push_str(if (200..220).contains(&j) { "inf" } else { "1" });
            text.push(if j == 399 { '\n' } else { ',' });
        }
    }
    std::fs::write(&map, text).expect("写墙地图");

    let out = dir.join("out.csv");
    let output = trapla(&[
        "plan",
        "--map",
        map.to_str().expect("utf8 路径"),
        "--start",
        "0,0",
        "--goal",
        "350,350",
        "--out",
        out.to_str().expect("utf8 路径"),
        "--seed",
        "0",
    ]);

    assert_eq!(
        code(&output),
        2,
        "图内确实不可达应仍是 2：{}",
        stderr(&output)
    );
    assert!(
        stderr(&output).contains("无路径"),
        "要报无路径：{}",
        stderr(&output)
    );
}
