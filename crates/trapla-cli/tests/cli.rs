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
