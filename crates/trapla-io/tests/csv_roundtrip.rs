//! `trapla-io` 的 CSV 契约：写→读回一致、坏输入明确报错、空文件、列数不齐、逐字节可复现。

use std::path::{Path, PathBuf};

use trapla_core::geom::Dot;
use trapla_core::grid::Grid;
use trapla_io::csv::{read_matrix, read_xy, write_xy};

/// 本次测试专用的临时目录（按测试名 + 进程号隔离）。
fn tmp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("trapla_io_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("创建临时目录");
    dir
}

fn write_text(dir: &Path, name: &str, text: &str) -> PathBuf {
    let path = dir.join(name);
    std::fs::write(&path, text).expect("写入测试夹具");
    path
}

fn err_text<T>(res: Result<T, trapla_io::IoError>) -> String {
    match res {
        Ok(_) => panic!("期望报错，实际成功"),
        Err(e) => e.to_string(),
    }
}

#[test]
fn write_read_xy_roundtrip_exact() {
    let dir = tmp_dir("roundtrip");
    let path = dir.join("path.csv");
    let pts = vec![
        Dot::new(1.5, -2.25),
        Dot::new(0.0, 0.0),
        Dot::new(33.4096, 11.7822),
        Dot::new(-0.000001, 12.5),
    ];

    write_xy(&path, &pts).expect("写点位");
    let back = read_xy(&path).expect("读回点位");

    assert_eq!(back, pts, "写→读回必须逐点相等");
}

#[test]
fn write_xy_header_and_fixed_decimals() {
    let dir = tmp_dir("format");
    let path = dir.join("path.csv");

    write_xy(&path, &[Dot::new(1.5, -2.25), Dot::new(0.0, 0.0)]).expect("写点位");
    let text = std::fs::read_to_string(&path).expect("读文本");

    assert_eq!(text, "x,y\n1.500000,-2.250000\n0.000000,0.000000\n");
}

#[test]
fn write_xy_is_byte_deterministic() {
    let dir = tmp_dir("deterministic");
    let a = dir.join("a.csv");
    let b = dir.join("b.csv");
    // 用不可整表示的十进制值，确认两次写出的文本逐字节相同。
    let pts = vec![Dot::new(0.1 + 0.2, 1.0 / 3.0), Dot::new(1e-7, -1e7)];

    write_xy(&a, &pts).expect("第一次写");
    write_xy(&b, &pts).expect("第二次写");

    assert_eq!(
        std::fs::read(&a).expect("读 a"),
        std::fs::read(&b).expect("读 b"),
        "同一输入两次写出必须逐字节相同"
    );
}

#[test]
fn read_matrix_parses_row_major() {
    let dir = tmp_dir("matrix");
    let path = write_text(&dir, "map.csv", "1,2,3\n4,5,6\n");

    let grid: Grid = read_matrix(&path).expect("读矩阵");
    assert_eq!(grid.rows(), 2);
    assert_eq!(grid.cols(), 3);
    // x 是行、y 是列：第 2 行第 3 列 = 6.0
    assert_eq!(grid.cell(trapla_core::geom::Idx::new(1, 2)), Some(6.0));
    assert_eq!(grid.cell(trapla_core::geom::Idx::new(0, 1)), Some(2.0));
}

#[test]
fn read_matrix_rejects_non_numeric_with_position() {
    let dir = tmp_dir("badcell");
    let path = write_text(&dir, "map.csv", "1,2\n3,oops\n");

    let msg = err_text(read_matrix(&path));
    assert!(msg.contains("第 2 行"), "报错要带行号：{msg}");
    assert!(msg.contains("第 2 列"), "报错要带列号：{msg}");
    assert!(msg.contains("oops"), "报错要带原文：{msg}");
}

#[test]
fn read_matrix_rejects_ragged_rows() {
    let dir = tmp_dir("ragged");
    let path = write_text(&dir, "map.csv", "1,2,3\n4,5\n");

    let msg = err_text(read_matrix(&path));
    assert!(msg.contains("第 2 行"), "报错要带行号：{msg}");
    assert!(msg.contains("3 列"), "报错要带首行列数：{msg}");
    assert!(msg.contains("2 列"), "报错要带实际列数：{msg}");
}

#[test]
fn read_matrix_rejects_empty_file_and_blank_line() {
    let dir = tmp_dir("empty");
    let empty = write_text(&dir, "empty.csv", "");
    let msg = err_text(read_matrix(&empty));
    assert!(msg.contains("空文件"), "空文件要明确报错：{msg}");

    let blank = write_text(&dir, "blank.csv", "1,2\n\n3,4\n");
    let msg = err_text(read_matrix(&blank));
    assert!(msg.contains("第 2 行"), "空行要带行号：{msg}");
    assert!(msg.contains("空行"), "空行要明确报错：{msg}");
}

#[test]
fn read_xy_rejects_bad_column_count_and_empty_file() {
    let dir = tmp_dir("badxy");
    let three = write_text(&dir, "three.csv", "x,y\n1,2,3\n");
    let msg = err_text(read_xy(&three));
    assert!(msg.contains("第 2 行"), "报错要带行号：{msg}");
    assert!(msg.contains("实际 3 列"), "报错要带实际列数：{msg}");

    let empty = write_text(&dir, "empty.csv", "x,y\n");
    let msg = err_text(read_xy(&empty));
    assert!(msg.contains("空文件"), "只有表头也要明确报错：{msg}");
}
