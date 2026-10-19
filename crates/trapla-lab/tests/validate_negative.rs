//! 集成验收：CSV 进出的负向对照 —— 干净路径照过，动一步就红，修回再绿。
//!
//! 三段走的是同一条外部入口（点位 CSV + 地图 CSV + 起点），不碰内部 API：
//!
//! 1. 夹具 `tests/fixtures/clean_path.csv`（合成的一条逐点合法路径）⇒ 通过；
//! 2. 把某个**同脚位移**改成 > 40（步长判据是严格小于）⇒ 报出该步 `stride` 且不通过；
//! 3. 造一个双足间距 > 10 的坏点 ⇒ 报出 `spacing`；
//! 4. 改回夹具原文 ⇒ 重新通过。

use std::fs;
use std::path::{Path, PathBuf};

use trapla_core::geom::Dot;
use trapla_lab::validate::{Limits, RzConvention, ValidateOptions, Violation, validate_path};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn scratch(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("trapla-lab-{tag}-{}", std::process::id()));
    fs::create_dir_all(&dir).expect("建临时目录");
    dir
}

fn options() -> ValidateOptions {
    ValidateOptions {
        limits: Limits::canonical(),
        rz: RzConvention::Smoothed,
        parity0_is_left: true,
        // 夹具坐标是 6 位小数写出的整数 / 5.2 一半，读回来逐位相同 ⇒ 可以按严格判据判。
        coord_quantum: 0.0,
    }
}

fn load(path: &Path) -> Vec<Dot> {
    trapla_io::csv::read_xy(path).expect("读点位 CSV")
}

fn save(path: &Path, points: &[Dot]) {
    trapla_io::csv::write_xy(path, points).expect("写点位 CSV");
}

fn validate_file(path: &Path) -> trapla_lab::validate::ValidationReport {
    let map = fixtures().join("flat_map.csv");
    let grid = trapla_io::csv::read_matrix(&map).expect("读地图");
    let terrain = trapla_core::terrain::Terrain::from_grid(grid);
    validate_path(
        &load(path),
        Dot::new(0.0, 10.0),
        None,
        Some(&terrain),
        &options(),
    )
}

#[test]
fn negative_controls_on_csv_round_trip() {
    let clean = fixtures().join("clean_path.csv");
    let baseline = load(&clean);

    // ① 干净路径：通过，且每一步的足覆盖都在图内。
    let report = validate_file(&clean);
    assert_eq!(
        report.strict_violation_steps, 0,
        "夹具本身必须无越界：{:?}",
        report.strict_counts
    );
    assert!(report.passed());
    assert!(
        report.steps.iter().all(|step| step.standable == Some(true)),
        "夹具必须每步可站立"
    );

    let work = scratch("negative");
    let broken = work.join("bad_stride.csv");

    // ② 把第 10 步（偶数下标那一只脚）的同脚位移推到 75（> 40）。
    let mut corrupted = baseline.clone();
    corrupted[10] = Dot::new(corrupted[10].x() + 55.0, corrupted[10].y());
    save(&broken, &corrupted);
    let report = validate_file(&broken);
    assert!(!report.passed(), "步长越界必须被判出来");
    assert_eq!(
        report.steps[10].stride, 75.0,
        "第 10 步同脚位移应为 75"
    );
    assert!(
        report.steps[10].strict_violations.contains(&Violation::Stride),
        "第 10 步应报步长越界：{:?}",
        report.steps[10]
    );
    assert!(
        report.steps[10].hard_violations.contains(&Violation::Stride),
        "75 远超量化界，必须是硬越界"
    );
    assert_eq!(report.hard_counts.get(&Violation::Stride), Some(&1));

    // ③ 双足间距 > 10：把第 10 步横向拉开（支撑脚在 y=12.6 一侧）。
    let mut wide = baseline.clone();
    wide[10] = Dot::new(wide[10].x(), 30.0);
    save(&broken, &wide);
    let report = validate_file(&broken);
    assert!(!report.passed(), "间距越界必须被判出来");
    assert!(
        report.steps[10].spacing > Limits::canonical().max_separation,
        "第 10 步间距应超过上限，实得 {}",
        report.steps[10].spacing
    );
    assert!(
        report.steps[10]
            .strict_violations
            .contains(&Violation::Spacing),
        "第 10 步应报间距越界：{:?}",
        report.steps[10]
    );

    // ④ 修回原文：重新通过。
    save(&broken, &baseline);
    let report = validate_file(&broken);
    assert_eq!(
        report.strict_violation_steps, 0,
        "修回后必须重新通过：{:?}",
        report.strict_counts
    );
    assert!(report.passed());

    fs::remove_dir_all(&work).ok();
}

#[test]
fn tiny_coordinate_shift_does_not_flip_hard_verdict() {
    // 量化敏感性：把第 10 步回退 5e-7（小于 6 位小数的半步）后，判据仍是「通过」。
    let clean = fixtures().join("clean_path.csv");
    let mut shifted = load(&clean);
    shifted[10] = Dot::new(shifted[10].x() - 5e-7, shifted[10].y());
    let report = validate_path(
        &shifted,
        Dot::new(0.0, 10.0),
        None,
        None,
        &ValidateOptions {
            coord_quantum: 5e-7,
            ..options()
        },
    );
    assert!(report.passed(), "5e-7 的扰动不该翻结论：{:?}", report.strict_counts);
}
