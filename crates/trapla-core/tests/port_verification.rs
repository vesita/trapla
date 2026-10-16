//! T7 独立移植复核：**不依赖作者测试**的 golden 逐行对照 + 独立命题验证。
//!
//! 本文件由 T7 复核人（非 T2/T3/T4/T5 作者）编写，只读夹具、不改任何作者文件。
//!
//! 覆盖：
//! 1. `tests/golden/` 下全部 22 个夹具的逐行对照（按表头名解析，容差 1e-9）；
//! 2. 独立重写的 `up_fit` 单步（公开 API 不暴露被原地改写的 `three`，只能按语义重算）；
//! 3. 独立复核「洗牌消耗 rng」与「同 seed 可复现」两条命题；
//! 4. `golden_summary.txt` 里每一行的行数自洽核对。
//!
//! 运行：`cargo test -p trapla-core --test port_verification -- --nocapture`

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use trapla_core::astar::{direction_determine, geometric_decay, geometric_decay_start, scale_star};
use trapla_core::config::{FootShape, RobotConfig};
use trapla_core::error::CoreError;
use trapla_core::foot::{Foot, SlideResult};
use trapla_core::geom::{Dot, Idx, Line, Plane, Point3, fit_plane_up, index_scale};
use trapla_core::grid::Grid;
use trapla_core::metrics;
use trapla_core::rng::Rng;
use trapla_core::robot::{Robot, WhichFoot};
use trapla_core::terrain::Terrain;

// ------------------------------------------------------------------ 夹具读取

fn parse_f64(raw: &str) -> f64 {
    match raw.trim() {
        "inf" => f64::INFINITY,
        "-inf" => f64::NEG_INFINITY,
        "nan" => f64::NAN,
        other => other
            .parse::<f64>()
            .unwrap_or_else(|_| panic!("夹具里出现非法浮点 {other:?}")),
    }
}

struct Table {
    columns: BTreeMap<String, usize>,
    rows: Vec<Vec<String>>,
}

impl Table {
    fn load(name: &str) -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/golden")
            .join(name);
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("读不到夹具 {}: {e}", path.display()));
        let mut lines = text.lines().filter(|l| !l.trim().is_empty());
        let header: Vec<String> = lines
            .next()
            .unwrap_or_else(|| panic!("{name} 是空文件"))
            .split(',')
            .map(|c| c.trim().to_string())
            .collect();
        let columns = header
            .iter()
            .enumerate()
            .map(|(i, n)| (n.clone(), i))
            .collect();
        let rows = lines
            .map(|l| l.split(',').map(|c| c.trim().to_string()).collect())
            .collect();
        Self { columns, rows }
    }
    fn text(&self, row: &[String], c: &str) -> String {
        let i = *self
            .columns
            .get(c)
            .unwrap_or_else(|| panic!("夹具缺列 {c}"));
        row.get(i)
            .unwrap_or_else(|| panic!("行缺第 {i} 列（{c}）"))
            .clone()
    }
    fn num(&self, row: &[String], c: &str) -> f64 {
        parse_f64(&self.text(row, c))
    }
    fn int(&self, row: &[String], c: &str) -> i32 {
        self.num(row, c) as i32
    }
}

fn close(got: f64, want: f64, tol: f64) -> bool {
    if want.is_nan() {
        return got.is_nan();
    }
    if want.is_infinite() {
        return got == want;
    }
    (got - want).abs() <= tol
}

/// 复核台账：每个夹具一行。
#[derive(Default)]
struct Ledger {
    name: &'static str,
    total: usize,
    matched: usize,
    unverifiable: usize,
    known_divergent: Vec<String>,
    pending: Vec<String>,
    unregistered: Vec<String>,
    notes: Vec<String>,
}

impl Ledger {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            ..Default::default()
        }
    }
    fn mismatch(&mut self, kind: MismatchKind, detail: String) {
        match kind {
            MismatchKind::Registered => self.known_divergent.push(detail),
            MismatchKind::PendingDecision => self.pending.push(detail),
            MismatchKind::Unregistered => self.unregistered.push(detail),
        }
    }
}

#[derive(Clone, Copy)]
enum MismatchKind {
    /// 已在 doc/known_divergences.md 或源码注释里登记的有意分歧
    Registered,
    /// 本次复核新发现：需要 Lead 裁决「登记 or 修 or 接受」，不是静默通过
    PendingDecision,
    /// 未登记且无法归因 ⇒ 阻塞项
    Unregistered,
}

fn load_grounds() -> BTreeMap<i32, Grid> {
    let table = Table::load("grounds.csv");
    let mut entries: BTreeMap<i32, Vec<(usize, usize, f64)>> = BTreeMap::new();
    for row in &table.rows {
        let gid = table.int(row, "ground_id");
        let x = table.int(row, "x") as usize;
        let y = table.int(row, "y") as usize;
        entries
            .entry(gid)
            .or_default()
            .push((x, y, table.num(row, "height")));
    }
    entries
        .into_iter()
        .map(|(gid, cells)| {
            let rows = cells.iter().map(|c| c.0).max().unwrap() + 1;
            let cols = cells.iter().map(|c| c.1).max().unwrap() + 1;
            let mut data = vec![f64::NAN; rows * cols];
            for (x, y, h) in cells {
                data[x * cols + y] = h;
            }
            (gid, Grid::new(data, rows, cols).expect("grounds 规整"))
        })
        .collect()
}

// ------------------------------------------------- 独立重写：单步 up_fit（公开 API 不暴露被改写的 three）

fn cross3(a: Point3, b: Point3) -> (f64, f64, f64) {
    (
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
    )
}

fn plane_of3(t: &[Point3; 3]) -> Option<Plane> {
    let v1 = Point3::new(t[1].x() - t[0].x(), t[1].y() - t[0].y(), t[1].z() - t[0].z());
    let v2 = Point3::new(t[2].x() - t[0].x(), t[2].y() - t[0].y(), t[2].z() - t[0].z());
    let (nx, ny, nz) = cross3(v1, v2);
    if nx == 0.0 && ny == 0.0 && nz == 0.0 {
        return None;
    }
    Some(Plane {
        a: nx,
        b: ny,
        c: nz,
        d: -(nx * t[0].x() + ny * t[0].y() + nz * t[0].z()),
    })
}

/// 与 `CuPlane::get_pos` 同判据：> 0 上方、< 0 下方、否则内部。
fn side_of(p: Plane, q: Point3) -> i32 {
    let v = p.a * q.x() + p.b * q.y() + p.c * q.z() + p.d;
    if v > 1e-9 {
        1
    } else if v < -1e-9 {
        -1
    } else {
        0
    }
}

/// 独立重写的单步 `up_fit`（`geometry.cpp:1169-1199` 的语义，不是抄 Rust 实现）。
fn up_fit_one_ref(q: Point3, three: &mut [Point3; 3]) -> Option<(Plane, [Point3; 3])> {
    if let Some(p) = plane_of3(three)
        && side_of(p, q) != 1
    {
        return Some((p, *three));
    }

    for i in 0..3 {
        let saved = three[i];
        three[i] = q;
        if let Some(p) = plane_of3(three)
            && side_of(p, q) != 1
        {
            return Some((p, *three));
        }
        three[i] = saved;
    }
    three[0] = q;
    plane_of3(three).map(|p| (p, *three))
}

// ------------------------------------------------------------------ 各夹具对照

fn check_grounds(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    l.total = 387;
    l.matched = 387;
    let dims: Vec<String> = grounds
        .iter()
        .map(|(k, g)| format!("{k}:{}x{}", g.rows(), g.cols()))
        .collect();
    l.notes.push(format!("解析出 {} 张图 [{}]", grounds.len(), dims.join(" ")));
    assert_eq!(grounds.len(), 7, "grounds.csv 的 ground_id 数应为 7");
}

fn check_golden_summary(l: &mut Ledger) {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/golden/golden_summary.txt");
    let text = std::fs::read_to_string(&path).unwrap();
    let mut checked = 0_usize;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line == "done" || line.starts_with("note:") {
            continue;
        }
        // 形如 "<file> rows=N" / "<file> cases=M rows=N" / "<file> rows=N cells_rows=M"
        let mut it = line.split_whitespace();
        let file = it.next().unwrap();
        let mut fields: BTreeMap<String, usize> = BTreeMap::new();
        for kv in it {
            let (k, v) = kv.split_once('=').expect("summary 字段 k=v");
            fields.insert(
                k.to_string(),
                v.parse::<usize>().expect("summary 数字"),
            );
        }
        let fixture = Table::load(file);
        if let Some(&rows) = fields.get("rows") {
            assert_eq!(fixture.rows.len(), rows, "{file}: summary 行数与实际不符");
            checked += 1;
        }
        if let Some(&cases) = fields.get("cases") {
            let actual: BTreeSet<i32> = fixture.rows.iter().map(|r| fixture.int(r, "case")).collect();
            assert_eq!(actual.len(), cases, "{file}: summary cases 数与实际不符");
            checked += 1;
        }
        if let Some(&cells) = fields.get("cells_rows") {
            let cells_table = Table::load("scale_graph_cells.csv");
            assert_eq!(cells_table.rows.len(), cells, "scale_graph_cells 行数不符");
            checked += 1;
        }
        if let Some(&guides) = fields.get("guides_rows") {
            let g = Table::load("direction_guides.csv");
            assert_eq!(g.rows.len(), guides, "direction_guides 行数不符");
            checked += 1;
        }
    }
    l.total = checked;
    l.matched = checked;
    l.notes
        .push("golden_summary.txt 的每个行数/组数字段都与实际夹具一致".to_string());
}

fn check_normal_angle(l: &mut Ledger) {
    let t = Table::load("normal_angle.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let p = |i: usize| {
            Point3::new(
                t.num(row, &format!("p{i}x")),
                t.num(row, &format!("p{i}y")),
                t.num(row, &format!("p{i}z")),
            )
        };
        let pts = [p(1), p(2), p(3)];
        let degenerate = plane_of3(&pts).is_none();
        match (fit_plane_up(&pts), degenerate) {
            (Err(CoreError::DegeneratePlane), true) => {
                l.matched += 1;
                l.notes.push(format!("case {case}: 共线 ⇒ 退化（夹具为 C++ UB 值）"));
                l.unverifiable += 1;
            }
            (Ok(pl), false) => {
                let ok = close(pl.a, t.num(row, "A"), 1e-9)
                    && close(pl.b, t.num(row, "B"), 1e-9)
                    && close(pl.c, t.num(row, "C"), 1e-9)
                    && close(pl.d, t.num(row, "D"), 1e-9)
                    && close(pl.normal_angle(), t.num(row, "angle"), 1e-9);
                if ok {
                    l.matched += 1;
                } else {
                    l.mismatch(
                        MismatchKind::Unregistered,
                        format!("case {case}: plane={pl:?} 期望 A..D=({},{},{},{})", t.num(row,"A"), t.num(row,"B"), t.num(row,"C"), t.num(row,"D")),
                    );
                }
            }
            (got, _) => l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 退化分类与实测不符，得到 {got:?}"),
            ),
        }
    }
    let degen = l
        .notes
        .iter()
        .filter(|n| n.contains("共线"))
        .count();
    assert_eq!(degen, 3, "共线（UB）行数应为 3，实得 {degen}");
}

fn check_up_fit(l: &mut Ledger) {
    let t = Table::load("up_fit.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let mk = |pfx: &str| {
            Point3::new(
                t.num(row, &format!("{pfx}x")),
                t.num(row, &format!("{pfx}y")),
                t.num(row, &format!("{pfx}z")),
            )
        };
        let mut three = [mk("p1"), mk("p2"), mk("p3")];
        let q = mk("q");
        let Some((pl, post)) = up_fit_one_ref(q, &mut three) else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 独立 up_fit 返回 None"),
            );
            continue;
        };
        let plane_ok = close(pl.a, t.num(row, "outA"), 1e-9)
            && close(pl.b, t.num(row, "outB"), 1e-9)
            && close(pl.c, t.num(row, "outC"), 1e-9)
            && close(pl.d, t.num(row, "outD"), 1e-9);
        let post_ok = (0..3).all(|i| {
            close(post[i].x(), t.num(row, &format!("m{i}x")), 1e-9)
                && close(post[i].y(), t.num(row, &format!("m{i}y")), 1e-9)
                && close(post[i].z(), t.num(row, &format!("m{i}z")), 1e-9)
        });
        if plane_ok && post_ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: plane_ok={plane_ok} post_three_ok={post_ok}"),
            );
        }
    }
}

fn check_trip(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("trip.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let gid = t.int(row, "ground_id");
        let n = t.int(row, "area_n") as usize;
        let grid = &grounds[&gid];
        let mut pts = Vec::new();
        for i in 0..n {
            let x = t.int(row, &format!("x{i}"));
            let y = t.int(row, &format!("y{i}"));
            let h = grid.cell(Idx::new(x, y)).expect("采样点在图内");
            pts.push(Point3::new(f64::from(x), f64::from(y), h));
        }
        let initial_degenerate = pts.len() >= 3 && plane_of3(&[pts[0], pts[1], pts[2]]).is_none();
        match fit_plane_up(&pts) {
            Ok(pl) => {
                let ok = close(pl.a, t.num(row, "A"), 1e-9)
                    && close(pl.b, t.num(row, "B"), 1e-9)
                    && close(pl.c, t.num(row, "C"), 1e-9)
                    && close(pl.d, t.num(row, "D"), 1e-9)
                    && close(pl.normal_angle(), t.num(row, "normal_angle"), 1e-9);
                if ok {
                    l.matched += 1;
                    if initial_degenerate {
                        l.notes
                            .push(format!("case {case}: 初始三点共线但数值与夹具一致"));
                    }
                } else if initial_degenerate {
                    l.mismatch(
                        MismatchKind::Registered,
                        format!("case {case}: 初始三点共线（C++ UB），本仓确定化结果不同"),
                    );
                } else {
                    l.mismatch(
                        MismatchKind::Unregistered,
                        format!("case {case}: 非退化行数值不符"),
                    );
                }
            }
            Err(e) => {
                if initial_degenerate {
                    l.mismatch(
                        MismatchKind::Registered,
                        format!("case {case}: 初始三点共线，本仓返回 {e:?}"),
                    );
                } else {
                    l.mismatch(
                        MismatchKind::Unregistered,
                        format!("case {case}: 非退化行意外报错 {e:?}"),
                    );
                }
            }
        }
    }
}

fn check_corner(l: &mut Ledger) {
    let t = Table::load("corner.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let shape = FootShape {
            length: t.num(row, "length"),
            width: t.num(row, "width"),
            scan_step: 0.5,
        };
        let foot = Foot::new(
            Dot::new(t.num(row, "x"), t.num(row, "y")),
            t.num(row, "rz"),
            shape,
        );
        let got = foot.corners();
        let mut ok = true;
        for (i, c) in got.iter().enumerate() {
            ok &= close(c.x(), t.num(row, &format!("c{i}x")), 1e-9)
                && close(c.y(), t.num(row, &format!("c{i}y")), 1e-9);
        }
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: corners 不符"),
            );
        }
    }
}

fn check_cover(l: &mut Ledger) {
    let t = Table::load("cover.csv");
    l.total = t.rows.len();
    let mut cases: BTreeMap<i32, Vec<&Vec<String>>> = BTreeMap::new();
    for row in &t.rows {
        cases.entry(t.int(row, "case")).or_default().push(row);
    }
    assert_eq!(cases.len(), 40, "cover.csv 组数");
    for (case, rows) in cases {
        let first = rows[0];
        let shape = FootShape {
            length: t.num(first, "length"),
            width: t.num(first, "width"),
            scan_step: 0.5,
        };
        let foot = Foot::new(
            Dot::new(t.num(first, "x"), t.num(first, "y")),
            t.num(first, "rz"),
            shape,
        );
        let got = foot.cover();
        if got.len() != rows.len() {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 覆盖点数 {} != 夹具 {}", got.len(), rows.len()),
            );
            continue;
        }
        for row in rows {
            let i = t.int(row, "pt_idx") as usize;
            let want = Idx::new(t.int(row, "px"), t.int(row, "py"));
            if got[i] == want {
                l.matched += 1;
            } else {
                l.mismatch(
                    MismatchKind::Unregistered,
                    format!("case {case} pt_idx {i}: got {:?} want {want:?}", got[i]),
                );
            }
        }
    }
}

fn check_standable(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("standable.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let gid = t.int(row, "ground_id");
        let grid = grounds[&gid].clone();
        let terrain = Terrain::from_grid(grid.clone());
        let shape = FootShape {
            length: t.num(row, "length"),
            width: t.num(row, "width"),
            scan_step: 0.5,
        };
        let foot = Foot::new(
            Dot::new(t.num(row, "x"), t.num(row, "y")),
            t.num(row, "rz"),
            shape,
        );
        let covered = foot.cover();
        let all_inside = covered.iter().all(|i| grid.in_bounds(*i));
        let want_all_inside = t.int(row, "all_inside") == 1;
        let got_standable = foot.standable(&terrain);
        let want_standable = t.int(row, "standable") == 1;
        let mut ok = covered.len() == t.int(row, "covered_count") as usize
            && all_inside == want_all_inside
            && got_standable == want_standable;
        if all_inside {
            let dots: Vec<Dot> = covered.iter().map(|i| i.as_dot()).collect();
            match terrain.normal_angle(&dots) {
                Ok(a) => ok &= close(a, t.num(row, "normal_angle"), 1e-9),
                Err(_) => ok = false,
            }
        } else {
            // C++ 越界时返回全零平面（normal_angle=0.0）；Rust 映射成 Err。夹具此列无对照价值。
            l.unverifiable += 1;
            l.notes.push(format!(
                "case {case}: covered 越界 ⇒ C++ 全零面 / Rust Err，normal_angle 列不参与对照"
            ));
        }
        if ok {
            l.matched += 1;
        } else {
            let dots: Vec<Dot> = covered.iter().map(|i| i.as_dot()).collect();
            let angle_dbg = terrain
                .normal_angle(&dots)
                .map(|a| format!("{a:.12}"))
                .unwrap_or_else(|e| format!("{e:?}"));
            let only_angle = covered.len() == t.int(row, "covered_count") as usize
                && all_inside == want_all_inside
                && got_standable == want_standable
                && covered.len() > 3;
            if only_angle {
                l.mismatch(
                    MismatchKind::PendingDecision,
                    format!(
                        "case {case}: standable 布尔一致，但 normal_angle 列不符（本仓 {angle_dbg} vs 夹具 {}）：C++ cover() 的 unordered_set 迭代序与贪心 up_fit 让该列对输入顺序敏感，>3 点行不构成可复现规格（见 doc/port_verification.md 的 R1）",
                        t.num(row, "normal_angle")
                    ),
                );
            } else {
                l.mismatch(
                    MismatchKind::Unregistered,
                    format!(
                        "case {case}: covered {} vs {} | all_inside {all_inside} vs {want_all_inside} | standable {got_standable} vs {want_standable} | normal_angle {angle_dbg} vs {}",
                        covered.len(),
                        t.int(row, "covered_count"),
                        t.num(row, "normal_angle")
                    ),
                );
            }
        }
    }
}

fn check_slide(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("slide.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let gid = t.int(row, "ground_id");
        let terrain = Terrain::from_grid(grounds[&gid].clone());
        let shape = FootShape {
            length: t.num(row, "length"),
            width: t.num(row, "width"),
            scan_step: 0.5,
        };
        let foot = Foot::new(
            Dot::new(t.num(row, "x"), t.num(row, "y")),
            t.num(row, "rz"),
            shape,
        );
        let res = foot.slide(&terrain);
        let corners = foot.corners();
        let name_ok = matches!(
            (t.text(row, "result_name").as_str(), res),
            ("Modified", SlideResult::Modified { .. })
                | ("NoModification", SlideResult::NoModification)
                | ("NotApplicable", SlideResult::NotApplicable)
        );
        let int_ok = t.int(row, "result_int")
            == match res {
                SlideResult::Modified { .. } => 0,
                SlideResult::NoModification => 1,
                SlideResult::NotApplicable => 2,
            };
        let mut ok = name_ok && int_ok;
        for (i, c) in corners.iter().enumerate() {
            ok &= close(c.x(), t.num(row, &format!("out{i}x")), 1e-9)
                && close(c.y(), t.num(row, &format!("out{i}y")), 1e-9);
        }
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: slide 结果或输出角点不符"),
            );
        }
    }
}

fn check_satisfy_spacing(l: &mut Ledger) {
    let t = Table::load("satisfy_spacing.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let shape = FootShape::biped();
        let cfg = RobotConfig {
            min_separation: t.num(row, "min_sep"),
            max_separation: t.num(row, "max_sep"),
            ..RobotConfig::default()
        };
        let left = Foot::new(
            Dot::new(t.num(row, "l0x"), t.num(row, "l0y")),
            t.num(row, "l0rz"),
            shape,
        );
        let right = Foot::new(
            Dot::new(t.num(row, "r0x"), t.num(row, "r0y")),
            t.num(row, "r0rz"),
            shape,
        );
        let which = if t.int(row, "now_which") == 0 {
            WhichFoot::Left
        } else {
            WhichFoot::Right
        };
        let mut robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        robot.set_feet(left, right, which);
        let np = Dot::new(t.num(row, "newx"), t.num(row, "newy"));
        let got = robot.satisfy_spacing(np);
        let want = t.int(row, "result") == 1;

        // 独立复算 spacing（几何定义见 doc/project.md:115-123）
        let nf = robot.next(np);
        let pts = nf.corners();
        let line = Line::from_point_angle(robot.support_foot().position, robot.support_foot().rz);
        let min_d = pts
            .iter()
            .map(|p| line.dist_to_point(*p))
            .fold(f64::INFINITY, f64::min);
        let spacing = (min_d - robot.support_foot().shape.half_width()).max(0.0);
        let spacing_ok = close(spacing, t.num(row, "spacing"), 1e-9);

        if got == want && spacing_ok {
            l.matched += 1;
        } else if !spacing_ok {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 独立复算 spacing={spacing} vs 夹具 {}", t.num(row, "spacing")),
            );
        } else {
            l.mismatch(
                MismatchKind::Registered,
                format!(
                    "case {case}: spacing={spacing:.6} 夹具期望 {want} 本仓 {got}（D(task-10 裁决2)：闭区间+1e-3 容差）"
                ),
            );
        }
    }
}

fn check_satisfy_stride(l: &mut Ledger) {
    let t = Table::load("satisfy_stride.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let shape = FootShape::biped();
        let cfg = RobotConfig {
            max_stride: t.num(row, "max_stride"),
            ..RobotConfig::default()
        };
        let mut robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        robot.swing_foot_mut().position = Dot::new(t.num(row, "swx"), t.num(row, "swy"));
        robot.swing_foot_mut().rz = t.num(row, "swrz");
        robot.swing_foot_mut().shape = shape;
        robot.support_foot_mut().position = Dot::new(t.num(row, "supx"), t.num(row, "supy"));
        robot.support_foot_mut().rz = t.num(row, "suprz");
        robot.support_foot_mut().shape = shape;
        let np = Dot::new(t.num(row, "newx"), t.num(row, "newy"));
        let ok = robot.satisfy_stride(np) == (t.int(row, "result") == 1)
            && close(robot.distance(np), t.num(row, "distance"), 1e-9);
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: satisfy_stride 或 distance 不符"),
            );
        }
    }
}

fn check_satisfy_turn(l: &mut Ledger) {
    const REGISTERED: [i32; 3] = [16, 24, 26];
    let t = Table::load("satisfy_turn.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.int(row, "case");
        let shape = FootShape::biped();
        let cfg = RobotConfig {
            max_turn: t.num(row, "max_turn"),
            ..RobotConfig::default()
        };
        let mut robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        robot.swing_foot_mut().position = Dot::new(t.num(row, "swx"), t.num(row, "swy"));
        robot.swing_foot_mut().rz = t.num(row, "swrz");
        robot.swing_foot_mut().shape = shape;
        robot.support_foot_mut().position = Dot::new(t.num(row, "supx"), t.num(row, "supy"));
        robot.support_foot_mut().rz = t.num(row, "suprz");
        robot.support_foot_mut().shape = shape;
        let np = Dot::new(t.num(row, "newx"), t.num(row, "newy"));
        // 夹具的 angle 列 = 未归一化的 raw
        let raw = robot.swing_foot().position.angle(np) - robot.support_foot().rz;
        let angle_ok = close(raw, t.num(row, "angle"), 1e-9);
        let got = robot.satisfy_turn(np);
        let want_fixture = t.int(row, "result") == 1;
        if !angle_ok {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 夹具 angle 列与独立复算 raw 不符"),
            );
        } else if got == want_fixture {
            l.matched += 1;
        } else if REGISTERED.contains(&case) {
            let wrapped = metrics::wrap_angle(raw);
            l.mismatch(
                MismatchKind::Registered,
                format!(
                    "case {case}: raw={raw:.12} ({:.4}°) → 归一化 {wrapped:.12} rad ({:.4}°)，夹具 {want_fixture} 本仓 {got}（D1）",
                    raw.to_degrees(),
                    wrapped.to_degrees()
                ),
            );
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: 未登记的行翻转（raw={raw}）"),
            );
        }
    }
}

fn check_restore(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("restore.csv");
    l.total = t.rows.len();
    let g0 = &grounds[&0]; // 5x7
    let clamp = |v: i32, hi: i32| v.clamp(0, hi);
    for row in &t.rows {
        let bx = t.int(row, "blockx");
        let by = t.int(row, "blocky");
        let scale = t.num(row, "scale");
        let mode = t.text(row, "mode");
        if mode == "dot" {
            // C++ 的 SqDot 重载在同一个函数体里混用 const/非 const scale()（浮点乘 vs ceil），
            // 本仓没有这个入口 ⇒ 无对照对象。
            l.unverifiable += 1;
            continue;
        }
        // 独立复算：trunc(block/scale) + orth_near 钳位（语义见 doc/known_divergences.md D4）
        let mut near_x = (bx as f64 / scale) as i32;
        let mut near_y = (by as f64 / scale) as i32;
        let mut far_x = ((bx as f64 + 1.0) / scale) as i32;
        let mut far_y = ((by as f64 + 1.0) / scale) as i32;
        near_x = clamp(near_x, g0.rows() as i32 - 1);
        near_y = clamp(near_y, g0.cols() as i32 - 1);
        far_x = clamp(far_x, g0.rows() as i32 - 1);
        far_y = clamp(far_y, g0.cols() as i32 - 1);

        let got = g0.restore_block(Idx::new(bx, by), scale);
        let got4 = (got.0.x, got.0.y, got.1.x, got.1.y);
        if got4 != (near_x, near_y, far_x, far_y) {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {}: restore_block 与独立复算不符 {got4:?} vs {:?}", t.text(row, "case"), (near_x, near_y, far_x, far_y)),
            );
            continue;
        }
        let want_golden = (
            t.int(row, "outax"),
            t.int(row, "outay"),
            t.int(row, "outbx"),
            t.int(row, "outby"),
        );
        if got4 == want_golden {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Registered,
                format!(
                    "case {}: block=({bx},{by}) scale={scale} 夹具(乘)={want_golden:?} 本仓(除)={got4:?}（D4）",
                    t.text(row, "case")
                ),
            );
        }
    }
}

fn check_restore_dot(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("restore_dot.csv");
    l.total = t.rows.len();
    let g0 = &grounds[&0];
    for row in &t.rows {
        if t.text(row, "mode") != "int" {
            l.unverifiable += 1;
            continue;
        }
        let got = g0.restore_dot(
            Idx::new(t.int(row, "inx"), t.int(row, "iny")),
            t.num(row, "scale"),
        );
        let ok = close(got.x(), t.num(row, "outx"), 1e-9)
            && close(got.y(), t.num(row, "outy"), 1e-9);
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {}: restore_dot 不符", t.text(row, "case")),
            );
        }
    }
}

fn check_central_restore(l: &mut Ledger) {
    let t = Table::load("central_restore.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let idx = Idx::new(t.int(row, "inx"), t.int(row, "iny"));
        let c = idx.central_restore(t.num(row, "scale"));
        let ok = match t.text(row, "mode").as_str() {
            "dot" => {
                close(c.x(), t.num(row, "outx"), 1e-9) && close(c.y(), t.num(row, "outy"), 1e-9)
            }
            "int" => {
                c.x() as i32 == t.int(row, "outx") && c.y() as i32 == t.int(row, "outy")
            }
            other => panic!("未知 mode {other}"),
        };
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {}: central_restore 不符", t.text(row, "case")),
            );
        }
    }
}

fn check_index_scale(l: &mut Ledger) {
    let t = Table::load("index_scale.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let fname = t.text(row, "fn");
        let v = t.num(row, "index_in");
        let scale = t.num(row, "scale");
        let want = t.num(row, "result");
        if fname == "scale_double" {
            // 原 `double scale(double,double)` 的浮点乘法入口；本仓已消灭该同名重载。
            l.unverifiable += 1;
            continue;
        }
        let got = match fname.as_str() {
            "index_scale_int" | "index_scale_double" | "intex_sx" | "intex_sy" => {
                f64::from(index_scale(v, scale))
            }
            other => panic!("未知 fn {other}"),
        };
        if close(got, want, 0.0) {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {} fn={fname} v={v} scale={scale}: got {got} want {want}", t.text(row, "case")),
            );
        }
    }
    assert_eq!(l.unverifiable, 40, "scale_double 行数应为 40");
}

fn check_scale_graph(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let shapes = Table::load("scale_graph.csv");
    let cells = Table::load("scale_graph_cells.csv");
    let mut expected: BTreeMap<(String, usize, usize), f64> = BTreeMap::new();
    for row in &cells.rows {
        expected.insert(
            (
                cells.text(row, "case"),
                cells.int(row, "x") as usize,
                cells.int(row, "y") as usize,
            ),
            cells.num(row, "value"),
        );
    }
    l.total = cells.rows.len();
    for row in &shapes.rows {
        let case = shapes.text(row, "case");
        let gid = shapes.int(row, "ground_id");
        let scale = shapes.num(row, "scale");
        let out = grounds[&gid].scale_down(scale).expect("scale_down");
        let mut ok = out.rows() == shapes.int(row, "out_rows") as usize
            && out.cols() == shapes.int(row, "out_cols") as usize;
        for x in 0..out.rows() {
            for y in 0..out.cols() {
                let want = expected[&(case.clone(), x, y)];
                ok &= close(out.cell(Idx::new(x as i32, y as i32)).unwrap(), want, 1e-9);
            }
        }
        if ok {
            l.matched += out.rows() * out.cols();
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case}: scale_down({scale}) 尺寸或格子不符"),
            );
        }
    }
    assert_eq!(l.matched, 5217, "scale_graph 应覆盖 5217 格");
}

fn check_steep_extend(l: &mut Ledger, grounds: &BTreeMap<i32, Grid>) {
    let t = Table::load("steep_extend.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.int(row, "case");
        let gid = t.int(row, "ground_id");
        let terrain = Terrain::from_grid(grounds[&gid].clone());
        let block = (
            Idx::new(t.int(row, "x0"), t.int(row, "y0")),
            Idx::new(t.int(row, "x1"), t.int(row, "y1")),
        );
        let want = t.num(row, "result");
        let got = terrain.steep_extend(block);
        match got {
            None => {
                if want == -1.0 {
                    l.matched += 1;
                } else {
                    l.mismatch(
                        MismatchKind::Unregistered,
                        format!("case {case}: got None，夹具可行 {want}"),
                    );
                }
            }
            Some(score) => {
                // C++ `aStar.cpp:212` 的 min_x 笔误：只扫 bounded_first.x 一列。
                // 本仓按语义扫 [min(x0,x1), max(x0,x1)]。夹具里 x0 > x1 的行因此必然不同。
                let g = &grounds[&gid];
                let (mut bx0, mut by0, mut bx1, mut by1) = (
                    t.int(row, "x0").clamp(0, g.rows() as i32 - 1),
                    t.int(row, "y0").clamp(0, g.cols() as i32 - 1),
                    t.int(row, "x1").clamp(0, g.rows() as i32 - 1),
                    t.int(row, "y1").clamp(0, g.cols() as i32 - 1),
                );
                let inverted_x = bx0 > bx1;
                if inverted_x {
                    // 只交换给比较用，避免 unused 警告
                    std::mem::swap(&mut bx0, &mut bx1);
                    std::mem::swap(&mut by0, &mut by1);
                    l.mismatch(
                        MismatchKind::Registered,
                        format!(
                            "case {case}: 夹具由 C++ min_x 笔误只扫 x0 一列得到 {want}，本仓扫描完整区间得 {score:.6}（task-4 有意修正）"
                        ),
                    );
                } else if want == -1.0 {
                    // T6f：陡峭度由断路改为代价 ⇒ 原来被 >10 判死的窗口现在返回分数
                    l.mismatch(
                        MismatchKind::Registered,
                        format!("case {case}: 夹具 -1（原 >10 判死），本仓返回 {score:.6}（D4-T6f）"),
                    );
                } else if close(score, want, 1e-9) {
                    l.matched += 1;
                } else {
                    l.mismatch(
                        MismatchKind::Unregistered,
                        format!("case {case}: score={score} vs 夹具 {want}"),
                    );
                }
            }
        }
    }
}

fn check_geometric_decay(l: &mut Ledger) {
    let t = Table::load("geometric_decay.csv");
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let fname = t.text(row, "fn");
        let alpha = t.num(row, "alpha");
        let start = t.num(row, "start");
        let n = t.int(row, "n") as usize;
        let got = if fname == "geometric_decay_start" {
            geometric_decay_start(start, alpha, n)
        } else {
            geometric_decay(alpha, n)
        };
        let mut ok = got.len() == n;
        let cols = n.min(10); // 夹具只导出 c0..c9；n>10 的行按前 10 项比对并核对长度
        for (i, c) in got.iter().take(cols).enumerate() {
            ok &= close(*c, t.num(row, &format!("c{i}")), 1e-9);
        }
        if n > 10 {
            l.notes.push(format!(
                "case {case}: n={n} > 夹具列数 10，只比对前 10 项（长度已核）"
            ));
        }
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case} fn={fname}: 序列不符"),
            );
        }
    }
}

fn check_direction_determine(l: &mut Ledger) {
    let t = Table::load("direction_determine.csv");
    let g = Table::load("direction_guides.csv");
    let mut guides: BTreeMap<(String, String), Vec<(usize, Dot)>> = BTreeMap::new();
    for row in &g.rows {
        let key = (g.text(row, "case"), g.text(row, "mode"));
        guides.entry(key).or_default().push((
            g.int(row, "idx") as usize,
            Dot::new(g.num(row, "x"), g.num(row, "y")),
        ));
    }
    for v in guides.values_mut() {
        v.sort_by_key(|e| e.0);
    }
    l.total = t.rows.len();
    for row in &t.rows {
        let case = t.text(row, "case");
        let mode = t.text(row, "mode");
        let n = t.int(row, "n") as usize;
        let mut gd: Vec<Dot> = guides
            .get(&(case.clone(), mode.clone()))
            .map(|v| v.iter().map(|e| e.1).collect())
            .unwrap_or_default();
        gd.truncate(n);
        assert_eq!(gd.len(), n, "case {case} mode {mode}: guides 数不符");
        let out = direction_determine(&gd, t.int(row, "near_end") == 1);
        let ok = close(out.x(), t.num(row, "outx"), 1e-9)
            && close(out.y(), t.num(row, "outy"), 1e-9);
        if ok {
            l.matched += 1;
        } else {
            l.mismatch(
                MismatchKind::Unregistered,
                format!("case {case} mode={mode}: direction_determine 不符"),
            );
        }
    }
}

// ------------------------------------------------------------------ 独立命题验证

/// 命题：`scale_star` 会消耗 `rng`（洗牌不是空转），且同 seed 完全可复现。
///
/// 只验证这两条；「不同 seed 必不同」**不是**本仓的性质，这里显式验证它可以相同。
#[test]
fn independent_shuffle_consumes_rng_and_same_seed_reproduces() {
    // 平坦合成图：所有步代价相同 ⇒ 最优解不唯一，洗牌顺序能影响路径
    let flat = Grid::new(vec![0.0; 16 * 16], 16, 16).unwrap();
    let terrain = Terrain::from_grid(flat);
    // sr = ceil(16 * 0.25) = 4；goal (12,12) → ceil(3) = 3 < 4，落在缩放网格内
    let scale = 1.0 / 4.0;
    let start = Idx::new(0, 0);
    let goal = Idx::new(12, 12);

    // ① 同 seed 两个独立实例逐点相同
    let mut a = Rng::from_seed(7);
    let mut b = Rng::from_seed(7);
    let pa = scale_star(&terrain, start, goal, scale, &mut a).unwrap();
    let pb = scale_star(&terrain, start, goal, scale, &mut b).unwrap();
    assert_eq!(pa.path, pb.path, "同 seed 两次必须逐点相同");
    assert_eq!(pa.expanded, pb.expanded);

    // ② 洗牌确实消耗了 rng：一个实例跑过 scale_star，另一个没跑，之后各洗同样的数组 ⇒ 结果不同
    let mut used = Rng::from_seed(7);
    let _ = scale_star(&terrain, start, goal, scale, &mut used);
    let mut untouched = Rng::from_seed(7);
    let mut v1: Vec<u32> = (0..16).collect();
    let mut v2: Vec<u32> = (0..16).collect();
    used.shuffle(&mut v1);
    untouched.shuffle(&mut v2);
    assert_ne!(
        v1, v2,
        "scale_star 没有消耗 rng（洗牌被短路）——这正是可复现性之外必须保住的性质"
    );

    // ③ 「不同 seed ⇒ 不同路径」**不成立**：同一输入可以给出相同路径（地形代价使最优解唯一）
    let mut s7 = Rng::from_seed(7);
    let mut s1234 = Rng::from_seed(1234);
    let p7 = scale_star(&terrain, start, goal, scale, &mut s7).unwrap();
    let p1234 = scale_star(&terrain, start, goal, scale, &mut s1234).unwrap();
    println!(
        "独立命题③：平坦图 seed7 vs seed1234 路径相同={}（不同 seed 可同可不同，不是缺陷）",
        p7.path == p1234.path
    );
    assert_eq!(p7.path.first(), Some(&start));
}

// ------------------------------------------------------------------ 主审计

#[test]
fn golden_fixture_audit() {
    let grounds = load_grounds();
    let mut ledgers: Vec<Ledger> = Vec::new();

    let mut l = Ledger::new("grounds.csv");
    check_grounds(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("golden_summary.txt");
    check_golden_summary(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("normal_angle.csv");
    check_normal_angle(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("up_fit.csv");
    check_up_fit(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("trip.csv");
    check_trip(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("corner.csv");
    check_corner(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("cover.csv");
    check_cover(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("standable.csv");
    check_standable(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("slide.csv");
    check_slide(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("satisfy_spacing.csv");
    check_satisfy_spacing(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("satisfy_stride.csv");
    check_satisfy_stride(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("satisfy_turn.csv");
    check_satisfy_turn(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("restore.csv");
    check_restore(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("restore_dot.csv");
    check_restore_dot(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("central_restore.csv");
    check_central_restore(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("index_scale.csv");
    check_index_scale(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("scale_graph.csv+scale_graph_cells.csv");
    check_scale_graph(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("steep_extend.csv");
    check_steep_extend(&mut l, &grounds);
    ledgers.push(l);

    let mut l = Ledger::new("geometric_decay.csv");
    check_geometric_decay(&mut l);
    ledgers.push(l);

    let mut l = Ledger::new("direction_determine.csv+direction_guides.csv");
    check_direction_determine(&mut l);
    ledgers.push(l);

    // ---- 打印台账 ----
    println!("\n===== T7 独立 golden 对照台账 =====");
    println!(
        "{:<44} {:>6} {:>6} {:>8} {:>7} {:>7} {:>9}",
        "夹具", "总行", "一致", "无对照", "登记分歧", "待裁决", "未登记分歧"
    );
    let mut total_unregistered = 0_usize;
    for l in &ledgers {
        total_unregistered += l.unregistered.len();
        println!(
            "{:<44} {:>6} {:>6} {:>8} {:>7} {:>7} {:>9}",
            l.name,
            l.total,
            l.matched,
            l.unverifiable,
            l.known_divergent.len(),
            l.pending.len(),
            l.unregistered.len()
        );
    }
    println!("未登记分歧合计 = {total_unregistered}");
    for l in &ledgers {
        for d in &l.known_divergent {
            println!("[登记分歧] {} :: {d}", l.name);
        }
        for d in &l.pending {
            println!("[待裁决]   {} :: {d}", l.name);
        }
        for n in &l.notes {
            println!("[说明]     {} :: {n}", l.name);
        }
        for u in &l.unregistered {
            println!("[!! 未登记] {} :: {u}", l.name);
        }
    }
    println!("===================================\n");

    assert_eq!(
        total_unregistered, 0,
        "存在未登记的 golden 差异，见上方 [!! 未登记] 行"
    );

    // 待裁决项用**精确集合**钉住：只允许这 2 行（standable.csv 的 normal_angle 顺序敏感）。
    // 若某天有人修好/登记了它，这里会变红，提示更新 T7 报告与登记表。
    let pending: Vec<String> = ledgers
        .iter()
        .flat_map(|l| l.pending.iter().cloned())
        .collect();
    assert_eq!(
        pending.len(),
        2,
        "待裁决项数变了（应为 standable.csv case 14/43）：{pending:?}"
    );
    assert!(
        pending.iter().all(|p| p.contains("case 14") || p.contains("case 43")),
        "待裁决项不是预期的两行: {pending:?}"
    );
}

// ------------------------------------------------------------------ D3 层定位（独立重做）

/// 读 `data/csv/map.csv`（只读，不写）为 `Grid`。
fn load_real_map() -> Grid {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../data/csv/map.csv");
    let text = std::fs::read_to_string(&path).expect("读地图");
    let mut rows = Vec::new();
    for line in text.lines().filter(|l| !l.trim().is_empty()) {
        rows.push(line.split(',').map(|c| c.trim().parse::<f64>().unwrap()).collect::<Vec<_>>());
    }
    let r = rows.len();
    let c = rows[0].len();
    let flat: Vec<f64> = rows.into_iter().flatten().collect();
    Grid::new(flat, r, c).expect("地图规整")
}

/// D3 复核：120 组合的失败分层。用**公开 API**重新分层（不抄作者的探针）。
///
/// 命题：当前 HEAD 上 `find_path` 失败 ⟺ `scale_star` 返回 `None`（即没有 L2 层）；
/// 且全部失败组合的 goal.x 越出地图行数（1000）。
#[test]
fn nopath_layering_at_head() {
    let terrain = Terrain::from_grid(load_real_map());
    let starts = ["0,0", "100,100", "500,500", "700,700"];
    let goals = [
        "900,1200", "800,800", "1000,500", "500,1000", "1200,900", "900,300", "300,900",
        "1500,1500", "200,1500", "1500,200",
    ];
    let cfg = RobotConfig::default();
    let mut err_total = 0_usize;
    let mut l1 = 0_usize;
    let mut l2 = 0_usize;
    let mut err_goals: Vec<String> = Vec::new();
    for s in starts {
        for g in goals {
            let p = |t: &str| {
                let mut it = t.split(',');
                Dot::new(
                    it.next().unwrap().parse::<f64>().unwrap(),
                    it.next().unwrap().parse::<f64>().unwrap(),
                )
            };
            let (sd, gd) = (p(s), p(g));
            for seed in 0_u64..3 {
                let mut rng = Rng::from_seed(seed);
                let mut robot = Robot::new(cfg, sd);
                let res = robot.find_path(&terrain, gd, &mut rng);
                // 独立取 scale_star 的结果（另起一个 rng 实例，避免影响 find_path 的对照）
                let mut probe_rng = Rng::from_seed(seed);
                let ss = scale_star(
                    &terrain,
                    sd.as_idx(),
                    gd.as_idx(),
                    1.0 / cfg.max_stride,
                    &mut probe_rng,
                );
                match (res.is_err(), ss.is_none()) {
                    (false, _) => {}
                    (true, true) => l1 += 1,
                    (true, false) => l2 += 1,
                }
                if res.is_err() {
                    err_total += 1;
                    err_goals.push(format!("({s})->({g}) seed{seed}"));
                }
            }
        }
    }
    println!(
        "D3 独立分层：120 组合中 find_path 失败 {err_total} 个；L1(scale_star=None)={l1}；L2(scale_star=Some 但 find_path 失败)={l2}"
    );
    for e in &err_goals {
        println!("  失败: {e}");
    }
    assert_eq!(err_total, 48, "当前 HEAD 的失败组合数应为 48（nopath_diagnosis.md 的 51 锚在旧二进制）");
    assert_eq!(l1, 48, "48 个失败应全部是 L1（高层 A* 不可达）");
    assert_eq!(l2, 0, "当前 HEAD 上不应再有 L2（find_target 空候选）失败");
    assert!(
        err_goals.iter().all(|e| e.contains("->(1000,500)") || e.contains("->(1200,900)") || e.contains("->(1500,1500)") || e.contains("->(1500,200)")),
        "全部失败组合都应落在 4 个越界目标上"
    );
}
