//! `geom.rs` / `grid.rs` 的语义钉死 + C++ 函数级 golden 对照。
//!
//! 三部分：
//! 1. **源码语义测试**：每条都注明它钉住的 C++ 出处（`文件:行`），防止后来人按「看起来对」改实现；
//! 2. **golden 对照**：喂 `tests/golden/*.csv` 的输入列，比输出列，容差 1e-9；
//! 3. 边界 / 拒绝路径（`Grid::new` 形状、非法 scale、越界、退化平面）。
//!
//! 夹具一律**按表头名**取值（价值：夹具只改值或重排列时测试不散架）。

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
use std::path::PathBuf;

use approx::assert_relative_eq;
use trapla_core::astar::scale_star;
use trapla_core::error::CoreError;
use trapla_core::geom::{Dir4, Dot, Idx, Line, Plane, Point3, fit_plane_up, index_scale};
use trapla_core::grid::{Grid, OBSTACLE};
use trapla_core::rng::Rng;
use trapla_core::terrain::Terrain;

// ---------------------------------------------------------------- 夹具读取

fn parse_f64(raw: &str) -> f64 {
    match raw.trim() {
        "inf" => f64::INFINITY,
        "-inf" => f64::NEG_INFINITY,
        "nan" => f64::NAN,
        other => other
            .parse::<f64>()
            .unwrap_or_else(|_| panic!("golden 夹具里出现非浮点值 {other:?}")),
    }
}

/// 按表头名取值的极简 CSV 读取器（夹具里没有引号/逗号转义）。
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
            .unwrap_or_else(|e| panic!("读不到 golden 夹具 {}: {e}", path.display()));
        let mut lines = text.lines().filter(|l| !l.trim().is_empty());
        let header: Vec<String> = lines
            .next()
            .unwrap_or_else(|| panic!("{} 是空文件", path.display()))
            .split(',')
            .map(|c| c.trim().to_string())
            .collect();
        let columns = header
            .iter()
            .enumerate()
            .map(|(i, name)| (name.clone(), i))
            .collect();
        let rows = lines
            .map(|l| l.split(',').map(|c| c.trim().to_string()).collect())
            .collect();
        Self { columns, rows }
    }

    fn has(&self, column: &str) -> bool {
        self.columns.contains_key(column)
    }

    fn text(&self, row: &[String], column: &str) -> String {
        let i = *self
            .columns
            .get(column)
            .unwrap_or_else(|| panic!("golden 夹具缺列 {column}"));
        row.get(i)
            .unwrap_or_else(|| panic!("一行里没有第 {i} 列（列 {column}）"))
            .clone()
    }

    fn num(&self, row: &[String], column: &str) -> f64 {
        parse_f64(&self.text(row, column))
    }
}

/// golden 容差：1e-9（相对 + 绝对底），非有限值按逐位比较。
fn assert_close(got: f64, want: f64) {
    if want.is_nan() {
        assert!(got.is_nan(), "want NaN, got {got}");
        return;
    }
    if want.is_infinite() {
        assert_eq!(got, want, "want {want}, got {got}");
        return;
    }
    assert_relative_eq!(got, want, epsilon = 1e-9, max_relative = 1e-9);
}

fn assert_dot_close(got: Dot, want: Dot) {
    assert_close(got.x(), want.x());
    assert_close(got.y(), want.y());
}

/// 三维点是否共线（法向全零）—— **只用来给 `trip.csv` / `normal_angle.csv` 里
/// 「初始三点退化」的行归类**（那些行的 C++ 输出来自未初始化成员），不充当期望值 oracle。
fn collinear(a: Point3, b: Point3, c: Point3) -> bool {
    let v1 = [b.x() - a.x(), b.y() - a.y(), b.z() - a.z()];
    let v2 = [c.x() - a.x(), c.y() - a.y(), c.z() - a.z()];
    let nx = v1[1] * v2[2] - v1[2] * v2[1];
    let ny = v1[2] * v2[0] - v1[0] * v2[2];
    let nz = v1[0] * v2[1] - v1[1] * v2[0];
    nx == 0.0 && ny == 0.0 && nz == 0.0
}

/// 平面在一点上的取值（`a*x + b*y + c*z + d`，`CuPlane::get_pos` 的分子，
/// `src/utils/geometry.cpp:1115`）。用于检查「没有任何点在平面上方」这条后置条件。
fn plane_value(plane: Plane, p: Point3) -> f64 {
    plane.a * p.x() + plane.b * p.y() + plane.c * p.z() + plane.d
}

/// 从 `grounds.csv` 造出与 `golden_dump.cpp` 完全一致的 7 张图（key = ground_id）。
fn load_grounds() -> BTreeMap<i32, Grid> {
    let table = Table::load("grounds.csv");
    let mut entries: BTreeMap<i32, Vec<(usize, usize, f64)>> = BTreeMap::new();
    for row in &table.rows {
        let gid = table.num(row, "ground_id") as i32;
        let x = table.num(row, "x") as usize;
        let y = table.num(row, "y") as usize;
        let h = table.num(row, "height");
        entries.entry(gid).or_default().push((x, y, h));
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
            (
                gid,
                Grid::new(data, rows, cols).expect("grounds.csv 构造的图必须形状一致"),
            )
        })
        .collect()
}

// ---------------------------------------------------------------- 语义：索引与缩放

/// `SqDot::x_index/y_index/as_index`（`geometry.cpp:97-108,354-356`）：**四舍五入**，
/// 既不是截断也不是 `ceil`。
#[test]
fn dot_as_idx_is_round_half_away_from_zero() {
    assert_eq!(Dot::new(2.9, 2.9).as_idx(), Idx::new(3, 3)); // 截断会给 2
    assert_eq!(Dot::new(2.4, 2.4).as_idx(), Idx::new(2, 2)); // ceil 会给 3
    assert_eq!(Dot::new(2.5, -2.5).as_idx(), Idx::new(3, -3)); // 半值远离零（非银行家舍入）
    assert_eq!(Dot::new(-2.4, -2.4).as_idx(), Idx::new(-2, -2));
    assert_eq!(Dot::new(0.5, -0.5).as_idx(), Idx::new(1, -1));
    assert_eq!(Dot::new(-0.4, 0.4).as_idx(), Idx::new(0, 0));
    assert_eq!(Dot::new(6.5, 7.5).as_idx(), Idx::new(7, 8));
}

/// `index_scale`（`scale.cpp:12`）：`ceil(v * scale)`，含 0 / 负数 / 恰好整数三种边界。
#[test]
fn index_scale_ceil_boundaries() {
    assert_eq!(index_scale(0.0, 0.025), 0);
    assert_eq!(index_scale(0.0, 1.0), 0);
    assert_eq!(index_scale(-1.0, 0.025), 0); // ceil(-0.025) = -0.0 → 0
    assert_eq!(index_scale(-1.0, 0.5), 0);
    assert_eq!(index_scale(-1.0, 1.0), -1);
    assert_eq!(index_scale(-1.0, 2.0), -2);
    assert_eq!(index_scale(-7.0, 0.025), 0);
    assert_eq!(index_scale(0.5, 1.0), 1);
    assert_eq!(index_scale(-0.5, 1.0), 0);
    // 乘积恰为整数：ceil 不会再多加一（`40 * 0.025` 在 IEEE double 下正好是 1.0）
    assert_eq!(index_scale(40.0, 0.025), 1);
    assert_eq!(index_scale(40.0, 0.5), 20);
    assert_eq!(index_scale(40.0, 1.0), 40);
    assert_eq!(index_scale(1000.0, 2.0), 2000);
}

/// `index_scale.csv` 全部 220 行的输入→输出对照（`index_scale_int/_double/intex_sx/intex_sy`
/// 在原仓是同一个 `ceil` 函数的四个入口；`scale_double` 是已消灭的浮点同名重载，见下）。
#[test]
fn golden_index_scale_rows() {
    let table = Table::load("index_scale.csv");
    let mut checked = 0_usize;
    let mut eliminated_float_scale = 0_usize;
    for row in &table.rows {
        let function = table.text(row, "fn");
        if function == "scale_double" {
            // 原 `double scale(double, double)`（`scale.cpp:20-22`）—— 与 `index_scale` 同名不同义，
            // 本重写里没有这个入口（浮点缩放只在 `Idx::central_restore`，且语义不同）。
            eliminated_float_scale += 1;
            continue;
        }
        let value = table.num(row, "index_in");
        let scale = table.num(row, "scale");
        assert_eq!(
            f64::from(index_scale(value, scale)),
            table.num(row, "result"),
            "fn={function} value={value} scale={scale}"
        );
        checked += 1;
    }
    assert!(checked >= 150, "只核对了 {checked} 行，夹具是不是换了？");
    assert_eq!(
        eliminated_float_scale, 40,
        "scale_double 行数变了，请复核夹具"
    );
}

/// `central_restore.csv`：`dot` 行 = 不截断的浮点中心（本仓 `Idx::central_restore`），
/// `int` 行 = 同一公式截断向零的结果（原 `Intex::central_restore`，`index.cpp:108-111`）。
#[test]
fn golden_central_restore_rows() {
    let table = Table::load("central_restore.csv");
    let mut dot_rows = 0_usize;
    let mut int_rows = 0_usize;
    for row in &table.rows {
        let idx = Idx::new(table.num(row, "inx") as i32, table.num(row, "iny") as i32);
        let center = idx.central_restore(table.num(row, "scale"));
        match table.text(row, "mode").as_str() {
            "dot" => {
                assert_close(center.x(), table.num(row, "outx"));
                assert_close(center.y(), table.num(row, "outy"));
                dot_rows += 1;
            }
            "int" => {
                assert_eq!(center.x() as i32, table.num(row, "outx") as i32);
                assert_eq!(center.y() as i32, table.num(row, "outy") as i32);
                int_rows += 1;
            }
            other => panic!("未知 mode {other}"),
        }
    }
    assert_eq!((dot_rows, int_rows), (35, 35));
}

/// `restore_dot.csv` 的 `int` 行：原 `SqPlane::restore_dot(Intex&, scale)`（`geometry.cpp:908-910`）。
/// `Grid::restore_dot` 就是它，且**委托** `Idx::central_restore`（公式只有一处）。
#[test]
fn golden_restore_dot_rows_and_delegation() {
    let table = Table::load("restore_dot.csv");
    let grounds = load_grounds();
    let g0 = &grounds[&0]; // 5x7
    assert_eq!((g0.rows(), g0.cols()), (5, 7));
    let mut checked = 0_usize;
    for row in &table.rows {
        if table.text(row, "mode") != "int" {
            continue; // dot 行是 SqDot 重载，本仓入口对应 Intex 重载
        }
        let block = Idx::new(table.num(row, "inx") as i32, table.num(row, "iny") as i32);
        let scale = table.num(row, "scale");
        assert_dot_close(
            g0.restore_dot(block, scale),
            Dot::new(table.num(row, "outx"), table.num(row, "outy")),
        );
        checked += 1;
    }
    assert_eq!(checked, 30);

    // 委托关系（可判定）：restore_dot == 截断向零(central_restore) 再钳到本图边界
    let block = Idx::new(43, 10);
    let center = block.central_restore(0.025);
    assert_close(center.x(), 1740.0);
    assert_close(center.y(), 420.0);
    assert_dot_close(g0.restore_dot(block, 0.025), Dot::new(4.0, 6.0)); // 钳位到 rows-1 / cols-1
    let negative = Idx::new(-2, 5).central_restore(0.025);
    assert_close(negative.x(), -60.0);
    assert_close(negative.y(), 220.0);
    assert_dot_close(g0.restore_dot(Idx::new(-2, 5), 0.025), Dot::new(0.0, 6.0));
}

/// `restore.csv`：原 `SqPlane::restore(const Intex&, scale)`（`src/utils/geometry.cpp:925-929`）。
///
/// **有意分歧（D4，登记见 `doc/known_divergences.md`）**：夹具记的是原实现的**乘**方向
/// （`ceil(block * scale)`），本仓修正为**除**（`trunc(block / scale)`，与 `scale_down` /
/// `central_restore` 同向）。夹具**一个字节都没有改**；20 组里 8 组新旧恰好一致
/// （全部 `scale = 1.0` 的行，以及两端都被钳到边界的行），12 组有意分歧。
///
/// 表里的期望值是对 `trunc` + `orth_near` 钳位的**独立复算**（算式见 D4），不是从被测函数抄的；
/// 同时逐行断言夹具仍是**旧值那一份** —— 夹具一旦被改成新语义，这里立刻变红。
#[test]
fn golden_restore_block_rows_register_direction_fix() {
    // (blockx, blocky, scale, 修后期的 (near_x, near_y, far_x, far_y))，独立复算得到
    type Corners = (i32, i32, i32, i32);
    const CORRECTED: [(i32, i32, f64, Corners); 20] = [
        (26, 13, 1.0, (4, 6, 4, 6)),
        (26, 26, 1.0, (4, 6, 4, 6)),
        (-2, 8, 1.0, (0, 6, 0, 6)),
        (-3, -1, 0.025, (0, 0, 0, 0)),
        (7, 12, 0.5, (4, 6, 4, 6)),
        (0, 17, 1.0, (0, 6, 1, 6)),
        (15, -2, 0.5, (4, 0, 4, 0)),
        (5, 27, 1.0, (4, 6, 4, 6)),
        (28, 17, 0.025, (4, 6, 4, 6)),
        (3, 7, 0.5, (4, 6, 4, 6)),
        (23, 27, 0.025, (4, 6, 4, 6)),
        (8, 11, 0.025, (4, 6, 4, 6)),
        (8, 1, 0.5, (4, 2, 4, 4)),
        (20, 16, 0.025, (4, 6, 4, 6)),
        (22, 4, 0.025, (4, 6, 4, 6)),
        (22, 3, 0.5, (4, 6, 4, 6)),
        (6, 8, 0.025, (4, 6, 4, 6)),
        (0, 7, 0.025, (0, 6, 4, 6)),
        (30, 10, 0.025, (4, 6, 4, 6)),
        (23, 24, 0.025, (4, 6, 4, 6)),
    ];
    let table = Table::load("restore.csv");
    let grounds = load_grounds();
    let g0 = &grounds[&0]; // 5x7

    let expected: BTreeMap<(i32, i32), (f64, Corners)> = CORRECTED
        .iter()
        .map(|&(bx, by, scale, corners)| ((bx, by), (scale, corners)))
        .collect();

    let mut same_as_golden = 0_usize;
    let mut divergent = 0_usize;
    let mut dot_equals_int = 0_usize;
    let mut golden_int = BTreeMap::new();
    let mut golden_dot = BTreeMap::new();
    for row in &table.rows {
        let key = (
            table.num(row, "blockx") as i32,
            table.num(row, "blocky") as i32,
        );
        let corners = (
            table.num(row, "outax") as i32,
            table.num(row, "outay") as i32,
            table.num(row, "outbx") as i32,
            table.num(row, "outby") as i32,
        );
        match table.text(row, "mode").as_str() {
            "int" => {
                golden_int.insert(key, corners);
            }
            "dot" => {
                golden_dot.insert(key, corners);
            }
            other => panic!("未知 mode {other}"),
        }
    }
    assert_eq!(golden_int.len(), 20);

    for (block, want_golden) in &golden_int {
        let (scale, want_corrected) = expected[block];
        let got = g0.restore_block(Idx::new(block.0, block.1), scale);
        assert_eq!(
            (got.0.x, got.0.y, got.1.x, got.1.y),
            want_corrected,
            "restore_block({block:?}, {scale}) 与独立复算不符"
        );
        // 结构性不变量：近端不越过远端，且都落在 5x7 内
        assert!(got.0.x <= got.1.x && got.0.y <= got.1.y, "{got:?}");
        assert!(got.0.x >= 0 && got.1.x <= 4 && got.0.y >= 0 && got.1.y <= 6);

        if got.0.x == want_golden.0 && got.0.y == want_golden.1 {
            same_as_golden += 1;
        } else {
            divergent += 1;
        }
        if *want_golden == golden_dot[block] {
            dot_equals_int += 1;
        }
    }
    assert_eq!(
        (same_as_golden, divergent),
        (8, 12),
        "新旧一致/分歧的行数变了，请复核 restore.csv 与 D4"
    );
    // 夹具事实：原 `SqDot` 重载混用 const/非 const `scale()`，13 组与 int 行相同、7 组不同
    assert_eq!(dot_equals_int, 13);
}

/// D4 的核心判据：区块映射必须**铺满整张图**，而不是全挤在左上角同一小块。
///
/// 修前 `ceil(block*scale)`：25×75 个区块只映射到 `near_x ∈ {0,1}`、`far_x = {1}`、
/// `near_y ∈ {0,1,2}`、`far_y ∈ {1,2}`，1875 个区块加起来只扫到 2002 格，`steep_extend`
/// 在这 2002 格上恒不判死（无 `inf`、起伏也没被看见）。
/// 修后每个区块覆盖 41×41 的原图窗口（相邻区块边界重叠一格）。
///
/// 同时钉住与 [`Grid::restore_dot`] 的方向一致性：区块中心的落格必须落在本区块两角之间。
#[test]
fn restore_block_spans_the_whole_grid() {
    let grid = Grid::new(vec![0.0; 1000 * 3000], 1000, 3000).unwrap();
    let scale = 1.0 / 40.0;

    assert_eq!(
        grid.restore_block(Idx::new(0, 0), scale),
        (Idx::new(0, 0), Idx::new(40, 40))
    );
    assert_eq!(
        grid.restore_block(Idx::new(24, 74), scale),
        (Idx::new(960, 2960), Idx::new(999, 2999))
    );

    let mut near_x = BTreeSet::new();
    let mut near_y = BTreeSet::new();
    let mut scanned = 0_usize;
    for i in 0..25 {
        for j in 0..75 {
            let (near, far) = grid.restore_block(Idx::new(i, j), scale);
            near_x.insert(near.x);
            near_y.insert(near.y);
            scanned += (far.x - near.x + 1) as usize * (far.y - near.y + 1) as usize;

            let center = grid.restore_dot(Idx::new(i, j), scale).as_idx();
            assert!(
                center.x >= near.x && center.x <= far.x && center.y >= near.y && center.y <= far.y,
                "区块 ({i},{j}) 的中心 {center:?} 不在角 {near:?}..{far:?} 之间"
            );
        }
    }
    assert_eq!(near_x.len(), 25, "区块行起点必须铺满整列");
    assert_eq!(near_y.len(), 75, "区块列起点必须铺满整行");
    assert_eq!(
        (*near_x.iter().min().unwrap(), *near_x.iter().max().unwrap()),
        (0, 960)
    );
    assert_eq!(
        (*near_y.iter().min().unwrap(), *near_y.iter().max().unwrap()),
        (0, 2960)
    );
    assert!(
        scanned > 3_000_000,
        "全图扫描格数应超过 3e6（修前只有 2002），实际 {scanned}"
    );
}

// ---------------------------------------------------------------- 语义：点 / 线 / 面

/// `SqDot::angle`（`geometry.cpp:340-342`）：`atan2(dy, dx)`，值域 `(-PI, PI]`；同点返回 0。
#[test]
fn dot_angle_domain_and_degenerate() {
    let origin = Dot::new(0.0, 0.0);
    assert_relative_eq!(origin.angle(Dot::new(1.0, 0.0)), 0.0, epsilon = 1e-12);
    assert_relative_eq!(origin.angle(Dot::new(0.0, 1.0)), FRAC_PI_2, epsilon = 1e-12);
    assert_relative_eq!(
        origin.angle(Dot::new(0.0, -1.0)),
        -FRAC_PI_2,
        epsilon = 1e-12
    );
    assert_relative_eq!(origin.angle(Dot::new(-1.0, 0.0)), PI, epsilon = 1e-12); // 取 +PI，不是 -PI
    assert_relative_eq!(origin.angle(origin), 0.0, epsilon = 1e-12); // atan2(0,0) == 0
    assert_relative_eq!(
        Dot::new(-1.0, -1.0).angle(origin),
        FRAC_PI_4,
        epsilon = 1e-12
    );
    for (x, y) in [
        (1.0, 1.0),
        (-1.0, 1.0),
        (-1.0, -1.0),
        (1.0, -1.0),
        (0.0, 0.0),
        (1e-300, -1e-300),
    ] {
        let angle = origin.angle(Dot::new(x, y));
        assert!(
            angle > -PI && angle <= PI,
            "angle({x},{y}) = {angle} 越出 (-PI, PI]"
        );
    }
}

/// `SqDot::distance`（`geometry.cpp:336-338`）。
#[test]
fn dot_distance_is_euclidean() {
    assert_relative_eq!(
        Dot::new(0.0, 0.0).distance(Dot::new(3.0, 4.0)),
        5.0,
        epsilon = 1e-12
    );
    assert_relative_eq!(
        Dot::new(-1.0, -1.0).distance(Dot::new(2.0, 3.0)),
        5.0,
        epsilon = 1e-12
    );
    assert_relative_eq!(
        Dot::new(7.0, 7.0).distance(Dot::new(7.0, 7.0)),
        0.0,
        epsilon = 1e-12
    );
}

/// `SqLine::distance`（`geometry.cpp:375-377`）：**到无限长直线**的距离，线段外的点不改判据。
#[test]
fn line_dist_is_infinite_line_not_segment() {
    let horizontal = Line {
        p0: Dot::new(0.0, 0.0),
        p1: Dot::new(1.0, 0.0),
    };
    assert_close(horizontal.dist_to_point(Dot::new(0.5, 0.0)), 0.0);
    assert_close(horizontal.dist_to_point(Dot::new(0.0, 5.0)), 5.0);
    // 落在线段之外，仍按所在直线算 —— 这是「无限长直线」语义的判据
    assert_close(horizontal.dist_to_point(Dot::new(3.0, 5.0)), 5.0);
    assert_close(horizontal.dist_to_point(Dot::new(-4.0, -5.0)), 5.0);

    // `SqLine(point, angle)`（`geometry.cpp:373-374`）与两点式是同一条直线
    let axis = Line::from_point_angle(Dot::new(0.0, 0.0), 0.0);
    assert_close(axis.p1.x(), 1.0);
    assert_close(axis.p1.y(), 0.0);
    assert_close(axis.dist_to_point(Dot::new(0.0, 3.0)), 3.0);

    let vertical = Line::from_point_angle(Dot::new(2.0, 1.0), FRAC_PI_2);
    assert_close(vertical.p1.x(), 2.0);
    assert_close(vertical.p1.y(), 2.0);
    assert_close(vertical.dist_to_point(Dot::new(7.0, -4.0)), 5.0);
}

/// `segments_intersect`（`geometry.cpp:1242-1259`）：跨立判据是 `<= 0`，所以
/// **端点相触算相交**、**共线重叠算相交**、共线不相接被包围盒拒掉。
#[test]
fn line_intersects_boundary_semantics() {
    let seg = |ax, ay, bx, by| Line {
        p0: Dot::new(ax, ay),
        p1: Dot::new(bx, by),
    };
    assert!(seg(0.0, 0.0, 2.0, 2.0).intersects(seg(0.0, 2.0, 2.0, 0.0))); // 真交叉
    assert!(seg(0.0, 0.0, 2.0, 0.0).intersects(seg(2.0, 0.0, 2.0, 2.0))); // 端点相触
    assert!(seg(0.0, 0.0, 1.0, 0.0).intersects(seg(1.0, 0.0, 2.0, 0.0))); // 共线端点相触
    assert!(seg(0.0, 0.0, 2.0, 0.0).intersects(seg(1.0, 0.0, 3.0, 0.0))); // 共线重叠
    assert!(!seg(0.0, 0.0, 1.0, 0.0).intersects(seg(2.0, 0.0, 3.0, 0.0))); // 共线不相接
    assert!(!seg(0.0, 0.0, 1.0, 0.0).intersects(seg(0.0, 1.0, 1.0, 1.0))); // 平行分离
    // 退化线段（点）：结果等价于「点是否落在另一条线段上」
    assert!(seg(0.5, 0.0, 0.5, 0.0).intersects(seg(0.0, 0.0, 1.0, 0.0)));
    assert!(!seg(0.5, 0.5, 0.5, 0.5).intersects(seg(0.0, 0.0, 1.0, 0.0)));
}

/// `CuPlane::normal_angle`（`geometry.cpp:1160-1167`）：`atan2(sqrt(a²+b²), |c|)`，模为 0 返回 0。
#[test]
fn plane_normal_angle_analytic_values() {
    let angle = |a, b, c| Plane { a, b, c, d: 0.0 }.normal_angle();
    assert_relative_eq!(angle(0.0, 0.0, 1.0), 0.0, epsilon = 1e-12);
    assert_relative_eq!(angle(0.0, 0.0, -3.0), 0.0, epsilon = 1e-12); // 取 |c|
    assert_relative_eq!(angle(1.0, 0.0, 0.0), FRAC_PI_2, epsilon = 1e-12); // 竖直面
    assert_relative_eq!(angle(3.0, 4.0, 0.0), FRAC_PI_2, epsilon = 1e-12);
    assert_relative_eq!(angle(1.0, 0.0, 1.0), FRAC_PI_4, epsilon = 1e-12); // 45°
    assert_relative_eq!(angle(0.0, 0.0, 0.0), 0.0, epsilon = 1e-12); // 退化面
}

/// `Plane::z_at` 的公式，以及 `c == 0` 时**保留除零行为**（调用方负责判断）。
#[test]
fn plane_z_at_keeps_division_by_zero() {
    let slanted = Plane {
        a: 1.0,
        b: 2.0,
        c: 4.0,
        d: -8.0,
    };
    assert_close(slanted.z_at(0.0, 0.0), 2.0);
    assert_close(slanted.z_at(2.0, 1.0), 1.0);

    let vertical = Plane {
        a: 1.0,
        b: 0.0,
        c: 0.0,
        d: -1.0,
    };
    let zero_over_zero = vertical.z_at(1.0, 0.0);
    assert!(
        zero_over_zero.is_nan(),
        "0/0 应得到 NaN，得到 {zero_over_zero}"
    );
    let divided_by_zero = Plane {
        a: 1.0,
        b: 0.0,
        c: 0.0,
        d: 0.0,
    }
    .z_at(2.0, 0.0);
    assert!(
        !divided_by_zero.is_finite(),
        "c == 0 必须产生非有限值，得到 {divided_by_zero}"
    );
}

// ---------------------------------------------------------------- 语义：平面拟合

/// `fit_plane_up` 的三个分支（`Ground::trip`，`ground.cpp:61-90`）+ `up_fit`（`geometry.cpp:1169-1199`）。
#[test]
fn fit_plane_up_three_branches() {
    let p = |x, y, z| Point3::new(x, y, z);

    // < 3：退化
    assert!(matches!(fit_plane_up(&[]), Err(CoreError::DegeneratePlane)));
    assert!(matches!(
        fit_plane_up(&[p(0.0, 0.0, 0.0)]),
        Err(CoreError::DegeneratePlane)
    ));
    assert!(matches!(
        fit_plane_up(&[p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)]),
        Err(CoreError::DegeneratePlane)
    ));

    // = 3：三点定面（z = 0 平面）
    let plane = fit_plane_up(&[p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)]).unwrap();
    assert_close(plane.a, 0.0);
    assert_close(plane.b, 0.0);
    assert_close(plane.c, 1.0);
    assert_close(plane.d, 0.0);
    for q in [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)] {
        assert_close(plane_value(plane, q), 0.0);
    }

    // = 3 共线：原实现在这里读未初始化成员（UB），本仓明确报退化
    assert!(matches!(
        fit_plane_up(&[p(1.0, 1.0, 1.0), p(2.0, 2.0, 2.0), p(3.0, 3.0, 3.0)]),
        Err(CoreError::DegeneratePlane)
    ));

    // > 3 且第 4 点在初始面上方：up_fit 把第 0 个点换成它
    // （数值 = up_fit.csv case 0 的 outA..outD，输入来自 golden_dump.cpp:199-203）
    let points = [
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(0.0, 1.0, 0.0),
        p(0.25, 0.25, 3.0),
    ];
    let fitted = fit_plane_up(&points).unwrap();
    assert_close(fitted.a, 3.0);
    assert_close(fitted.b, 3.0);
    assert_close(fitted.c, 0.5);
    assert_close(fitted.d, -3.0);
    for q in points {
        assert!(
            plane_value(fitted, q) <= 1e-9,
            "up-fit 后置条件被破坏：{q:?} 在平面上方"
        );
    }

    // > 3 但全部共面：返回该面
    let coplanar = fit_plane_up(&[
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(0.0, 1.0, 0.0),
        p(2.0, 3.0, 0.0),
    ])
    .unwrap();
    assert_close(coplanar.a, 0.0);
    assert_close(coplanar.b, 0.0);
    assert_close(coplanar.c, 1.0);
    assert_close(coplanar.d, 0.0);

    // > 3 且初始三点共线（x 轴）：C++ 在这里带着未初始化的面继续迭代（UB）；
    // 本仓把退化面当「无约束」、跳过退化候选，于是第 4 个点换掉第 0 个点后定出 z = 0。
    let recovered = fit_plane_up(&[
        p(0.0, 0.0, 0.0),
        p(1.0, 0.0, 0.0),
        p(2.0, 0.0, 0.0),
        p(0.0, 1.0, 0.0),
    ])
    .unwrap();
    assert_close(recovered.a, 0.0);
    assert_close(recovered.b, 0.0);
    assert_close(recovered.c, 1.0);
    assert_close(recovered.d, 0.0);

    // > 3 且全部共线：任何候选三点都定不了面 → 退化
    assert!(matches!(
        fit_plane_up(&[
            p(1.0, 1.0, 1.0),
            p(2.0, 2.0, 2.0),
            p(3.0, 3.0, 3.0),
            p(4.0, 4.0, 4.0)
        ]),
        Err(CoreError::DegeneratePlane)
    ));
}

// ---------------------------------------------------------------- golden：拟合

/// `normal_angle.csv`：三点平面（`= 3` 分支）+ `normal_angle()`。
///
/// case 3/4/5 是**三条共线点**：原 C++ 的 `CuPlane(const std::array<CuDot,3>&)` 在
/// `define_plane` 返回 false 时留下未初始化的 `A/B/C/D`（`geometry.cpp:1060-1062,1088-1090`），
/// 夹具里那三行恒为 `3,3,3,-5` 且不构成任何过这三点的平面 —— 是 UB 的产物，不可复现也不该复现，
/// 这里断言本仓返回 `Err(DegeneratePlane)`。
#[test]
fn golden_normal_angle_rows() {
    let table = Table::load("normal_angle.csv");
    let mut checked = 0_usize;
    let mut degenerate = 0_usize;
    for row in &table.rows {
        let p1 = Point3::new(
            table.num(row, "p1x"),
            table.num(row, "p1y"),
            table.num(row, "p1z"),
        );
        let p2 = Point3::new(
            table.num(row, "p2x"),
            table.num(row, "p2y"),
            table.num(row, "p2z"),
        );
        let p3 = Point3::new(
            table.num(row, "p3x"),
            table.num(row, "p3y"),
            table.num(row, "p3z"),
        );
        let case = table.text(row, "case");
        if collinear(p1, p2, p3) {
            assert!(
                matches!(fit_plane_up(&[p1, p2, p3]), Err(CoreError::DegeneratePlane)),
                "case {case}: 共线三点必须报退化"
            );
            degenerate += 1;
            continue;
        }
        let plane = fit_plane_up(&[p1, p2, p3]).unwrap();
        assert_close(plane.a, table.num(row, "A"));
        assert_close(plane.b, table.num(row, "B"));
        assert_close(plane.c, table.num(row, "C"));
        assert_close(plane.d, table.num(row, "D"));
        assert_close(plane.normal_angle(), table.num(row, "angle"));
        checked += 1;
    }
    assert_eq!(degenerate, 3, "夹具里的共线行数变了");
    assert_eq!(checked, 42);
}

/// `trip.csv`：45 组 `Ground::trip`（`ground.cpp:61-90`）的完整输入/输出。
/// 输入列 `x0,y0…x5,y5` 全部来自夹具，高度从 `grounds.csv` 取。
///
/// case 1/21/26 的**初始三点共线**：C++ 带着未初始化的面继续迭代（UB），本仓的确定化结果
/// 在 1/21 上恰好与夹具一致、在 26 上不同（见报告），因此这三行只断言「三分支里 >3 这一支
/// 不报错」，数值对齐交给其余 42 行。
///
/// 另外实测到一条原实现语义（不要当成 bug 去「修」）：`up_fit` 是贪心换点，**不保证**
/// 所有点最终都在平面下方 —— 先处理的点可能被后续换点抬到面上方（夹具 case 27 就是这样，
/// 而它的 A/B/C/D 与本仓逐值一致）。所以本测试不对 trip 结果加「无点在面上方」的后置条件。
#[test]
fn golden_trip_rows() {
    let table = Table::load("trip.csv");
    let grounds = load_grounds();
    let mut checked = 0_usize;
    let mut degenerate_initial = 0_usize;
    for row in &table.rows {
        let case = table.text(row, "case");
        let gid = table.num(row, "ground_id") as i32;
        let n = table.num(row, "area_n") as usize;
        let grid = &grounds[&gid];
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let x = table.num(row, &format!("x{i}")) as i32;
            let y = table.num(row, &format!("y{i}")) as i32;
            let height = grid
                .cell(Idx::new(x, y))
                .unwrap_or_else(|| panic!("case {case}: 采样点 ({x},{y}) 不在图内"));
            points.push(Point3::new(f64::from(x), f64::from(y), height));
        }

        let fitted = fit_plane_up(&points)
            .unwrap_or_else(|e| panic!("case {case}: trip 不应失败，得到 {e:?}"));

        if collinear(points[0], points[1], points[2]) {
            degenerate_initial += 1;
            continue;
        }
        assert_close(fitted.a, table.num(row, "A"));
        assert_close(fitted.b, table.num(row, "B"));
        assert_close(fitted.c, table.num(row, "C"));
        assert_close(fitted.d, table.num(row, "D"));
        assert_close(fitted.normal_angle(), table.num(row, "normal_angle"));
        checked += 1;
    }
    assert_eq!(degenerate_initial, 3, "夹具里的退化初始三点行数变了");
    assert_eq!(checked, 42);
}

/// `up_fit.csv`：单步 `up_fit`（`geometry.cpp:1169-1199`）的输入/输出。
///
/// 夹具的历史坑：`p1..p3` 曾经打印的是**调用后**的 `three`（与 `m0..m2` 重复），
/// 那样替换行就丢失了调用前输入。这里按 schema 自适应：
/// - 若存在 `m0..m2` 且**没有**任何一行 `p != m` → 旧夹具，只在 `q` 不等于任一 `m` 时喂
///   （即未发生替换，`p` 同时是输入与输出）；
/// - 否则（`p` 已是调用前值 / 根本没有 `m` 列）→ 全部行都喂 `p1..p3` + `q`。
#[test]
fn golden_up_fit_rows() {
    let table = Table::load("up_fit.csv");
    let has_post = table.has("m0x");
    let fixed_schema = has_post
        && table.rows.iter().any(|row| {
            (0..3).any(|i| {
                table.text(row, &format!("p{}x", i + 1)) != table.text(row, &format!("m{i}x"))
            })
        });

    let mut checked = 0_usize;
    let mut skipped = 0_usize;
    for row in &table.rows {
        let case = table.text(row, "case");
        let q = Point3::new(
            table.num(row, "qx"),
            table.num(row, "qy"),
            table.num(row, "qz"),
        );
        let three = [
            Point3::new(
                table.num(row, "p1x"),
                table.num(row, "p1y"),
                table.num(row, "p1z"),
            ),
            Point3::new(
                table.num(row, "p2x"),
                table.num(row, "p2y"),
                table.num(row, "p2z"),
            ),
            Point3::new(
                table.num(row, "p3x"),
                table.num(row, "p3y"),
                table.num(row, "p3z"),
            ),
        ];

        let usable = if !has_post || fixed_schema {
            true
        } else {
            // 旧夹具：q 出现在调用后的 three 里说明这一步做过替换，p 列不是输入
            !(0..3).any(|i| table.text(row, &format!("m{i}x")) == table.text(row, "qx"))
        };
        if !usable {
            skipped += 1;
            continue;
        }

        let plane = fit_plane_up(&[three[0], three[1], three[2], q])
            .unwrap_or_else(|e| panic!("case {case}: up_fit 不应失败，得到 {e:?}"));
        assert_close(plane.a, table.num(row, "outA"));
        assert_close(plane.b, table.num(row, "outB"));
        assert_close(plane.c, table.num(row, "outC"));
        assert_close(plane.d, table.num(row, "outD"));
        checked += 1;
    }

    assert!(
        checked >= 20,
        "只核对了 {checked} 行（跳过 {skipped}），夹具是不是坏了？"
    );
    assert_eq!(checked + skipped, table.rows.len());
    println!(
        "up_fit.csv: 新 schema = {fixed_schema}，核对 {checked} 行，跳过 {skipped} 行（旧 schema 下替换行的调用前输入不在夹具里）"
    );
}

// ---------------------------------------------------------------- 语义：栅格

/// `Grid::new` 的形状拒绝：`cells.len() != rows*cols` ⇒ `GridShapeMismatch`。
#[test]
fn grid_new_rejects_shape_mismatch() {
    let err = Grid::new(vec![0.0; 5], 2, 3).unwrap_err();
    match &err {
        CoreError::GridShapeMismatch { rows, cols, got } => {
            assert_eq!((*rows, *cols, *got), (2, 3, 5));
        }
        other => panic!("期望 GridShapeMismatch，得到 {other:?}"),
    }
    let message = err.to_string();
    assert!(message.contains("2x3"), "Display 应带形状: {message}");
    assert!(message.contains('5'), "Display 应带实际长度: {message}");

    assert!(Grid::new(vec![0.0; 6], 2, 3).is_ok());
    assert!(Grid::new(vec![0.0], 0, 0).is_err());
    // rows * cols 溢出按不匹配拒绝，不 panic
    assert!(Grid::new(vec![0.0; 4], usize::MAX, 2).is_err());

    // 0x0 空图可以构造（0 == 0），但所有访问器都空
    let empty = Grid::new(vec![], 0, 0).unwrap();
    assert_eq!((empty.rows(), empty.cols()), (0, 0));
    assert!(empty.cell(Idx::new(0, 0)).is_none());
    assert!(empty.neighbours4(Idx::new(0, 0)).is_empty());
}

/// 四邻域：**顺序固定**为 [`Dir4::ALL`]（左、右、下、上），只返回界内邻居。
/// 这条测试就是负向对照的目标（调换两个方向必须变红）。
#[test]
fn grid_neighbours4_order_and_bounds() {
    assert_eq!(
        Dir4::ALL.map(|d| d.delta()),
        [(-1, 0), (1, 0), (0, -1), (0, 1)]
    );

    let grid = Grid::new(vec![0.0; 9], 3, 3).unwrap();
    assert_eq!(
        grid.neighbours4(Idx::new(1, 1)),
        vec![
            Idx::new(0, 1), // 左
            Idx::new(2, 1), // 右
            Idx::new(1, 0), // 下
            Idx::new(1, 2), // 上
        ]
    );
    assert_eq!(
        grid.neighbours4(Idx::new(0, 0)),
        vec![Idx::new(1, 0), Idx::new(0, 1)]
    );
    assert_eq!(
        grid.neighbours4(Idx::new(2, 2)),
        vec![Idx::new(1, 2), Idx::new(2, 1)]
    );
    assert_eq!(grid.neighbours4(Idx::new(5, 5)), Vec::<Idx>::new());
    assert_eq!(grid.neighbours4(Idx::new(-1, -1)), Vec::<Idx>::new());

    let single = Grid::new(vec![0.0], 1, 1).unwrap();
    assert_eq!(single.neighbours4(Idx::new(0, 0)), Vec::<Idx>::new());
}

/// `OBSTACLE = +inf` 的语义（`SqPlane` 的惯例），以及它与 `Ground` 的 `< 0` 惯例的区别。
#[test]
fn grid_obstacle_semantics() {
    // 2x3 行优先：行 0 = {1, 2, 3}，行 1 = {4, inf, -1}
    let grid = Grid::new(vec![1.0, 2.0, 3.0, 4.0, OBSTACLE, -1.0], 2, 3).unwrap();
    assert!(grid.is_obstacle(Idx::new(1, 1)));
    assert!(!grid.is_obstacle(Idx::new(0, 0)));
    // -1.0 是 `Ground::obstacle`（`ground.cpp:282-287`，`map[x][y] < 0`）的障碍惯例，不是 Grid 的
    assert!(!grid.is_obstacle(Idx::new(1, 2)));
    // 越界不是障碍
    assert!(!grid.is_obstacle(Idx::new(5, 5)));
    assert!(!grid.is_obstacle(Idx::new(-1, 0)));
    // NaN 不等于任何值，因此不是障碍
    let nan_grid = Grid::new(vec![f64::NAN], 1, 1).unwrap();
    assert!(!nan_grid.is_obstacle(Idx::new(0, 0)));
    assert!(nan_grid.cell(Idx::new(0, 0)).unwrap().is_nan());
}

/// `SqPlane::cost`（`geometry.cpp:889-895`）：曼哈顿距离 **+ 目标格的高度值**；
/// OBSTACLE 邻格因此是 `+inf`，越界确定化为 `+inf`。
#[test]
fn grid_cost_is_manhattan_plus_target_height() {
    let grid = Grid::new(vec![1.0, 2.0, 3.0, 4.0, OBSTACLE, -1.0], 2, 3).unwrap();
    assert_close(grid.cost(Idx::new(0, 0), Idx::new(0, 1)), 1.0 + 2.0);
    assert_close(grid.cost(Idx::new(0, 0), Idx::new(0, 0)), 1.0);
    assert_close(grid.cost(Idx::new(1, 2), Idx::new(0, 1)), 1.0 + 1.0 + 2.0);
    assert_close(grid.cost(Idx::new(0, 1), Idx::new(1, 2)), 1.0 + 1.0 - 1.0);
    assert!(grid.cost(Idx::new(0, 0), Idx::new(1, 1)).is_infinite());
    assert!(grid.cost(Idx::new(0, 0), Idx::new(9, 9)).is_infinite());
}

/// `SqPlane::scale_graph` / `summary`（`geometry.cpp:657-688,794-813`）：窗口算术平均，
/// 含 OBSTACLE 的窗口因 `inf` 参与求和而整格记障碍；非法 scale 与空图被拒。
#[test]
fn grid_scale_down_pooling_and_rejections() {
    // scale = 1.0：窗口 = max(1, trunc(1/1)) = 1，即 3x3，2x2 图里覆盖全图 → 全格 = 全图均值
    let grid = Grid::new(vec![1.0, 3.0, 5.0, 7.0], 2, 2).unwrap();
    let pooled = grid.scale_down(1.0).unwrap();
    assert_eq!((pooled.rows(), pooled.cols()), (2, 2));
    for x in 0..2 {
        for y in 0..2 {
            assert_close(pooled.cell(Idx::new(x, y)).unwrap(), 4.0);
        }
    }

    // 含 OBSTACLE 的窗口 → +inf（不是「对有限值取均值」）
    let with_obstacle = Grid::new(vec![1.0, 3.0, OBSTACLE, 7.0], 2, 2).unwrap();
    let pooled = with_obstacle.scale_down(1.0).unwrap();
    assert!(pooled.cell(Idx::new(0, 0)).unwrap().is_infinite());

    // 缩小：4x4 scale = 0.5 → 2x2，窗口半边长 = trunc(1/0.5) = 2
    let ramp: Vec<f64> = (0..16).map(f64::from).collect();
    let small = Grid::new(ramp, 4, 4).unwrap().scale_down(0.5).unwrap();
    assert_eq!((small.rows(), small.cols()), (2, 2));
    assert_close(small.cell(Idx::new(0, 0)).unwrap(), 7.5); // 全图均值
    assert_close(small.cell(Idx::new(0, 1)).unwrap(), 8.0); // 列 1..3 的均值

    // 非法 scale / 空图
    let grid = Grid::new(vec![0.0; 4], 2, 2).unwrap();
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            matches!(grid.scale_down(bad), Err(CoreError::InvalidScale(_))),
            "scale = {bad} 应被拒"
        );
    }
    let empty = Grid::new(vec![], 0, 0).unwrap();
    assert!(matches!(
        empty.scale_down(1.0),
        Err(CoreError::DegeneratePlane)
    ));
    // scale > 1 时窗口仍是 3x3（trunc(1/scale) = 0 → 抬到 1）
    let upscaled = Grid::new(vec![1.0, 2.0, 3.0, 4.0], 2, 2)
        .unwrap()
        .scale_down(2.0)
        .unwrap();
    assert_eq!((upscaled.rows(), upscaled.cols()), (4, 4));
    assert_close(upscaled.cell(Idx::new(0, 0)).unwrap(), 2.5);
}

/// `scale_graph.csv` + `scale_graph_cells.csv`：35 组（7 张图 × 5 个 scale）的
/// 输出尺寸与全部 5217 个格子。
#[test]
fn golden_scale_graph_rows() {
    let shapes = Table::load("scale_graph.csv");
    let cells = Table::load("scale_graph_cells.csv");
    let mut expected: BTreeMap<(String, usize, usize), f64> = BTreeMap::new();
    for row in &cells.rows {
        expected.insert(
            (
                cells.text(row, "case"),
                cells.num(row, "x") as usize,
                cells.num(row, "y") as usize,
            ),
            cells.num(row, "value"),
        );
    }

    let grounds = load_grounds();
    let mut checked = 0_usize;
    for row in &shapes.rows {
        let case = shapes.text(row, "case");
        let gid = shapes.num(row, "ground_id") as i32;
        let scale = shapes.num(row, "scale");
        let out = grounds[&gid]
            .scale_down(scale)
            .unwrap_or_else(|e| panic!("case {case}: scale_down({scale}) 失败: {e:?}"));
        assert_eq!(
            out.rows(),
            shapes.num(row, "out_rows") as usize,
            "case {case}"
        );
        assert_eq!(
            out.cols(),
            shapes.num(row, "out_cols") as usize,
            "case {case}"
        );
        for x in 0..out.rows() {
            for y in 0..out.cols() {
                let want = expected[&(case.clone(), x, y)];
                assert_close(out.cell(Idx::new(x as i32, y as i32)).unwrap(), want);
                checked += 1;
            }
        }
    }
    assert_eq!(checked, 5217);
}

/// 测量型判据：`scale_star`（高层 A*）在**真实地图**上的 `expanded` 与引导点个数。
///
/// D4 修正 `restore_block` 方向后，高层搜索的代价项从「恒 0 的 `steep_extend`」变成真实地形代价，
/// `expanded` 与引导路径都会变。CLI 的 metrics JSON 不含 `expanded`，而 `astar.rs` 不在本任务写域内，
/// 所以这里用公开 API 单独探一次（固定 seed，可复现）。
#[test]
fn probe_scale_star_expanded_on_real_map() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../data/csv/map.csv");
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("读不到 {}: {e}", path.display()));
    let mut cells = Vec::new();
    let (mut rows, mut cols) = (0_usize, 0_usize);
    for line in text.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let mut width = 0_usize;
        for token in line.split(',') {
            cells.push(
                token
                    .trim()
                    .parse::<f64>()
                    .expect("map.csv 必须是纯数值（无表头）"),
            );
            width += 1;
        }
        rows += 1;
        cols = width;
    }
    assert_eq!((rows, cols), (1000, 3000), "map.csv 形状变了");
    let terrain = Terrain::from_grid(Grid::new(cells, rows, cols).unwrap());
    let scale = 1.0 / 40.0;

    let start = Idx::new(0, 0);
    let goal = Idx::new(500, 500);
    let mut rng = Rng::from_seed(42);
    let search = scale_star(&terrain, start, goal, scale, &mut rng)
        .expect("(0,0)->(500,500) 应可达（同一粗图条带内）");
    assert_eq!(search.path.first(), Some(&start));
    assert_eq!(search.path.last(), Some(&goal));
    assert!(search.expanded > 0 && search.expanded <= 25 * 75);
    println!(
        "scale_star {start:?}->{goal:?} seed42: expanded={} 引导点数={}",
        search.expanded,
        search.path.len()
    );

    // 方向修正后，粗图的判死块把地图切成条带；(500,500)->(900,1200) 分处条带两侧，
    // 高层 A* 因此返回 None —— 这是真实地形约束的结果，如实打印，不假装可达。
    let mut rng = Rng::from_seed(42);
    match scale_star(
        &terrain,
        Idx::new(500, 500),
        Idx::new(900, 1200),
        scale,
        &mut rng,
    ) {
        Some(found) => println!(
            "scale_star (500,500)->(900,1200) seed42: expanded={} 引导点数={}",
            found.expanded,
            found.path.len()
        ),
        None => println!("scale_star (500,500)->(900,1200) seed42: None（粗图不可达）"),
    }

    // 确定性：同 seed 两次逐点相同（可复现性不能因为换方向而丢）
    let mut a = Rng::from_seed(42);
    let mut b = Rng::from_seed(42);
    let first = scale_star(&terrain, start, goal, scale, &mut a).unwrap();
    let second = scale_star(&terrain, start, goal, scale, &mut b).unwrap();
    assert_eq!(first.path, second.path);
    assert_eq!(first.expanded, second.expanded);
}
