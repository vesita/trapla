//! `terrain.rs` / `astar.rs` / `rng.rs` 的语义钉死 + C++ 函数级 golden 对照。
//!
//! 三部分：
//! 1. **源码语义测试**：每条都注明它钉住的 C++ 出处（`文件:行`）；
//! 2. **golden 对照**：喂 `tests/golden/*.csv` 的输入列，比输出列，容差 1e-9；
//! 3. 边界 / 拒绝路径（退化平面、越界、无路、非法 scale）。
//!
//! 夹具一律**按表头名**取值。`steep_extend.csv` 的对照有一条**有意修正**：原
//! `src/aStar/aStar.cpp:212` 的 `std::min(bounded_first.x, bounded_first.x)` 笔误，
//! 本实现取 `min(bounded_first.x, bounded_second.x)`；只有 `clamp(x0) > clamp(x1)` 的行
//! 可能与该笔误不同，这些行单独列出、不参与 1e-9 相等断言（见 `steep_extend_golden`）。

use std::collections::BTreeMap;
use std::path::PathBuf;

use trapla_core::astar::{
    a_star_search, direction_determine, geometric_decay, geometric_decay_start, scale_star,
};
use trapla_core::error::CoreError;
use trapla_core::geom::{Dot, Idx, Plane, Point3, fit_plane_up};
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

    fn int(&self, row: &[String], column: &str) -> i32 {
        let raw = self.text(row, column);
        raw.parse::<i32>()
            .unwrap_or_else(|_| panic!("列 {column} 期望整数，实得 {raw:?}"))
    }
}

fn assert_close(got: f64, want: f64, context: &str) {
    let diff = (got - want).abs();
    let tolerance = 1e-9_f64.max(1e-9 * want.abs());
    assert!(
        diff <= tolerance,
        "{context}: want {want}, got {got} (diff {diff}, tol {tolerance})"
    );
}

/// 读 `grounds.csv`，按 `ground_id` 组装成稠密 `Grid`。
///
/// 每张图的行列上界由 `x`/`y` 的最大值 +1 决定（`grounds.csv` 是全量导出，没有缺格）；
/// 高度列直接是栅格值（`inf` 是 [`OBSTACLE`]）。
fn load_grounds() -> BTreeMap<i32, Grid> {
    let table = Table::load("grounds.csv");

    let mut max_index: BTreeMap<i32, (i32, i32)> = BTreeMap::new();
    for row in &table.rows {
        let gid = table.int(row, "ground_id");
        let entry = max_index.entry(gid).or_insert((-1, -1));
        entry.0 = entry.0.max(table.int(row, "x"));
        entry.1 = entry.1.max(table.int(row, "y"));
    }

    let mut cells: BTreeMap<i32, Vec<f64>> = BTreeMap::new();
    for (gid, (max_x, max_y)) in &max_index {
        let len = ((max_x + 1) as usize) * ((max_y + 1) as usize);
        cells.insert(*gid, vec![0.0; len]);
    }
    for row in &table.rows {
        let gid = table.int(row, "ground_id");
        let x = table.int(row, "x") as usize;
        let y = table.int(row, "y") as usize;
        let cols = (max_index[&gid].1 + 1) as usize;
        cells.get_mut(&gid).expect("ground_id 已登记")[x * cols + y] = table.num(row, "height");
    }

    cells
        .into_iter()
        .map(|(gid, data)| {
            let (max_x, max_y) = max_index[&gid];
            let grid = Grid::new(data, (max_x + 1) as usize, (max_y + 1) as usize)
                .expect("grounds.csv 是稠密矩形");
            (gid, grid)
        })
        .collect()
}

fn terrains() -> BTreeMap<i32, Terrain> {
    load_grounds()
        .into_iter()
        .map(|(gid, grid)| (gid, Terrain::from_grid(grid)))
        .collect()
}

/// 原 `SqPlane::orth_near`（`src/utils/geometry.cpp:549-553`）的 `x` 分量，
/// 用于在测试里判「这一行是否落在 aStar.cpp:212 笔误的差异区」。
fn clamped_x(grid: &Grid, raw: i32) -> i32 {
    raw.clamp(0, (grid.rows() as i32 - 1).max(0))
}

// ------------------------------------------------------- rng.rs：确定性守卫

/// `Rng::from_seed(42)` 的前 8 个 `next_u64()` —— **跨平台/跨版本的冻结守卫**
/// （task-16 决策记录的核心判据）。改算法必先改这里，并且要是一次有意的、跨版本的决定。
const FROZEN_SEED_42: [u64; 8] = [
    3_580_622_183_945_639_842,
    10_378_725_325_292_465_923,
    8_967_075_514_996_744_559,
    5_001_014_893_397_904_463,
    14_825_054_885_549_601_002,
    10_736_401_887_688_096_443,
    5_571_599_124_965_916_891,
    14_671_890_910_021_047_719,
];

/// `Rng::from_seed(42)` 洗 `0..10` 得到的排列 —— 冻结 Fisher–Yates 的消费顺序。
const FROZEN_SHUFFLE_42: [u32; 10] = [0, 1, 6, 8, 3, 9, 5, 7, 4, 2];

#[test]
fn rng_frozen_seed_42_sequence() {
    let mut rng = Rng::from_seed(42);
    let got: Vec<u64> = (0..8).map(|_| rng.next_u64()).collect();
    assert_eq!(
        got, FROZEN_SEED_42,
        "seed=42 的冻结序列变了（跨版本不稳定）"
    );
    // 非退化守卫：不许全 0、不许有重复（xorshift64* 退化的典型症状）。
    assert!(
        got.iter().all(|&v| v != 0),
        "冻结序列里出现 0，说明状态退化"
    );
    let mut sorted = got.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(sorted.len(), got.len(), "冻结序列里出现重复值");
}

#[test]
fn rng_frozen_seed_42_shuffle() {
    let mut rng = Rng::from_seed(42);
    let mut array: Vec<u32> = (0..10).collect();
    rng.shuffle(&mut array);
    assert_eq!(
        array, FROZEN_SHUFFLE_42,
        "seed=42 的冻结洗牌结果变了（Fisher–Yates 消费顺序不稳定）"
    );

    // 同一 seed 两个实例：逐项相同（可复现性的最小判据）。
    let mut second = Rng::from_seed(42);
    let mut array2: Vec<u32> = (0..10).collect();
    second.shuffle(&mut array2);
    assert_eq!(array, array2, "同一 seed 两次洗牌结果必须逐项相同");
}

#[test]
fn rng_seed_1_and_2_shuffle_differ() {
    let mut first = Rng::from_seed(1);
    let mut array1: Vec<u32> = (0..10).collect();
    first.shuffle(&mut array1);

    let mut second = Rng::from_seed(2);
    let mut array2: Vec<u32> = (0..10).collect();
    second.shuffle(&mut array2);

    assert_ne!(array1, array2, "不同 seed 的洗牌结果不该相同");
}

#[test]
fn rng_seed_0_is_usable() {
    // SplitMix64 对 seed=0 良定义；状态 0 会让 xorshift64* 退化，必须有保护。
    let mut rng = Rng::from_seed(0);
    let values: Vec<u64> = (0..8).map(|_| rng.next_u64()).collect();
    assert!(values.iter().all(|&v| v != 0), "seed=0 退化成了全 0 序列");
}

#[test]
fn rng_next_below_is_in_range() {
    for k in 1_u64..=64 {
        let mut rng = Rng::from_seed(k);
        for _ in 0..256 {
            let value = rng.next_below(k);
            assert!(value < k, "next_below({k}) 给出 {value}");
        }
    }
    // k == 1 时恒为 0（阈值 0，不丢样本）。
    let mut rng = Rng::from_seed(7);
    for _ in 0..64 {
        assert_eq!(rng.next_below(1), 0);
    }
}

#[test]
fn rng_shuffle_degenerate_lengths() {
    let mut rng = Rng::from_seed(3);
    let mut empty: [u32; 0] = [];
    rng.shuffle(&mut empty); // 不许 panic

    let mut single = [99_u32];
    rng.shuffle(&mut single);
    assert_eq!(single, [99]);
}

// ---------------------------------------------------------- astar.rs：A*

#[test]
fn a_star_start_equals_goal_returns_single_point() {
    // 原 `aStar_test.cpp:167`：起点终点相同时路径只有一个点。
    let grid = Grid::new(vec![0.0; 9], 3, 3).expect("3x3");
    let search = a_star_search(&grid, Idx::new(1, 1), Idx::new(1, 1)).expect("同点必有路径");
    assert_eq!(search.path, vec![Idx::new(1, 1)]);
    assert_eq!(search.expanded, 1);
}

#[test]
fn a_star_detours_around_obstacle() {
    // 5x5 全 0，在 x=2 的中间竖墙上从 y=0 堵到 y=3，只留 y=4 的通道。
    let rows = 5;
    let cols = 5;
    let mut cells = vec![0.0; rows * cols];
    for y in 0..4 {
        cells[2 * cols + y] = OBSTACLE;
    }
    let grid = Grid::new(cells, rows, cols).expect("5x5");
    let start = Idx::new(0, 0);
    let goal = Idx::new(4, 0);
    let search = a_star_search(&grid, start, goal).expect("绕行路径存在");

    assert_eq!(search.path.first().copied(), Some(start));
    assert_eq!(search.path.last().copied(), Some(goal));
    assert!(
        search.path.len() > 5,
        "直连长度 5 不可达，必须绕行；实得 {} 点",
        search.path.len()
    );
    for window in search.path.windows(2) {
        let (a, b) = (window[0], window[1]);
        assert!(!grid.is_obstacle(b), "路径踩到障碍 {b:?}");
        let step = (a.x - b.x).abs() + (a.y - b.y).abs();
        assert_eq!(step, 1, "路径必须四邻接：{a:?} -> {b:?}");
    }
    // 必须真的用到 y=4 的通道。
    assert!(
        search.path.iter().any(|i| i.y == 4),
        "绕行路径应经过 y=4 通道"
    );
}

#[test]
fn a_star_unreachable_returns_none() {
    // 整列 x=2 都是障碍，左右完全隔断。
    let rows = 5;
    let cols = 5;
    let mut cells = vec![0.0; rows * cols];
    for y in 0..cols {
        cells[2 * cols + y] = OBSTACLE;
    }
    let grid = Grid::new(cells, rows, cols).expect("5x5");
    assert!(
        a_star_search(&grid, Idx::new(0, 0), Idx::new(4, 0)).is_none(),
        "被完全隔断时必须返回 None"
    );
    // 起终点本身非法 / 是障碍也返回 None。
    assert!(a_star_search(&grid, Idx::new(-1, 0), Idx::new(0, 0)).is_none());
    assert!(a_star_search(&grid, Idx::new(0, 0), Idx::new(2, 2)).is_none());
}

#[test]
fn scale_star_same_seed_is_pointwise_identical() {
    // 10x10 全 0 + 一个障碍；scale=0.5 ⇒ 缩放网格 5x5。
    let rows = 10;
    let cols = 10;
    let mut cells = vec![0.0; rows * cols];
    cells[5 * cols + 5] = OBSTACLE;
    let terrain = Terrain::from_grid(Grid::new(cells, rows, cols).expect("10x10"));

    let start = Idx::new(0, 0);
    let goal = Idx::new(8, 8);

    let mut rng_a = Rng::from_seed(12345);
    let first = scale_star(&terrain, start, goal, 0.5, &mut rng_a).expect("缩放图上有路");
    let mut rng_b = Rng::from_seed(12345);
    let second = scale_star(&terrain, start, goal, 0.5, &mut rng_b).expect("缩放图上有路");

    assert_eq!(
        first.path, second.path,
        "同 seed 两次 scale_star 必须逐点相同"
    );
    assert_eq!(first.expanded, second.expanded);
    assert_eq!(first.path.first().copied(), Some(start));
    assert_eq!(first.path.last().copied(), Some(goal));
}

#[test]
fn scale_star_rejects_invalid_scale_and_empty_grid() {
    let terrain = Terrain::from_grid(Grid::new(vec![0.0; 4], 2, 2).expect("2x2"));
    for scale in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut rng = Rng::from_seed(1);
        assert!(
            scale_star(&terrain, Idx::new(0, 0), Idx::new(1, 1), scale, &mut rng).is_none(),
            "非法 scale {scale} 应返回 None"
        );
    }
    let empty = Terrain::from_grid(Grid::new(Vec::new(), 0, 0).expect("0x0"));
    let mut rng = Rng::from_seed(1);
    assert!(scale_star(&empty, Idx::new(0, 0), Idx::new(0, 0), 0.5, &mut rng).is_none());
}

// -------------------------------------------------- astar.rs：方向权重

#[test]
fn geometric_decay_golden() {
    let table = Table::load("geometric_decay.csv");
    let mut checked = 0usize;
    for row in &table.rows {
        let case = table.int(row, "case");
        let function = table.text(row, "fn");
        let alpha = table.num(row, "alpha");
        let start = table.num(row, "start");
        let n = table.int(row, "n") as usize;
        let coefficient = match function.as_str() {
            "geometric_decay" => geometric_decay(alpha, n),
            "geometric_decay_start" => geometric_decay_start(start, alpha, n),
            other => panic!("case {case}: 未知 fn {other:?}"),
        };
        assert_eq!(coefficient.len(), n, "case {case}: 系数个数");
        // 夹具只导出前 10 个系数（`n = 27` 的行按前 10 项对照，其余项无法逐条核）。
        for (i, &value) in coefficient.iter().enumerate().take(10) {
            let column = format!("c{i}");
            assert!(table.has(&column), "夹具缺列 {column}");
            let raw = table.text(row, &column);
            assert!(
                !raw.is_empty(),
                "case {case}: 系数有 {} 个但 {column} 为空",
                n
            );
            assert_close(
                value,
                parse_f64(&raw),
                &format!("geometric_decay.csv case {case} c{i}"),
            );
        }
        // 少于 10 项时，夹具的第 n..10 列必须留空（防夹具与实现一起漂）。
        for i in coefficient.len().min(10)..10 {
            let column = format!("c{i}");
            assert!(table.has(&column), "夹具缺列 {column}");
            assert!(
                table.text(row, &column).is_empty(),
                "case {case}: {column} 应为空"
            );
        }
        checked += 1;
    }
    assert_eq!(checked, 76, "geometric_decay.csv 应有 76 行被核对");
}

#[test]
fn geometric_decay_coefficients_sum_to_one_and_are_monotone() {
    // 原 `tests/direction_test.cpp:137-151` 的三条：和≈1、个数、单调不增。
    // 单调性是**反向排列后**的性质：只有 alpha >= 1 时反转后才是递减的
    // （alpha < 1 时最大项在末尾；原测试用的 sqrt(PI) > 1）。
    for &alpha in &[std::f64::consts::PI.sqrt(), 1.0, 3.0, 0.5, 2.0, 0.200_1] {
        for &n in &[1_usize, 2, 3, 5, 8, 27] {
            let coefficient = geometric_decay(alpha, n);
            assert_eq!(coefficient.len(), n, "alpha={alpha} n={n} 个数");
            let sum: f64 = coefficient.iter().sum();
            assert!(
                (sum - 1.0).abs() <= 1e-9,
                "alpha={alpha} n={n}: 系数和 {sum} 偏离 1"
            );
            if alpha >= 1.0 {
                for i in 1..coefficient.len() {
                    assert!(
                        coefficient[i] <= coefficient[i - 1] + 1e-15,
                        "alpha={alpha} n={n}: 第 {i} 项大于前一项"
                    );
                }
            }
        }
    }
}

#[test]
fn direction_determine_golden() {
    let table = Table::load("direction_determine.csv");
    let guides_table = Table::load("direction_guides.csv");

    let mut guides: BTreeMap<i32, Vec<(i32, Dot)>> = BTreeMap::new();
    for row in &guides_table.rows {
        let case = table.int(row, "case");
        let index = guides_table.int(row, "idx");
        // `int` 模式下引导点是 dot 取整后的整数，`as_dot` 后数值不变 ⇒ 两种模式都读 x/y。
        let point = Dot::new(guides_table.num(row, "x"), guides_table.num(row, "y"));
        guides.entry(case).or_default().push((index, point));
    }
    for list in guides.values_mut() {
        list.sort_by_key(|(index, _)| *index);
    }

    let mut checked = 0usize;
    for row in &table.rows {
        let case = table.int(row, "case");
        let near_end = table.int(row, "near_end") != 0;
        let n = table.int(row, "n") as usize;
        let list: Vec<Dot> = guides
            .get(&case)
            .map(|v| v.iter().map(|(_, point)| *point).collect())
            .unwrap_or_default();
        assert_eq!(list.len(), n, "case {case}: 引导点数与 n 不符");

        let got = direction_determine(&list, near_end);
        assert_close(
            got.x(),
            table.num(row, "outx"),
            &format!("direction_determine.csv case {case} outx"),
        );
        assert_close(
            got.y(),
            table.num(row, "outy"),
            &format!("direction_determine.csv case {case} outy"),
        );
        checked += 1;
    }
    assert_eq!(checked, 40, "direction_determine.csv 应有 40 行被核对");
}

// ------------------------------------------------------------ terrain.rs

#[test]
fn steep_extend_dead_branches_and_single_point() {
    // 分支 1：包围盒越界（空图钳位后 0 >= rows）。
    let empty = Terrain::from_grid(Grid::new(Vec::new(), 0, 0).expect("0x0"));
    assert!(
        empty
            .steep_extend((Idx::new(0, 0), Idx::new(0, 0)))
            .is_none()
    );

    // 分支 2：障碍占比 >= 0.3（2/4 = 0.5）。
    let half_obstacle =
        Terrain::from_grid(Grid::new(vec![OBSTACLE, OBSTACLE, 0.0, 0.0], 2, 2).expect("2x2"));
    assert!(
        half_obstacle
            .steep_extend((Idx::new(0, 0), Idx::new(1, 1)))
            .is_none()
    );

    // 分支 3：heights 为空（全负值 —— 原实现把 < 0 当障碍语义，不收进 heights）。
    let all_negative = Terrain::from_grid(Grid::new(vec![-1.0; 4], 2, 2).expect("2x2"));
    assert!(
        all_negative
            .steep_extend((Idx::new(0, 0), Idx::new(1, 1)))
            .is_none()
    );

    // 分支 4：综合评分 > 10（stddev=15、diff=30、slope=30 ⇒ 24）。
    let too_steep = Terrain::from_grid(Grid::new(vec![0.0, 30.0], 1, 2).expect("1x2"));
    assert!(
        too_steep
            .steep_extend((Idx::new(0, 0), Idx::new(0, 1)))
            .is_none()
    );

    // 单点 ⇒ 0.0（`aStar.cpp:251`）。
    let single = Terrain::from_grid(Grid::new(vec![7.0], 1, 1).expect("1x1"));
    assert_eq!(
        single.steep_extend((Idx::new(0, 0), Idx::new(0, 0))),
        Some(0.0)
    );

    // 可行解的解析值：heights=[0,2] ⇒ stddev=1、diff=2、slope=2 ⇒ 0.4+0.6+0.6 = 1.6。
    let gentle = Terrain::from_grid(Grid::new(vec![0.0, 2.0], 1, 2).expect("1x2"));
    assert_close(
        gentle
            .steep_extend((Idx::new(0, 0), Idx::new(0, 1)))
            .expect("可行"),
        1.6,
        "steep_extend 解析值",
    );
}

#[test]
fn steep_extend_golden() {
    let terrains = terrains();
    let table = Table::load("steep_extend.csv");

    // `clamp(x0) > clamp(x1)` 的行落在 aStar.cpp:212 笔误的差异区（潜在），清单钉死。
    let expected_typo_region_cases: Vec<i32> =
        vec![11, 14, 16, 22, 26, 27, 28, 29, 35, 36, 37, 39, 40, 42];
    // 差异区里**映射到 Option 后仍未变**（0 / -1 与 Some(0.0) / None 等价）的行。
    let expected_coincident_cases: Vec<i32> = vec![14, 16, 28, 40];
    // 差异区里**结果确实变了**的行 —— 这是 aStar.cpp:212 修正的真实影响面。
    // （= 潜在差异区 − 等值行；下面的断言会校验这个恒等式。）
    let expected_material_deviation_cases: Vec<i32> = vec![11, 22, 26, 27, 29, 35, 36, 37, 39, 42];

    let mut checked = 0usize;
    let mut typo_region_cases: Vec<i32> = Vec::new();
    let mut material_deviation_cases: Vec<i32> = Vec::new();
    let mut coincidence_cases: Vec<i32> = Vec::new();
    let mut deviation_report: Vec<String> = Vec::new();

    for row in &table.rows {
        let case = table.int(row, "case");
        let gid = table.int(row, "ground_id");
        let terrain = terrains
            .get(&gid)
            .unwrap_or_else(|| panic!("缺 ground {gid}"));
        let block = (
            Idx::new(table.int(row, "x0"), table.int(row, "y0")),
            Idx::new(table.int(row, "x1"), table.int(row, "y1")),
        );
        let want = table.num(row, "result");
        let got = terrain.steep_extend(block);
        let in_typo_region =
            clamped_x(&terrain.grid, block.0.x) > clamped_x(&terrain.grid, block.1.x);
        if in_typo_region {
            typo_region_cases.push(case);
        }

        // 与 C++ 值比较（`-1` 是「判死」哨兵 ⇒ 映射成 None）。
        let mapped_want = if want < 0.0 { None } else { Some(want) };
        let matches = match (got, mapped_want) {
            (None, None) => true,
            (Some(actual), Some(expected)) => {
                (actual - expected).abs() <= 1e-9_f64.max(1e-9 * expected.abs())
            }
            _ => false,
        };

        if matches {
            checked += 1;
            if in_typo_region {
                coincidence_cases.push(case);
            }
        } else {
            assert!(
                in_typo_region,
                "case {case}: 非差异区不允许与 golden 不一致"
            );
            material_deviation_cases.push(case);
            deviation_report.push(format!(
                "case {case} (ground {gid}, ({}, {})->({}, {})): C++ 笔误 {} → Rust 修正 {got:?}",
                block.0.x, block.0.y, block.1.x, block.1.y, want
            ));
        }
    }

    assert_eq!(
        typo_region_cases, expected_typo_region_cases,
        "aStar.cpp:212 潜在差异区清单变了"
    );
    assert_eq!(
        coincidence_cases, expected_coincident_cases,
        "差异区里「映射后仍等值」的清单变了"
    );
    assert_eq!(
        material_deviation_cases, expected_material_deviation_cases,
        "aStar.cpp:212 修正的真实影响面变了"
    );
    assert_eq!(
        checked + material_deviation_cases.len(),
        46,
        "steep_extend.csv 行数"
    );
    assert_eq!(
        coincidence_cases.len() + material_deviation_cases.len(),
        typo_region_cases.len()
    );

    for line in &deviation_report {
        println!("{line}");
    }
    println!(
        "steep_extend 对照：1e-9 相等 {checked}/46 行；笔误潜在差异区 {} 行，其中真实改变 {} 行（另 {} 行映射到 Option 后等值）",
        typo_region_cases.len(),
        material_deviation_cases.len(),
        coincidence_cases.len()
    );
}

#[test]
fn plane_at_truncates_index_but_keeps_fractional_coordinates() {
    // 2x2：cells[x*2+y]，高度 1,2 / 3,4。
    let terrain = Terrain::from_grid(Grid::new(vec![1.0, 2.0, 3.0, 4.0], 2, 2).expect("2x2"));

    // 原 `ground.cpp:64` 用未取整坐标判边界：1.9 在 2 行的图里**在界内**；
    // `ground.cpp:67` 的 `map[point.x][point.y]` 把它截断到 1（不是 round → 2）。
    let points = vec![Dot::new(0.9, 0.9), Dot::new(0.9, 1.9), Dot::new(1.9, 0.9)];
    let got = terrain.plane_at(&points).expect("三点定面");
    let want = fit_plane_up(&[
        Point3::new(0.9, 0.9, 1.0),
        Point3::new(0.9, 1.9, 2.0),
        Point3::new(1.9, 0.9, 3.0),
    ])
    .expect("同一组点");
    assert_close(got.a, want.a, "plane.a");
    assert_close(got.b, want.b, "plane.b");
    assert_close(got.c, want.c, "plane.c");
    assert_close(got.d, want.d, "plane.d");

    // 取整用 round 的话高度会变成 4，结果不同 —— 负向对照钉住截断语义。
    let rounded = fit_plane_up(&[
        Point3::new(0.9, 0.9, 1.0),
        Point3::new(0.9, 1.9, 2.0),
        Point3::new(1.9, 0.9, 4.0),
    ])
    .expect("同一组点");
    assert!(
        (got.a - rounded.a).abs() > 1e-6,
        "截断与四舍五入必须给出可区分的平面（a: {} vs {}）",
        got.a,
        rounded.a
    );
}

#[test]
fn plane_at_out_of_bounds_is_an_error() {
    let terrain = Terrain::from_grid(Grid::new(vec![0.0; 4], 2, 2).expect("2x2"));
    let points = vec![Dot::new(2.0, 0.0), Dot::new(0.0, 0.0), Dot::new(1.0, 1.0)];
    assert!(matches!(
        terrain.plane_at(&points),
        Err(CoreError::OutOfBounds { .. })
    ));
    assert!(matches!(
        terrain.normal_angle(&points),
        Err(CoreError::OutOfBounds { .. })
    ));

    // 退化：点不足 3 个 ⇒ DegeneratePlane（不是 OutOfBounds）。
    assert!(matches!(
        terrain.plane_at(&[Dot::new(0.0, 0.0), Dot::new(1.0, 1.0)]),
        Err(CoreError::DegeneratePlane)
    ));
}

#[test]
fn fit_plane_up_and_normal_angle_analytic_values() {
    // 水平面 z = 5 ⇒ 法向与 +z 夹角 0。
    let flat = fit_plane_up(&[
        Point3::new(0.0, 0.0, 5.0),
        Point3::new(1.0, 0.0, 5.0),
        Point3::new(0.0, 1.0, 5.0),
    ])
    .expect("水平面");
    assert_close(flat.normal_angle(), 0.0, "水平面 normal_angle");

    // 45° 斜面：n = (-1, 0, 1) ⇒ atan2(sqrt(1), 1) = PI/4。
    let tilted = fit_plane_up(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(0.0, 1.0, 0.0),
    ])
    .expect("45° 面");
    assert_close(
        tilted.normal_angle(),
        std::f64::consts::FRAC_PI_4,
        "45° normal_angle",
    );

    // 法向模为 0 ⇒ 返回 0.0（原 `geometry.cpp:1160-1167`）。
    assert_eq!(
        Plane {
            a: 0.0,
            b: 0.0,
            c: 0.0,
            d: 0.0
        }
        .normal_angle(),
        0.0
    );

    // 三点共线 / 点不足 ⇒ DegeneratePlane。
    assert!(matches!(
        fit_plane_up(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(2.0, 2.0, 2.0),
        ]),
        Err(CoreError::DegeneratePlane)
    ));
    assert!(matches!(
        fit_plane_up(&[Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)]),
        Err(CoreError::DegeneratePlane)
    ));
}
