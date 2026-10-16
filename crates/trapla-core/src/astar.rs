//! A* 搜索与方向权重（原 `aStar.cpp` / `direction.cpp`）。
//!
//! 启发式固定为**曼哈顿距离**（四邻域下可采纳）。原 C++ 的 `a_star_search` 通过
//! `get_neighbour_shuffled()` 打乱邻居顺序，导致同图同起终点结果不稳定；
//! Rust 侧 [`a_star_search`] 没有 `rng` 参数 ⇒ 邻居顺序固定 ⇒ 结果确定。
//!
//! **文档与代码的矛盾（保留原语义，不「顺手改对」）**：`src/aStar/aStar.cpp:6` 的文件头注释
//! 与 `doc/project.md:103` 都写「使用欧几里得距离作为启发式」，但 `aStar.cpp:33` 实际调用的是
//! `manhattan_distance(next, goal)` —— 即 `|dx| + |dy|`。本移植**保留曼哈顿**；`scale_star`
//! 的启发式（`aStar.cpp:117`）确实是 `euclidean_distance`，两者不同，不是笔误。
//!
//! 消灭的原仓重复：
//! - `SqPlane::find_path`（`src/utils/geometry.cpp:584-649`）是第二套 A*，原仓无人调用 ⇒ 不移植；
//! - `direction_determine` 的 `SqDot` / `Intex` 两个重载（`direction.cpp:3-19` / `21-37`）
//!   只差一次 `Idx::as_dot` ⇒ 收敛成 [`direction_determine`]（只收 `&[Dot]`）；
//! - `square_decay` 两个重载（`direction.cpp:62-76`）无人调用，且 `square_decay(double,int)`
//!   把 `start` 隐式截断后传给 `square_decay(int)` ⇒ 删除。

use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap};

use crate::geom::{Dir4, Dot, Idx, index_scale};
use crate::grid::Grid;
use crate::rng;
use crate::terrain::Terrain;

/// 搜索结果。
#[derive(Debug, Clone)]
pub struct Search {
    /// 从起点到终点的格子序列（含起终点）。
    pub path: Vec<Idx>,
    /// 扩展（出队）的节点数，**含判定到达终点的那一次 pop**。
    pub expanded: usize,
}

/// 优先队列条目：`f64` 优先级 + 单调递增的入队序号。
///
/// `f64::total_cmp` 给出全序（NaN 也排得进），序号保证同优先级时 **FIFO**，
/// 使整个搜索不依赖堆内部顺序 ⇒ 同输入结果逐点可复现。
#[derive(Debug, Clone, Copy, PartialEq)]
struct Frontier {
    priority: f64,
    order: u64,
    index: Idx,
}

impl Eq for Frontier {}

impl Ord for Frontier {
    fn cmp(&self, other: &Self) -> Ordering {
        self.priority
            .total_cmp(&other.priority)
            .then_with(|| self.order.cmp(&other.order))
    }
}

impl PartialOrd for Frontier {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 棋盘距离 `|dx| + |dy|`（原 `manhattan_distance(const Intex&, const Intex&)`，
/// `src/utils/geometry.cpp:14-16`）—— `a_star_search` 的启发式。
fn manhattan(a: Idx, b: Idx) -> f64 {
    (a.x as i64 - b.x as i64).unsigned_abs() as f64
        + (a.y as i64 - b.y as i64).unsigned_abs() as f64
}

/// 欧氏距离 `sqrt(dx² + dy²)`（原 `euclidean_distance(const Intex&, const Intex&)`，
/// `src/utils/geometry.cpp:29-31`）—— `scale_star` 的启发式。
fn euclidean(a: Idx, b: Idx) -> f64 {
    let dx = (a.x as i64 - b.x as i64) as f64;
    let dy = (a.y as i64 - b.y as i64) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// 在 `g` 上做 A*，启发式为曼哈顿距离 `|dx| + |dy|`。
///
/// 返回 `None` 表示不可达（或起终点本身越界/是障碍）。
/// 邻居顺序用 [`Grid::neighbours4`] 的固定顺序，保证可复现。
///
/// 与原 `a_star_search`（`src/aStar/aStar.cpp:14-64`）的差异（都是确定化 / 契约要求，不是「改对」）：
/// - 原实现只按 `edge_allowed` 过滤邻居（`geometry.cpp:407-419`：**越界与 `+inf` 障碍都拒**），
///   这里同样在 [`Grid::neighbours4`]（只滤越界）之上再滤 [`Grid::is_obstacle`]；
/// - 原实现沿 `came_from` 回走时不检查终点到底有没有被到达：不可达时会走回哨兵 `(-1,-1)`，
///   然后因为 `!path.empty()` 为真而返回一条假的 `[start, goal]`。契约规定不可达返回 `None`，
///   本实现改为回走前先确认终点已入队，回走时 `came_from` 取不到就返回 `None`（不会 panic）；
/// - 原实现用 `unordered_map` + 无序号优先队列，同代价节点的出队顺序不确定；这里用
///   单调序号做 FIFO 决胜（见 [`Frontier`]）。
pub fn a_star_search(g: &Grid, start: Idx, goal: Idx) -> Option<Search> {
    if !g.in_bounds(start) || !g.in_bounds(goal) || g.is_obstacle(start) || g.is_obstacle(goal) {
        return None;
    }

    let mut frontier: BinaryHeap<Reverse<Frontier>> = BinaryHeap::new();
    let mut order: u64 = 0;
    frontier.push(Reverse(Frontier {
        priority: 0.0,
        order,
        index: start,
    }));

    let mut came_from: HashMap<Idx, Idx> = HashMap::new();
    let mut cost_so_far: HashMap<Idx, f64> = HashMap::new();
    cost_so_far.insert(start, 0.0);

    let mut expanded = 0usize;
    let mut reached = start == goal;

    while let Some(Reverse(entry)) = frontier.pop() {
        let current = entry.index;
        expanded += 1;
        if current == goal {
            reached = true;
            break;
        }

        let base = cost_so_far[&current];
        for next in g.neighbours4(current) {
            if g.is_obstacle(next) {
                continue;
            }
            let new_cost = base + g.cost(current, next);
            let improved = match cost_so_far.get(&next) {
                None => true,
                Some(&old) => new_cost < old,
            };
            if improved {
                cost_so_far.insert(next, new_cost);
                order += 1;
                frontier.push(Reverse(Frontier {
                    priority: new_cost + manhattan(next, goal),
                    order,
                    index: next,
                }));
                came_from.insert(next, current);
            }
        }
    }

    if !reached {
        return None;
    }

    let mut path = Vec::new();
    let mut current = goal;
    let mut steps = 0usize;
    let max_steps = g.rows().saturating_mul(g.cols());
    loop {
        path.push(current);
        if current == start {
            break;
        }
        current = came_from.get(&current).copied()?;
        steps += 1;
        if steps > max_steps {
            return None;
        }
    }
    path.reverse();
    Some(Search { path, expanded })
}

/// 陡峭度代价的权重 `PENALTY_K`（量纲：代价单位 / 高度单位）。
///
/// `Terrain::steep_extend` 返回的是**高度单位**的综合评分
/// （`0.4*stddev + 0.3*height_diff + 0.3*avg_slope`），而单步代价
/// `Grid::cost` = 曼哈顿距离（≈1）+ 原图对应格高度。后者在本仓 `data/csv/map.csv` 上
/// 实测是 `1 + 27..123`，而缩放坐标只落在左上角 `25 × 75` 的区块里、取值约 27~31 ⇒ **单步 ≈ 31**。
///
/// 取 `K = 3.0` 的锚点：**原来被硬判死（`score > 10.0`）的窗口，现在恰好相当于多走一格**
/// （`3.0 × 10 = 30 ≈ 31`）。低于这个值，陡峭度只是同长度路径之间的 tie-break；
/// 高于它，搜索会为大范围绕行买单。敏感性实测见 `doc/known_divergences.md` D4。
pub const PENALTY_K: f64 = 3.0;

/// 缩放图上的搜索：先按 `scale` 降采样搜索，再把路径还原到原图。
///
/// 就是 [`scale_star_with_penalty`] 取 `penalty_k = PENALTY_K`。
pub fn scale_star(
    t: &Terrain,
    start: Idx,
    goal: Idx,
    scale: f64,
    rng: &mut rng::Rng,
) -> Option<Search> {
    scale_star_with_penalty(t, start, goal, scale, rng, PENALTY_K)
}

/// [`scale_star`] 的实现体；陡峭度权重 `penalty_k` 显式给（`0.0` = 完全忽略地形）。
///
/// 原实现对应 `scale_star`（`src/aStar/aStar.cpp:78-137`）。`rng` 的用途在这里：
/// 原 `current.get_neighbour_shuffled(sr, sc)`（`aStar.cpp:100`）每步洗牌邻居顺序，
/// 本实现同样每步 [`rng::Rng::shuffle`] 一次 —— 这是可复现性的唯一随机落点，
/// **不许**自建随机源，也不许改成固定顺序（那会偏离原算法的探索顺序）。
///
/// 逐条复刻的要点：
/// - 缩放索引 `ss`/`sg` 与网格尺寸 `sr`/`sc` 全走 `ceil(v * scale)`
///   （`Intex::scale` → `index_scale`，`index.cpp:43-45`、`geometry.cpp:839-860`）；
/// - `graph.cost(current, next)` 与 `steep_extend` 都用**原图**（`aStar.cpp:107,114`），
///   其中 `cost` 拿缩放坐标去索引原图 —— 原实现如此，照抄；
/// - 邻居代价 `= cost + steep_extend * penalty_k`：陡峭度进**代价**、不进断路
///   （`steep_extend` 返回 `None` 只表示越界 / 障碍占比 ≥ 0.3 / 无可用高度）；
/// - 启发式用欧氏距离（`aStar.cpp:117`）；
/// - 终端的「引导点」序列：`[goal]` + 从 `came_from[sg]` 回走到 `ss`（不含）的每个区块经
///   `restore_dot` 还原 + `[start]`，最后整体反转（`aStar.cpp:128-135`）。
///
/// 与原实现的差异（确定化 / 契约要求）：
/// - 非法 `scale`（`<= 0` / 非有限）、空图、起终点缩放后越出缩放网格 ⇒ 返回 `None`
///   （原实现直接索引 / 分配，是 UB / 崩溃）；
/// - 不可达时返回 `None`（原实现回退成假的 `[start, goal]`）；`ss == sg` 时原样返回
///   `[start, goal]`，与原实现一致。
pub fn scale_star_with_penalty(
    t: &Terrain,
    start: Idx,
    goal: Idx,
    scale: f64,
    rng: &mut rng::Rng,
    penalty_k: f64,
) -> Option<Search> {
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }

    let scaled_rows = index_scale(t.grid.rows() as f64, scale);
    let scaled_cols = index_scale(t.grid.cols() as f64, scale);
    if scaled_rows <= 0 || scaled_cols <= 0 {
        return None;
    }

    let in_scaled = |i: Idx| i.x >= 0 && i.y >= 0 && i.x < scaled_rows && i.y < scaled_cols;
    let scaled = |i: Idx| {
        Idx::new(
            index_scale(i.x as f64, scale),
            index_scale(i.y as f64, scale),
        )
    };

    let ss = scaled(start);
    let sg = scaled(goal);
    if !in_scaled(ss) || !in_scaled(sg) {
        return None;
    }

    let mut frontier: BinaryHeap<Reverse<Frontier>> = BinaryHeap::new();
    let mut order: u64 = 0;
    frontier.push(Reverse(Frontier {
        priority: 0.0,
        order,
        index: ss,
    }));

    let mut came_from: HashMap<Idx, Idx> = HashMap::new();
    let mut cost_so_far: HashMap<Idx, f64> = HashMap::new();
    cost_so_far.insert(ss, 0.0);

    let mut expanded = 0usize;
    let mut reached = ss == sg;

    // 陡峭度的惰性记忆化：`steep_extend(restore_block(next, scale))` **只取决于缩放节点 `next`**，
    // 与从哪个节点走来无关，而同一节点会作为多个节点的邻居被反复求值；每次求值要扫 1681 格并分配
    // 一个约 1600 元素的 `Vec`。这里按节点缓存第一次的结果。
    //
    // 槽位：`None` = 还没求值；`Some(None)` = 已求值且**不可行**（越界 / 障碍占比 ≥ 0.3 / 无可用高度）；
    // `Some(Some(v))` = 已求值，代价 `v`。外层 `Option` 就是「是否已求值」这一位。
    type SteepSlot = Option<Option<f64>>;
    let mut steep_cache: Vec<SteepSlot> = vec![None; scaled_rows as usize * scaled_cols as usize];
    let stride = scaled_cols as usize;
    let mut steep_at = |next: Idx| -> Option<f64> {
        let slot = next.x as usize * stride + next.y as usize;
        if let Some(cached) = steep_cache[slot] {
            return cached;
        }
        let value = t.steep_extend(t.grid.restore_block(next, scale));
        steep_cache[slot] = Some(value);
        value
    };

    while let Some(Reverse(entry)) = frontier.pop() {
        let current = entry.index;
        expanded += 1;
        if current == sg {
            reached = true;
            break;
        }

        let base = cost_so_far[&current];
        let mut neighbours: Vec<Idx> = Dir4::ALL
            .iter()
            .map(|dir| {
                let (dx, dy) = dir.delta();
                Idx::new(current.x + dx, current.y + dy)
            })
            .filter(|next| in_scaled(*next))
            .collect();
        rng.shuffle(&mut neighbours);

        for next in neighbours {
            let Some(steep) = steep_at(next) else {
                continue;
            };
            let new_cost = base + t.grid.cost(current, next) + steep * penalty_k;
            let improved = match cost_so_far.get(&next) {
                None => true,
                Some(&old) => new_cost < old,
            };
            if improved {
                cost_so_far.insert(next, new_cost);
                order += 1;
                frontier.push(Reverse(Frontier {
                    priority: new_cost + euclidean(next, sg),
                    order,
                    index: next,
                }));
                came_from.insert(next, current);
            }
        }
    }

    if !reached {
        return None;
    }

    let mut chain = Vec::new();
    let mut current = came_from.get(&sg).copied();
    let guard_limit = (scaled_rows as usize).saturating_mul(scaled_cols as usize);
    while let Some(block) = current {
        if block == ss {
            break;
        }
        chain.push(t.grid.restore_dot(block, scale).as_idx());
        current = came_from.get(&block).copied();
        if chain.len() > guard_limit {
            return None;
        }
    }

    let mut path = Vec::with_capacity(chain.len() + 2);
    path.push(start);
    path.extend(chain.iter().rev().copied());
    path.push(goal);
    Some(Search { path, expanded })
}

/// 等比数列部分和 `a * (1 + alpha + … + alpha^(n-1))`（原 `geometric_sum`，
/// `src/utils/sequence.cpp:3-7`）：`n == 0 ⇒ 0`、`alpha == 1 ⇒ a * n`、否则闭式。
fn geometric_sum(a: f64, alpha: f64, n: usize) -> f64 {
    if n == 0 {
        return 0.0;
    }
    if alpha == 1.0 {
        return a * n as f64;
    }
    a * (1.0 - alpha.powi(n as i32)) / (1.0 - alpha)
}

/// 首项为 `start` 的几何衰减系数（原 `geometric_decay(start, alpha, n)`，
/// `src/aStar/direction.cpp:51-60`）。
///
/// 第 `order` 项（`order = 1..=n`）是 `start * alpha^(order-1) / total`，**最后整体反转** ——
/// 所以返回向量里 `coefficient[0]` 对应 `order = n`、`coefficient[n-1]` 对应 `order = 1`。
/// `n == 0` 时返回空向量（`total` 为 0，但不做除法）。
pub fn geometric_decay_start(start: f64, alpha: f64, n: usize) -> Vec<f64> {
    let total = geometric_sum(start, alpha, n);
    let mut coefficient: Vec<f64> = (1..=n)
        .map(|order| start * alpha.powi(order as i32 - 1) / total)
        .collect();
    coefficient.reverse();
    coefficient
}

/// 首项为 1 的几何衰减系数（原 `geometric_decay(alpha, n)`，`src/aStar/direction.cpp:39-49`）。
pub fn geometric_decay(alpha: f64, n: usize) -> Vec<f64> {
    geometric_decay_start(1.0, alpha, n)
}

/// 按几何衰减权重对引导点做加权平均（原 `direction_determine`，
/// `src/aStar/direction.cpp:3-37`；`SqDot` / `Intex` 两个重载收敛到这里）。
///
/// `guides` 里 `Intex` 的那一路在调用前用 [`Idx::as_dot`] 转成 [`Dot`]，因此本函数只需一个入口。
///
/// **删掉了原签名的 `at` 形参**：原仓两个重载都收 `const SqDot& at` 却从未读取它
/// （`direction.cpp:3-19` / `21-37`，`at` 在函数体里零次出现），保留一个假参数会误导调用方。
/// 权重用 `geometric_decay(sqrt(PI), n)`（`direction.cpp:10,28` 的 `sqrt(M_PI)`）；
/// `near_end` 为真时把系数**再反转一次**（`direction.cpp:11-13`）。
/// `guides` 为空时原实现返回零向量；这里返回 `Dot(0, 0)`。
pub fn direction_determine(guides: &[Dot], near_end: bool) -> Dot {
    if guides.is_empty() {
        return Dot::new(0.0, 0.0);
    }

    let mut coefficient = geometric_decay(std::f64::consts::PI.sqrt(), guides.len());
    if near_end {
        coefficient.reverse();
    }

    let mut result = Dot::new(0.0, 0.0);
    for (guide, shift) in guides.iter().zip(coefficient.iter()) {
        result = Dot::new(
            result.x() + guide.x() * shift,
            result.y() + guide.y() * shift,
        );
    }
    result
}
