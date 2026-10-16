//! 地形查询：在 [`Grid`] 之上做拟合平面、法向角与陡峭度评估。

use crate::error::{CoreError, Result};
use crate::geom::{Dot, Idx, Plane, Point3, fit_plane_up};
use crate::grid::Grid;

/// `steep_extend` 判死的障碍占比阈值：`obstacle_count / total_count >= 0.3`
/// （原 `src/aStar/aStar.cpp:243`）。
const OBSTACLE_RATIO_DEAD: f64 = 0.3;
/// 综合评分的三个权重（原 `src/aStar/aStar.cpp:283`）：标准差 / 最大高度差 / 平均坡度。
const WEIGHT_STDDEV: f64 = 0.4;
const WEIGHT_HEIGHT_DIFF: f64 = 0.3;
const WEIGHT_AVG_SLOPE: f64 = 0.3;

/// 地形 = 栅格 + 在其上的查询。
#[derive(Debug, Clone)]
pub struct Terrain {
    pub grid: Grid,
}

impl Terrain {
    pub fn from_grid(grid: Grid) -> Self {
        Self { grid }
    }

    /// 由若干采样点拟合平面，返回平面法向与竖直方向的夹角（弧度）。
    ///
    /// 等价于 `plane_at(pts)?.normal_angle()`；可站立判据用它与
    /// [`crate::config::MAX_NORMAL_ANGLE_RAD`] 比较（原 `foot.cpp:252` 的硬编码 20°）。
    pub fn normal_angle(&self, pts: &[Dot]) -> Result<f64> {
        Ok(self.plane_at(pts)?.normal_angle())
    }

    /// 由采样点拟合平面；点不足 / 越界 / 退化 ⇒ [`CoreError::DegeneratePlane`] / [`CoreError::OutOfBounds`]。
    ///
    /// ## 与原 `Ground::trip` 的采样对齐（`src/ground/ground.cpp:61-68`）
    ///
    /// 原实现逐点做两件事，本函数逐字复刻：
    /// 1. **边界用未取整的 `SqDot` 浮点坐标判**：`point.x < 0 || point.x >= map.rows()`
    ///    （`ground.cpp:64`，注意比较的是 `double` 与 `int` 的提升值），因此 `x = 4.9`
    ///    在 5 行的图上算**在界内**；
    /// 2. **取格是 double → int 截断**（`ground.cpp:67` 的 `map[point.x][point.y]`），
    ///    即 `4.9 → 4` —— 不是 [`Dot::as_idx`] 的四舍五入。
    ///
    /// 拟合用的三维点坐标是**未取整的原始浮点** `(x, y)` 加上取到的高度
    /// （`ground.cpp:67` 的 `CuDot{point.x, point.y, map[...]}`）。
    ///
    /// ## 越界的处理（本任务的决定点）
    ///
    /// 原实现在任一点越界时返回**默认全零 `CuPlane`**（`ground.cpp:64-66`；
    /// `CuPlane(double A=0.0,…)` 的默认值见 `include/utils/geometry.hpp:705`），
    /// 于是 `normal_angle()` 恒为 0.0（法向模为 0，`geometry.cpp:1160-1167`）。
    /// 这里**映射成 [`CoreError::OutOfBounds`]**，理由：
    ///
    /// - 唯一的**生产调用方** `Foot::standable` 在调用 `trip` 之前已经自己判过边界并对越界返回
    ///   `false`（`src/robot/foot.cpp:242-247`），所以「返回全零平面」这条路在原仓就是死分支；
    /// - 全零平面把「越界」压成「平坦、安全」（`normal_angle == 0.0` 一定小于 20° 阈值），
    ///   是典型的 fail-open：调用方拿不到任何可区分的信号。Rust 侧按 `error.rs` 的分类
    ///   显式返回错误，把判定留给调用方（fail-closed）。
    ///
    /// 点不足 3 个时由 [`fit_plane_up`] 返回 [`CoreError::DegeneratePlane`]
    /// （原实现同样返回全零平面，见 `ground.cpp:70-72`；理由同上：退化不得冒充「平坦」）。
    pub fn plane_at(&self, pts: &[Dot]) -> Result<Plane> {
        let rows = self.grid.rows() as f64;
        let cols = self.grid.cols() as f64;

        let mut points = Vec::with_capacity(pts.len());
        for p in pts {
            if p.x() < 0.0 || p.x() >= rows || p.y() < 0.0 || p.y() >= cols {
                return Err(CoreError::OutOfBounds { x: p.x(), y: p.y() });
            }
            let index = Idx::new(p.x() as i32, p.y() as i32);
            let height = self
                .grid
                .cell(index)
                .ok_or(CoreError::OutOfBounds { x: p.x(), y: p.y() })?;
            points.push(Point3::new(p.x(), p.y(), height));
        }

        fit_plane_up(&points)
    }

    /// 求区块 `(fi, se)` 内的平均陡峭度评分。
    ///
    /// 原 `steep_extend(graph, fi, se)`（`src/aStar/aStar.cpp:205-291`）在区块越界或不可行时
    /// 返回 `-1.0`；Rust 侧映射为 `None`，可行时为 `Some(陡峭度代价)`。
    ///
    /// `None` 只表示**真正不可行**：
    /// 1. 两端各自 `orth_near` 钳位后取包围盒；越界 ⇒ `None`（`aStar.cpp:218`。钳位后只有空图会触发）；
    /// 2. 障碍占比 `>= 0.3` ⇒ `None`（`aStar.cpp:243`）；
    /// 3. `heights` 为空 ⇒ `None`（`aStar.cpp:247`）。
    ///
    /// 陡峭度本身**不再是断路**：`0.4*stddev + 0.3*height_diff + 0.3*avg_slope` 一律作为
    /// `Some(代价)` 返回，由调用方乘权重计入代价（`astar::PENALTY_K`）。原实现的
    /// `> 10.0 ⇒ -1` 只在 `restore_block` 换算方向错误、这条规则始终空转时才无害；
    /// 方向修正后它会把粗图按台阶切成条带、让跨条带目标在高层不可达
    /// （实测与登记见 `doc/known_divergences.md` D4）。单点时评分为 `0.0`（`aStar.cpp:251`）。
    ///
    /// 统计量按原运算顺序：`height == +inf` 记障碍、`height >= 0` 收进 `heights`（其余负数忽略），
    /// 标准差是总体标准差（除以 `n`），`avg_slope` 是**按 `heights` 收集顺序**的相邻绝对差均值。
    ///
    /// **有意的行为修正**：原 `aStar.cpp:212` 写的是
    /// `std::min(bounded_first.x, bounded_first.x)`（笔误，等价于 `bounded_first.x`）。
    /// 本实现按语义取 `min(bounded_first.x, bounded_second.x)`。当 `bounded_first.x <=
    /// bounded_second.x` 时两者完全一致；只有 `bounded_first.x > bounded_second.x` 时，
    /// 原实现只扫 `bounded_first.x` 这一列，本实现扫完整区间。
    pub fn steep_extend(&self, block: (Idx, Idx)) -> Option<f64> {
        let (fi, se) = block;
        let bounded_first = orth_near(&self.grid, fi);
        let bounded_second = orth_near(&self.grid, se);

        // 有意修正 aStar.cpp:212 的笔误，见方法文档。
        let min_x = bounded_first.x.min(bounded_second.x);
        let max_x = bounded_first.x.max(bounded_second.x);
        let min_y = bounded_first.y.min(bounded_second.y);
        let max_y = bounded_first.y.max(bounded_second.y);

        let rows = self.grid.rows() as i32;
        let cols = self.grid.cols() as i32;
        if min_x < 0 || max_x >= rows || min_y < 0 || max_y >= cols {
            return None;
        }

        // 一遍扫窗口，累计统计量，**不物化 `heights`**（原来每次调用都会 push 进一个约 1600 元素的
        // `Vec`，在 `scale_star` 的每次节点求值里都是一次分配 + 多次扩容）。
        // 累加顺序与「先收集成 `Vec` 再遍历」完全一致，所以 `mean` / `variance` / `avg_slope`
        // 的浮点结果逐位相同（`avg_slope` 只对**被收集**的相邻高度作差，`prev` 只在收集时更新）。
        let mut obstacle_count: usize = 0;
        let mut total_count: usize = 0;
        let mut count: usize = 0;
        let mut sum = 0.0_f64;
        let mut min_height = f64::INFINITY;
        let mut max_height = f64::NEG_INFINITY;
        let mut slope_sum = 0.0_f64;
        let mut previous: Option<f64> = None;
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                total_count += 1;
                // 包围盒已钳在本图内，取格不会失败；真失败就按不可行处理（fail-closed）。
                let height = self.grid.cell(Idx::new(x, y))?;
                if height == f64::INFINITY {
                    obstacle_count += 1;
                } else if height >= 0.0 {
                    count += 1;
                    sum += height;
                    min_height = min_height.min(height);
                    max_height = max_height.max(height);
                    if let Some(previous) = previous {
                        slope_sum += (height - previous).abs();
                    }
                    previous = Some(height);
                }
            }
        }

        if total_count > 0 && (obstacle_count as f64 / total_count as f64) >= OBSTACLE_RATIO_DEAD {
            return None;
        }
        if count == 0 {
            return None;
        }
        if count == 1 {
            return Some(0.0);
        }

        let count = count as f64;
        let mean = sum / count;
        // 第二遍按同一顺序累加平方差 —— 与 `heights.iter().map(|h| (h-mean)^2).sum()` 逐位相同。
        let mut variance_sum = 0.0_f64;
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                let height = self.grid.cell(Idx::new(x, y))?;
                if height != f64::INFINITY && height >= 0.0 {
                    let diff = height - mean;
                    variance_sum += diff * diff;
                }
            }
        }
        let stddev = (variance_sum / count).sqrt();
        let height_diff = max_height - min_height;
        let avg_slope = slope_sum / (count - 1.0);

        // 陡峭度是代价，不是断路：一律返回，由调用方（`astar::PENALTY_K`）加权。
        Some(
            WEIGHT_STDDEV * stddev
                + WEIGHT_HEIGHT_DIFF * height_diff
                + WEIGHT_AVG_SLOPE * avg_slope,
        )
    }
}

/// 原 `SqPlane::orth_near`（`src/utils/geometry.cpp:549-553`）：`max(0, min(v, size - 1))`。
///
/// 空图（`size == 0`）时原式给 `max(0, min(v, -1)) == 0`，这里保持一致（`max` 先抬到 0）。
fn orth_near(grid: &Grid, i: Idx) -> Idx {
    let max_x = (grid.rows() as i32 - 1).max(0);
    let max_y = (grid.cols() as i32 - 1).max(0);
    Idx::new(i.x.clamp(0, max_x), i.y.clamp(0, max_y))
}
