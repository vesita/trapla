//! 足部几何：覆盖格、四角、可站立判定与滑动修正。

use std::collections::BTreeSet;

use crate::config::{FootShape, RobotConfig};
use crate::geom::{Dot, Idx};
use crate::terrain::Terrain;

/// 滑动的结果。
///
/// 原 C++ 的 `SlideResult` 只用 `Modified` 表示「原地修改了 area」（`src/robot/foot.cpp:143`），
/// 不携带位移；Rust 侧额外把累计平移量 `dx/dy` 带出来，便于指标与调试。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlideResult {
    /// 被修正：相对原位置平移了 `(dx, dy)`（累计平移量）。
    Modified { dx: f64, dy: f64 },
    /// 无需修正（扫描后原位置已是最优）。
    NoModification,
    /// 不适用（例如覆盖点越界，原实现直接返回 false 的那条路径）。
    NotApplicable,
}

/// 一只脚。
#[derive(Debug, Clone, PartialEq)]
pub struct Foot {
    pub position: Dot,
    pub rz: f64,
    pub shape: FootShape,
}

impl Foot {
    pub fn new(position: Dot, rz: f64, shape: FootShape) -> Self {
        Self {
            position,
            rz,
            shape,
        }
    }

    /// 足部覆盖的栅格集合（原 `Foot::cover`，`src/robot/foot.cpp:195-215`）。
    ///
    /// 按 `(x, y)` 字典序排序，保证规划确定性与 golden 输出一致。
    pub fn cover(&self) -> Vec<Idx> {
        let half_length = self.shape.half_length();
        let half_width = self.shape.half_width();
        let scan_step = self.shape.scan_step;

        let cos_rz = self.rz.cos();
        let sin_rz = self.rz.sin();

        let mut point_set = BTreeSet::new();

        let mut l = -half_length;
        while l <= half_length {
            let mut w = -half_width;
            while w <= half_width {
                let world_l = self.position.x() + l * cos_rz - w * sin_rz;
                let world_w = self.position.y() + l * sin_rz + w * cos_rz;
                point_set.insert((world_l.round() as i32, world_w.round() as i32));
                w += scan_step;
            }
            l += scan_step;
        }

        point_set.into_iter().map(|(x, y)| Idx::new(x, y)).collect()
    }

    /// 足部四角（原 `Foot::corner`，`src/robot/foot.cpp:217-236`）。
    ///
    /// 返回顺序严格与原实现一致：
    /// 1. `(x + l_cos - w_sin, y + l_sin + w_cos)`
    /// 2. `(x + l_cos + w_sin, y + l_sin - w_cos)`
    /// 3. `(x - l_cos + w_sin, y - l_sin - w_cos)`
    /// 4. `(x - l_cos - w_sin, y - l_sin + w_cos)`
    pub fn corners(&self) -> [Dot; 4] {
        let half_length = self.shape.half_length();
        let half_width = self.shape.half_width();
        let l_cos = half_length * self.rz.cos();
        let l_sin = half_length * self.rz.sin();
        let w_cos = half_width * self.rz.cos();
        let w_sin = half_width * self.rz.sin();

        [
            Dot::new(
                self.position.x() + l_cos - w_sin,
                self.position.y() + l_sin + w_cos,
            ),
            Dot::new(
                self.position.x() + l_cos + w_sin,
                self.position.y() + l_sin - w_cos,
            ),
            Dot::new(
                self.position.x() - l_cos + w_sin,
                self.position.y() - l_sin - w_cos,
            ),
            Dot::new(
                self.position.x() - l_cos - w_sin,
                self.position.y() - l_sin + w_cos,
            ),
        ]
    }

    /// 是否可站立：覆盖点全部在界内，且拟合平面法向角 < [`crate::config::MAX_NORMAL_ANGLE_RAD`]。
    ///
    /// 原实现 `src/robot/foot.cpp:238-253`：
    /// 先把覆盖点逐个查边界（越界直接返回 false），再通过 `trip` 拟合平面计算法向角，
    /// 与 [`crate::config::MAX_NORMAL_ANGLE_RAD`] 严格比较（`<`）。
    ///
    /// 判定本身在 [`crate::constraints::check_standable`]（硬约束的唯一实现点），这里只是
    /// 冻结谓词的薄包装。本签名不带配置，阈值取 [`RobotConfig::default`] 的
    /// `max_normal_angle`（= [`crate::config::MAX_NORMAL_ANGLE_RAD`]）；规划器路径
    /// （`Robot::bfs`）走 `self.config`，好让 `--overrides` 换掉的阈值对规划器也生效。
    pub fn standable(&self, t: &Terrain) -> bool {
        let config = RobotConfig::default();
        let verdicts = crate::constraints::check_standable(self, t, &config);
        crate::constraints::all_satisfied(&verdicts)
    }

    /// 在当前位置附近扫描，找出法向角更优的落脚点；必要时返回平移量。
    ///
    /// 原实现的两阶段扫描见 `src/robot/foot.cpp:46-148`。
    pub fn slide(&self, t: &Terrain) -> SlideResult {
        let rows = t.grid.rows();
        let cols = t.grid.cols();
        if rows == 0 || cols == 0 {
            return SlideResult::NotApplicable;
        }

        let corners = self.corners();
        let Ok(plane) = t.plane_at(&corners) else {
            return SlideResult::NoModification;
        };

        let xy_mag = (plane.a * plane.a + plane.b * plane.b).sqrt();
        if xy_mag == 0.0 {
            return SlideResult::NoModification;
        }

        let slide_x = plane.a;
        let slide_y = plane.b;

        let original_angle = plane.normal_angle();
        let mut best_angle = original_angle;
        let mut best_dx = 0.0;
        let mut best_dy = 0.0;
        let mut found_better = false;

        const MAX_ITERATIONS: i32 = 3;

        // 第一阶段：正向扫描
        for i in 1..=MAX_ITERATIONS {
            let dx = slide_x * (i as f64);
            let dy = slide_y * (i as f64);
            let mut all_valid = true;
            let mut new_pts = Vec::with_capacity(corners.len());

            for pt in &corners {
                let new_x = (pt.x() + dx) as i32;
                let new_y = (pt.y() + dy) as i32;
                if new_x < 0 || new_x >= rows as i32 || new_y < 0 || new_y >= cols as i32 {
                    all_valid = false;
                    break;
                }
                new_pts.push(Dot::new(new_x as f64, new_y as f64));
            }

            if !all_valid {
                continue;
            }
            if let Some(angle) = t.normal_angle(&new_pts).ok().filter(|&a| a < best_angle) {
                best_angle = angle;
                best_dx = dx;
                best_dy = dy;
                found_better = true;
            }
        }

        if found_better {
            return SlideResult::Modified {
                dx: best_dx,
                dy: best_dy,
            };
        }

        // 第二阶段：反向扫描
        for i in 1..=MAX_ITERATIONS {
            let dx = -slide_x * (i as f64);
            let dy = -slide_y * (i as f64);
            let mut all_valid = true;
            let mut new_pts = Vec::with_capacity(corners.len());

            for pt in &corners {
                let new_x = (pt.x() + dx) as i32;
                let new_y = (pt.y() + dy) as i32;
                if new_x < 0 || new_x >= rows as i32 || new_y < 0 || new_y >= cols as i32 {
                    all_valid = false;
                    break;
                }
                new_pts.push(Dot::new(new_x as f64, new_y as f64));
            }

            if !all_valid {
                continue;
            }
            if let Some(angle) = t.normal_angle(&new_pts).ok().filter(|&a| a < best_angle) {
                best_angle = angle;
                best_dx = dx;
                best_dy = dy;
                found_better = true;
            }
        }

        if found_better {
            return SlideResult::Modified {
                dx: best_dx,
                dy: best_dy,
            };
        }

        SlideResult::NoModification
    }

    /// 足部外接半径（原 `FootShape::about_R() = (length + width) / 4.0`，`src/robot/foot.cpp:150-152`）。
    pub fn about_r(&self) -> f64 {
        (self.shape.length + self.shape.width) / 4.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_foot_corners_and_cover() {
        let foot = Foot::new(Dot::new(0.0, 0.0), 0.0, FootShape::biped());
        let corners = foot.corners();
        assert_eq!(corners[0], Dot::new(2.5, 1.5));
        assert_eq!(corners[1], Dot::new(2.5, -1.5));
        assert_eq!(corners[2], Dot::new(-2.5, -1.5));
        assert_eq!(corners[3], Dot::new(-2.5, 1.5));

        let cover = foot.cover();
        assert_eq!(cover.len(), 35);
        assert_eq!(cover[0], Idx::new(-3, -2));
        assert_eq!(cover[cover.len() - 1], Idx::new(3, 2));
    }
}
