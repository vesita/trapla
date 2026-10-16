//! 规划器主循环：在 [`Terrain`] 上交替落左右脚，产出点位序列。

use crate::config::RobotConfig;
use crate::error::{CoreError, Result};
use crate::foot::Foot;
use crate::geom::{Dot, Line};
use crate::rng;
use crate::terrain::Terrain;

/// 当前应该移动哪只脚（原 `WhichFoot`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WhichFoot {
    Left,
    Right,
}

/// 通道半宽（cm）：左右脚各自沿「本体前进方向」的垂线偏移该值落足（路线 D：足印分道）。
///
/// 物理含义 = 目标双足横向间距的一半。`Robot::new` 的初始双足间距是
/// `0.6 * min_separation + 0.4 * max_separation = 5.2`；走廊（`satisfy_spacing`）在足轴与
/// 支撑脚平行时把足中心侧距限制在 `[5, 13]`，因此本参数的有效上限 ≈ 13。
/// 取值依据与三取值敏感性见 `doc/catwalk_design.md`。
const LANE_HALF_WIDTH_CM: f64 = 13.0;

/// 朝向平滑系数（路线 B）：`next()` 把新脚朝向按此比例拉向本步步向，1.0 = 原行为。
///
/// 物理含义 = 一步最多把足朝向拉向实际步向几成；取值依据与敏感性见 `doc/catwalk_design.md`。
const HEADING_SMOOTHING: f64 = 0.5;

/// 双足机器人状态。
#[derive(Debug, Clone)]
pub struct Robot {
    pub(crate) config: RobotConfig,
    pub(crate) feet: [Foot; 2],
    pub(crate) now_which_foot_to_move: WhichFoot,
}

impl Robot {
    /// `start` 是双脚中心（支撑中点）的初始位置。
    ///
    /// 双脚对称放置在中心两侧，间距取 `0.6 * min_sep + 0.4 * max_sep`（原 `robot.cpp:22-26`）。
    pub fn new(config: RobotConfig, start: Dot) -> Self {
        let sep = 0.6 * config.min_separation + 0.4 * config.max_separation;
        let left_pos = Dot::new(start.x(), start.y() - sep / 2.0);
        let right_pos = Dot::new(start.x(), start.y() + sep / 2.0);

        let feet = [
            Foot::new(left_pos, 0.0, config.shape),
            Foot::new(right_pos, 0.0, config.shape),
        ];

        Self {
            config,
            feet,
            now_which_foot_to_move: WhichFoot::Left,
        }
    }

    /// 获取规划器配置。
    pub fn config(&self) -> &RobotConfig {
        &self.config
    }

    /// 获取两足的不可变引用（`feet[0]` 是左脚，`feet[1]` 是右脚）。
    pub fn feet(&self) -> &[Foot; 2] {
        &self.feet
    }

    /// 获取两足的可变引用。
    pub fn feet_mut(&mut self) -> &mut [Foot; 2] {
        &mut self.feet
    }

    /// 获取当前应移动的脚。
    pub fn which_foot(&self) -> WhichFoot {
        self.now_which_foot_to_move
    }

    /// 设置当前应移动的脚。
    pub fn set_which_foot(&mut self, which: WhichFoot) {
        self.now_which_foot_to_move = which;
    }

    /// 设置双脚位置和朝向（测试与 golden 夹具装配用）。
    pub fn set_feet(&mut self, left: Foot, right: Foot, which: WhichFoot) {
        self.feet[0] = left;
        self.feet[1] = right;
        self.now_which_foot_to_move = which;
    }

    /// 获取当前摆动脚的引用。
    pub fn swing_foot(&self) -> &Foot {
        match self.now_which_foot_to_move {
            WhichFoot::Left => &self.feet[0],
            WhichFoot::Right => &self.feet[1],
        }
    }

    /// 获取当前摆动脚的可变引用。
    pub fn swing_foot_mut(&mut self) -> &mut Foot {
        match self.now_which_foot_to_move {
            WhichFoot::Left => &mut self.feet[0],
            WhichFoot::Right => &mut self.feet[1],
        }
    }

    /// 获取当前支撑脚的引用。
    pub fn support_foot(&self) -> &Foot {
        match self.now_which_foot_to_move {
            WhichFoot::Left => &self.feet[1],
            WhichFoot::Right => &self.feet[0],
        }
    }

    /// 获取当前支撑脚的可变引用。
    pub fn support_foot_mut(&mut self) -> &mut Foot {
        match self.now_which_foot_to_move {
            WhichFoot::Left => &mut self.feet[1],
            WhichFoot::Right => &mut self.feet[0],
        }
    }

    /// 切换摆动脚与支撑脚身份。
    pub fn change_feet(&mut self) {
        self.now_which_foot_to_move = match self.now_which_foot_to_move {
            WhichFoot::Left => WhichFoot::Right,
            WhichFoot::Right => WhichFoot::Left,
        };
    }

    /// 当前双脚中心位置。
    pub fn position(&self) -> Dot {
        Dot::new(
            (self.feet[0].position.x() + self.feet[1].position.x()) / 2.0,
            (self.feet[0].position.y() + self.feet[1].position.y()) / 2.0,
        )
    }

    /// 计算摆动脚到新位置的距离。
    pub fn distance(&self, new_pos: Dot) -> f64 {
        self.swing_foot().position.distance(new_pos)
    }

    /// 根据新位置计算摆动脚的下一步状态（原 `Robot::next`，`src/robot/robot.cpp:635-646`）。
    ///
    /// 步长 >= `active_distance` 才更新 `rz`，否则保持原朝向。
    ///
    /// **语义被 golden 钉死**：`satisfy_spacing`/`satisfy_turn` 内部都调用本函数做几何判定，
    /// 所以这里保持原实现不动。路线 B 的朝向解耦在提交路径 [`Robot::next_committed`] 里做。
    pub fn next(&self, new_pos: Dot) -> Foot {
        let sw = self.swing_foot();
        let stride = sw.position.distance(new_pos);
        let new_rz = if stride >= self.config.active_distance {
            sw.position.angle(new_pos)
        } else {
            sw.rz
        };
        Foot::new(new_pos, new_rz, sw.shape)
    }

    /// 提交落足时使用的脚状态（路线 B：朝向解耦）。
    ///
    /// 与 [`Robot::next`] 的唯一差别是 `rz`：不再直接等于步向，而是按 `HEADING_SMOOTHING`
    /// 向步向插值。原实现里「步向 = 新脚朝向」是 T6 负结果里参考系自转的源头——步向带着
    /// π/12 网格抖动，抖动又被写进 `rz`，下一轮候选窗口就以它为参考。物理含义是
    /// 「一步最多把足朝向拉向实际步向几成」；平滑系数 1.0 = 原行为。敏感性见
    /// `doc/catwalk_design.md`。
    fn next_committed(&self, new_pos: Dot) -> Foot {
        let sw = self.swing_foot();
        let stride = sw.position.distance(new_pos);
        let new_rz = if stride >= self.config.active_distance {
            let step_dir = sw.position.angle(new_pos);
            let delta = crate::metrics::wrap_angle(step_dir - sw.rz);
            sw.rz + delta * HEADING_SMOOTHING
        } else {
            sw.rz
        };
        Foot::new(new_pos, new_rz, sw.shape)
    }

    /// 移动摆动脚到新位置，并切换当前活动的脚。
    pub fn walk_update(&mut self, new_pos: Dot) {
        let next_foot = self.next_committed(new_pos);
        *self.swing_foot_mut() = next_foot;
        self.change_feet();
    }

    /// 检查新位置是否满足步长约束（原 `satisfy_stride`，`src/robot/robot.cpp:228-230`）。
    ///
    /// 严格小于 `<`：`distance < max_stride`。
    pub fn satisfy_stride(&self, new_pos: Dot) -> bool {
        self.distance(new_pos) < self.config.max_stride
    }

    /// 检查新位置是否满足转角约束（原 `satisfy_turn`，`src/robot/robot.cpp:232-235`）。
    ///
    /// 严格开区间：`-max_turn < angle < max_turn`。
    ///
    /// 【有意修正】原 C++ 直接拿 `atan2` 差值去比 `max_turn`，**不做角度归一化**：
    /// `support.rz ≈ ±π` 时朝另一侧的候选方向会被 `atan2` 折回，差值变成 ±2π 量级的
    /// 伪大角（实测 raw −342.03°），于是本应合法的小转角候选被拒；候选集被清空后
    /// `find_target` 返回 [`CoreError::NoPath`]，整条规划以 `exit=2` 失败。
    /// 此处先用 [`crate::metrics::wrap_angle`] 把差值归一到 `(-π, π]` 再判开区间
    /// （语义仍是「严格开区间」，只是把差角换算到同一圈内）。
    ///
    /// 该项与冻结夹具 `tests/golden/satisfy_turn.csv` 的 case 16/24/26 三行有意分歧，
    /// 逐行手算与理由见 `doc/known_divergences.md` 与 `tests/robot_golden.rs`。
    pub fn satisfy_turn(&self, new_pos: Dot) -> bool {
        let angle = crate::metrics::wrap_angle(
            self.swing_foot().position.angle(new_pos) - self.support_foot().rz,
        );
        angle < self.config.max_turn && angle > -self.config.max_turn
    }

    /// 检查新位置是否满足足部间距约束（原 `satisfy_spacing`，`src/robot/robot.cpp:193-226`）。
    ///
    /// 按原实现语义：闭区间 + 浮点容差（`SEPARATION_EPS = 1e-3`）。
    /// `spacing = 摆动脚四角到支撑脚中心线的最小垂距 - 支撑脚半宽`，夹紧到 `>= 0`。
    pub fn satisfy_spacing(&self, new_pos: Dot) -> bool {
        let support_foot = self.support_foot();
        let new_foot = self.next(new_pos);
        let points = new_foot.corners();

        let half_width = support_foot.shape.half_width();
        let center_line = Line::from_point_angle(support_foot.position, support_foot.rz);

        let mut min_dist = f64::INFINITY;
        for pt in points {
            min_dist = min_dist.min(center_line.dist_to_point(pt));
        }

        let spacing = (min_dist - half_width).max(0.0);

        let epsilon = crate::config::SEPARATION_EPS;
        let min_allowed = self.config.min_separation - epsilon;
        let max_allowed = self.config.max_separation + epsilon;

        spacing >= min_allowed && spacing <= max_allowed
    }

    /// 综合约束检查：同时满足 spacing, stride, turn。
    pub fn position_check(&self, new_pos: Dot) -> bool {
        self.satisfy_spacing(new_pos) && self.satisfy_stride(new_pos) && self.satisfy_turn(new_pos)
    }

    /// 生成候选落足点集（原 `walk_candidates`，`src/robot/robot.cpp:661-689`）。
    pub fn walk_candidates(&self) -> Vec<Dot> {
        let swing_foot = self.swing_foot();
        let support_foot = self.support_foot();
        let resolution = 2.0;
        let angle_resolution = std::f64::consts::PI / 12.0;
        let mut candidates = Vec::new();

        let mut distance = self.config.active_distance;
        while distance <= self.config.max_stride + 1e-9 {
            let mut angle_offset = -self.config.max_turn;
            while angle_offset <= self.config.max_turn + 1e-9 {
                let angle = support_foot.rz + angle_offset;
                let new_pos = Dot::new(
                    swing_foot.position.x() + distance * angle.cos(),
                    swing_foot.position.y() + distance * angle.sin(),
                );
                if self.position_check(new_pos) {
                    candidates.push(new_pos);
                }
                angle_offset += angle_resolution;
            }
            distance += resolution;
        }
        candidates
    }

    /// 从候选集中选择离引导点最近的目标点（原 `find_target`，`src/robot/robot.cpp:321-331`）。
    ///
    /// 【修正有意偏离】：原实现在 candidates 为空时执行 `candidates[0]` 引发 SIGABRT(134)。
    /// Rust 侧显式返回 `Err(CoreError::NoPath)`，不 panic。
    pub fn find_target(&self, guide_point: Dot) -> Result<Dot> {
        let candidates = self.walk_candidates();
        if candidates.is_empty() {
            return Err(CoreError::NoPath {
                from: self.swing_foot().position,
                to: guide_point,
            });
        }

        let best = candidates
            .into_iter()
            .min_by(|a, b| {
                a.distance(guide_point)
                    .total_cmp(&b.distance(guide_point))
            })
            .expect("candidates is non-empty");
        Ok(best)
    }

    /// 原 `little_step`（`src/robot/robot.cpp:333-338`）。
    pub fn little_step(&self) -> Dot {
        let sw = self.swing_foot();
        let dist = self.config.max_stride * sw.rz.cos() * 0.5;
        let disn = self.config.max_stride * sw.rz.sin() * 0.5;
        Dot::new(sw.position.x() + dist, sw.position.y() + disn)
    }

    /// 原 `get_target`（`src/robot/robot.cpp:273-280`）。
    pub fn get_target(&self, guide_point: Dot) -> Result<Dot> {
        let result = self.find_target(guide_point)?;
        let sw = self.swing_foot();
        if result.distance(sw.position) < 0.3 * sw.about_r() {
            Ok(self.little_step())
        } else {
            Ok(result)
        }
    }

    fn rot_neighbour(dot: Dot, rot: f64, r: f64) -> [Dot; 4] {
        let dist = r * rot.cos();
        let disn = r * rot.sin();
        [
            Dot::new(dot.x() + dist, dot.y() + disn),
            Dot::new(dot.x() - dist, dot.y() - disn),
            Dot::new(dot.x() + disn, dot.y() - dist),
            Dot::new(dot.x() - disn, dot.y() + dist),
        ]
    }

    /// 原 `bfs`（`src/robot/robot.cpp:430-464`）。
    ///
    /// 【缺陷保留说明】：原实现在目标越界 (434) 或 tick 耗尽 (463) 时静默返回摆动脚当前位置。
    /// 此处保留该对外行为以对齐 baseline，同时在注释与测试中登记。
    pub fn bfs(&self, t: &Terrain, target: Dot) -> Dot {
        let sw = self.swing_foot();
        let rows = t.grid.rows() as i32;
        let cols = t.grid.cols() as i32;

        let is_valid = |p: Dot| {
            p.x() >= 0.0
                && (p.x() as i32) < rows
                && p.y() >= 0.0
                && (p.y() as i32) < cols
        };

        if !is_valid(target) {
            // [缺陷保留] 目标点越界静默返回当前摆动脚位置
            return sw.position;
        }

        let mut tick = 100;
        let mut pends = std::collections::VecDeque::new();
        let mut visited = std::collections::HashSet::new();

        pends.push_back(target);

        while let Some(now) = pends.pop_front() {
            tick -= 1;
            if tick <= 0 {
                break;
            }

            let key = (now.x().to_bits(), now.y().to_bits());
            if visited.contains(&key) {
                continue;
            }
            visited.insert(key);

            if !is_valid(now) {
                continue;
            }

            let new_foot = self.next(now);
            if !self.position_check(now) {
                continue;
            }

            if new_foot.standable(t) {
                return new_foot.position;
            }

            for point in Self::rot_neighbour(now, sw.rz, sw.about_r()) {
                if is_valid(point) {
                    pends.push_back(point);
                }
            }
        }

        // [缺陷保留] 遍历结束或 tick 耗尽未找到满足条件位置，静默返回摆动脚原位置
        sw.position
    }

    /// 原 `walk_to`（`src/robot/robot.cpp:267-271`）。
    pub fn walk_to(&self, t: &Terrain, guide_point: Dot) -> Result<Dot> {
        let target = self.get_target(guide_point)?;
        let next_pos = self.bfs(t, target);
        Ok(next_pos)
    }

    /// 判断机器人中心是否已到达目标点附近（原 `reach_target`，`src/robot/robot.cpp:466-468`）。
    pub fn reach_target(&self, target: Dot) -> bool {
        self.position().distance(target) < self.config.active_distance
    }

    /// 初始选择摆动脚（原 `select_swing_foot`，`src/robot/robot.cpp:615-627`）。
    pub fn select_swing_foot(&mut self, first_guide: Dot) {
        let left_dist = self.feet[0].position.distance(first_guide);
        let right_dist = self.feet[1].position.distance(first_guide);
        if left_dist <= right_dist {
            self.now_which_foot_to_move = WhichFoot::Left;
        } else {
            self.now_which_foot_to_move = WhichFoot::Right;
        }
    }

    /// 本脚通道上的瞄准点（路线 D：足印分道）。
    ///
    /// 把引导点沿**前进方向**（本体中心 → 引导点）的垂线偏移半个通道宽，左右脚各偏一侧：
    /// `feet[0]`(Left) 偏 −y 侧、`feet[1]`(Right) 偏 +y 侧，与 [`Robot::new`] 的初始布置一致。
    ///
    /// 参考系是**前进方向**，不是任何一只脚的朝向——T6 的负结果里 `guide_bias` 用的是
    /// 双足平均朝向（`central_angle`），朝向本身在抖，偏移量也跟着抖；这里换成引导方向后
    /// 偏移量只随路径转向变化。
    pub fn lane_aim(&self, guide: Dot) -> Dot {
        let center = self.position();
        let dx = guide.x() - center.x();
        let dy = guide.y() - center.y();
        let dist = (dx * dx + dy * dy).sqrt();
        if dist < 1e-9 {
            return guide;
        }
        let (ux, uy) = (dx / dist, dy / dist);
        let side = match self.now_which_foot_to_move {
            WhichFoot::Left => -1.0,
            WhichFoot::Right => 1.0,
        };
        let offset = side * LANE_HALF_WIDTH_CM;
        Dot::new(guide.x() - uy * offset, guide.y() + ux * offset)
    }

    /// 双脚轨迹是否会交叉（原 `will_feet_cross`，`src/robot/robot.cpp:578-591`）。
    pub fn will_feet_cross(&self, pends: &[Dot]) -> bool {
        if pends.len() < 2 {
            return false;
        }
        let sw = self.swing_foot();
        let sp = self.support_foot();
        let l1 = Line {
            p0: sw.position,
            p1: pends[0],
        };
        let l2 = Line {
            p0: sp.position,
            p1: pends[1],
        };
        l1.intersects(l2)
    }

    /// 避开双脚交叉（原 `avoid_feet_cross`，`src/robot/robot.cpp:593-608`）。
    pub fn avoid_feet_cross(&mut self, t: &Terrain, pends: &mut Vec<Dot>) -> Result<()> {
        if pends.len() < 2 {
            return Ok(());
        }
        let p1 = pends.pop().expect("len >= 2");
        let sp_target = self.walk_to(t, p1)?;
        self.walk_update(sp_target);

        let p0 = pends.pop().expect("len >= 1");
        let sw_target = self.walk_to(t, p0)?;
        self.walk_update(sw_target);
        Ok(())
    }

    /// 从当前位置规划到 `goal` 的落足点序列（原 `find_path`，`src/robot/robot.cpp:350-417`）。
    pub fn find_path(&mut self, t: &Terrain, goal: Dot, rng: &mut rng::Rng) -> Result<Vec<Dot>> {
        let start_idx = self.position().as_idx();
        let goal_idx = goal.as_idx();

        let search = crate::astar::scale_star(
            t,
            start_idx,
            goal_idx,
            1.0 / self.config.max_stride,
            rng,
        )
        .ok_or(CoreError::NoPath {
            from: self.position(),
            to: goal,
        })?;

        let mut guides_idx = search.path;
        guides_idx.reverse();
        let mut tick = 2 * guides_idx.len() + 127;

        let mut guides: Vec<Dot> = guides_idx.iter().map(|idx| idx.as_dot()).collect();
        let mut guide = crate::astar::direction_determine(&guides, true);
        self.select_swing_foot(guide);

        let mut path = Vec::new();
        let mut pend_dots: Vec<Dot> = Vec::with_capacity(2);
        let mut cursor = 0;

        while !guides.is_empty() && tick > 0 && !self.reach_target(goal) {
            while !guides.is_empty() && self.reach_target(guide) {
                guides.pop();
                if guides.is_empty() {
                    return Ok(path);
                }
                guide = crate::astar::direction_determine(&guides, true);
            }

            // 路线 D：瞄准点沿前进方向的垂线偏移到本脚通道，而不是直接瞄引导点。
            let aim = self.lane_aim(guide);
            let next_step = self.walk_to(t, aim)?;
            pend_dots.push(next_step);
            cursor = (cursor + 1) % 2;
            let waiting = cursor == 0;

            if waiting && pend_dots.len() >= 2 {
                if self.will_feet_cross(&pend_dots) {
                    self.avoid_feet_cross(t, &mut pend_dots)?;
                    path.push(self.swing_foot().position);
                    path.push(self.support_foot().position);
                } else {
                    // 书账修正：落足必须用「本次迭代为当前摆动脚生成的候选」`pend_dots[1]`。
                    // 原实现取 `pend_dots[0]`，那是上一次迭代为**另一只脚**生成的候选，
                    // 加上 walk_update 与循环末尾各翻一次脚，导致候选被落到错误的脚上
                    // （见 doc/catwalk_fix.md 的逐步归因与 doc/catwalk_diagnosis.md §②.1-7）。
                    let target = pend_dots[1];
                    self.walk_update(target);
                    path.push(target);
                    pend_dots.clear();
                }
            }

            self.change_feet();
            tick -= 1;
        }

        for dot in pend_dots {
            self.walk_update(dot);
            path.push(dot);
        }

        Ok(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{MAX_STRIDE_CM, MAX_TURN_RAD};

    #[test]
    fn test_satisfy_stride_strict_boundary() {
        let cfg = RobotConfig::default();
        let robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        // swing foot (Left) 初始位于 (0, -sep/2)
        let sw_pos = robot.swing_foot().position;

        // 恰好 40.0 应当失败（严格小于 <）
        let p_exact = Dot::new(sw_pos.x() + MAX_STRIDE_CM, sw_pos.y());
        assert!(!robot.satisfy_stride(p_exact));

        // 39.999999 应当成功
        let p_under = Dot::new(sw_pos.x() + MAX_STRIDE_CM - 1e-6, sw_pos.y());
        assert!(robot.satisfy_stride(p_under));

        // 40.000001 应当失败
        let p_over = Dot::new(sw_pos.x() + MAX_STRIDE_CM + 1e-6, sw_pos.y());
        assert!(!robot.satisfy_stride(p_over));
    }

    #[test]
    fn test_satisfy_turn_open_boundary() {
        let cfg = RobotConfig::default();
        let mut robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        // 设置支撑脚朝向为 0.0
        robot.support_foot_mut().rz = 0.0;
        let sw_pos = robot.swing_foot().position;

        // 恰好 +max_turn 应该失败（严格开区间）
        let angle_exact = MAX_TURN_RAD;
        let p_exact = Dot::new(
            sw_pos.x() + 10.0 * angle_exact.cos(),
            sw_pos.y() + 10.0 * angle_exact.sin(),
        );
        assert!(!robot.satisfy_turn(p_exact));

        // 略小于 max_turn 应该成功
        let angle_inside = MAX_TURN_RAD - 1e-4;
        let p_inside = Dot::new(
            sw_pos.x() + 10.0 * angle_inside.cos(),
            sw_pos.y() + 10.0 * angle_inside.sin(),
        );
        assert!(robot.satisfy_turn(p_inside));

        // 恰好 -max_turn 应该失败
        let angle_neg_exact = -MAX_TURN_RAD;
        let p_neg_exact = Dot::new(
            sw_pos.x() + 10.0 * angle_neg_exact.cos(),
            sw_pos.y() + 10.0 * angle_neg_exact.sin(),
        );
        assert!(!robot.satisfy_turn(p_neg_exact));
    }

    #[test]
    fn test_empty_candidates_returns_core_error_no_path() {
        // 构造候选必为空的场景：max_stride = 0.0 使得 distance < max_stride 恒假
        let cfg = RobotConfig {
            max_stride: 0.0,
            ..RobotConfig::default()
        };
        let robot = Robot::new(cfg, Dot::new(0.0, 0.0));
        let guide = Dot::new(20.0, 20.0);

        let res = robot.find_target(guide);
        match res {
            Err(CoreError::NoPath { from, to }) => {
                assert_eq!(from, robot.swing_foot().position);
                assert_eq!(to, guide);
            }
            Ok(pt) => panic!("空候选集理应返回 Err(NoPath)，实得 Ok({:?})", pt),
            Err(e) => panic!("期望 NoPath，实得其他错误变体: {:?}", e),
        }
    }
}
