//! 猫步指标：路径的量化载体（CLI `analyze` 与验收脚本消费）。
//!
//! 与 C++ 基线工具同源的冻结定义。

use std::f64::consts::PI;

use crate::geom::Dot;

/// 一条路径的指标。
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct PathMetrics {
    /// 步数（落足点次数，= n）。
    pub steps: usize,
    /// 路径总长：Σ|p[i+1]-p[i]|, i=0..n-2。
    pub total_length: f64,
    /// 终点与 goal 的误差：|p[n-1] - goal|。
    pub goal_error: f64,
    /// 相邻落足点的跨距序列（len n-1: |p[i+1]-p[i]|）。
    pub stride: Vec<f64>,
    /// 跨足相邻步带符号转角（len n-2）。
    pub turn_angle: Vec<f64>,
    /// 足中心距代理（len n-1: |p[i]-p[i-1]|）。
    pub foot_center_separation: Vec<f64>,
    /// 侧向间距序列（len n-2）。
    pub lateral_separation: Vec<f64>,
    /// 纵向间距序列（len n-2）。
    pub along_separation: Vec<f64>,
    /// 谓词交叉数（含共线/端点接触）。
    pub predicate_crossings: usize,
    /// 真交叉数（严格跨立）。
    pub proper_crossings: usize,
    /// 同足相邻步步向角差 Δψ（带符号，归一到 (-π, π]）。
    pub same_foot_turn: Vec<f64>,
}

/// 角度归一化到 (-π, π]。
pub fn wrap_angle(mut rad: f64) -> f64 {
    rad %= 2.0 * PI;
    if rad <= -PI {
        rad += 2.0 * PI;
    } else if rad > PI {
        rad -= 2.0 * PI;
    }
    if rad == -PI {
        rad = PI;
    }
    rad
}

/// 从点位序列算指标。
pub fn compute(path: &[Dot], goal: Dot) -> PathMetrics {
    let n = path.len();
    if n == 0 {
        return PathMetrics {
            goal_error: f64::NAN,
            ..Default::default()
        };
    }

    let goal_error = path[n - 1].distance(goal);

    if n == 1 {
        return PathMetrics {
            steps: 1,
            total_length: 0.0,
            goal_error,
            stride: Vec::new(),
            turn_angle: Vec::new(),
            foot_center_separation: Vec::new(),
            lateral_separation: Vec::new(),
            along_separation: Vec::new(),
            predicate_crossings: 0,
            proper_crossings: 0,
            same_foot_turn: Vec::new(),
        };
    }

    let mut stride = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        stride.push(path[i].distance(path[i + 1]));
    }
    let foot_center_separation = stride.clone();
    let total_length: f64 = stride.iter().sum();

    let mut turn_angle = Vec::with_capacity(n.saturating_sub(2));
    if n >= 3 {
        for i in 0..n - 2 {
            let v1_x = path[i + 1].x() - path[i].x();
            let v1_y = path[i + 1].y() - path[i].y();
            let v2_x = path[i + 2].x() - path[i + 1].x();
            let v2_y = path[i + 2].y() - path[i + 1].y();
            let cross = v1_x * v2_y - v1_y * v2_x;
            let dot = v1_x * v2_x + v1_y * v2_y;
            turn_angle.push(cross.atan2(dot));
        }
    }

    let mut lateral_separation = Vec::with_capacity(n.saturating_sub(2));
    let mut along_separation = Vec::with_capacity(n.saturating_sub(2));
    if n >= 3 {
        for i in 1..n - 1 {
            let u_x = path[i + 1].x() - path[i - 1].x();
            let u_y = path[i + 1].y() - path[i - 1].y();
            let v_x = path[i].x() - path[i - 1].x();
            let v_y = path[i].y() - path[i - 1].y();
            let u_len = (u_x * u_x + u_y * u_y).sqrt();
            if u_len == 0.0 {
                lateral_separation.push(0.0);
                along_separation.push(0.0);
            } else {
                let cross = (u_x * v_y - u_y * v_x).abs();
                let dot = u_x * v_x + u_y * v_y;
                lateral_separation.push(cross / u_len);
                along_separation.push(dot / u_len);
            }
        }
    }

    // 交叉数判定
    let mut seg_a = Vec::new();
    let mut seg_b = Vec::new();

    // 偶数下标折线 A
    let even_pts: Vec<Dot> = path.iter().step_by(2).copied().collect();
    if even_pts.len() >= 2 {
        for i in 0..even_pts.len() - 1 {
            seg_a.push((even_pts[i], even_pts[i + 1]));
        }
    }

    // 奇数下标折线 B
    if path.len() >= 2 {
        let odd_pts: Vec<Dot> = path[1..].iter().step_by(2).copied().collect();
        if odd_pts.len() >= 2 {
            for i in 0..odd_pts.len() - 1 {
                seg_b.push((odd_pts[i], odd_pts[i + 1]));
            }
        }
    }

    let mut predicate_crossings = 0;
    let mut proper_crossings = 0;

    for (p1, p2) in &seg_a {
        for (p3, p4) in &seg_b {
            // 包围盒快速排斥
            if p1.x().min(p2.x()) > p3.x().max(p4.x())
                || p3.x().min(p4.x()) > p1.x().max(p2.x())
                || p1.y().min(p2.y()) > p3.y().max(p4.y())
                || p3.y().min(p4.y()) > p1.y().max(p2.y())
            {
                continue;
            }

            let c1 = (p2.x() - p1.x()) * (p3.y() - p1.y()) - (p3.x() - p1.x()) * (p2.y() - p1.y());
            let c2 = (p2.x() - p1.x()) * (p4.y() - p1.y()) - (p4.x() - p1.x()) * (p2.y() - p1.y());
            let c3 = (p4.x() - p3.x()) * (p1.y() - p3.y()) - (p1.x() - p3.x()) * (p4.y() - p3.y());
            let c4 = (p4.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p4.y() - p3.y());

            if c1 * c2 <= 0.0 && c3 * c4 <= 0.0 {
                predicate_crossings += 1;
                if c1 * c2 < 0.0 && c3 * c4 < 0.0 {
                    proper_crossings += 1;
                }
            }
        }
    }

    // same_foot_turn
    let mut same_foot_turn = Vec::new();
    let compute_turns = |pts: &[Dot], out: &mut Vec<f64>| {
        if pts.len() >= 3 {
            let mut psis = Vec::with_capacity(pts.len() - 1);
            for k in 0..pts.len() - 1 {
                let dx = pts[k + 1].x() - pts[k].x();
                let dy = pts[k + 1].y() - pts[k].y();
                psis.push(dy.atan2(dx));
            }
            for k in 1..psis.len() {
                out.push(wrap_angle(psis[k] - psis[k - 1]));
            }
        }
    };

    compute_turns(&even_pts, &mut same_foot_turn);
    if path.len() >= 2 {
        let odd_pts: Vec<Dot> = path[1..].iter().step_by(2).copied().collect();
        compute_turns(&odd_pts, &mut same_foot_turn);
    }

    PathMetrics {
        steps: n,
        total_length,
        goal_error,
        stride,
        turn_angle,
        foot_center_separation,
        lateral_separation,
        along_separation,
        predicate_crossings,
        proper_crossings,
        same_foot_turn,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_metrics_lengths_n0_to_n5() {
        let goal = Dot::new(100.0, 100.0);

        // n = 0
        let m0 = compute(&[], goal);
        assert_eq!(m0.steps, 0);
        assert!(m0.goal_error.is_nan());
        assert_eq!(m0.stride.len(), 0);
        assert_eq!(m0.turn_angle.len(), 0);
        assert_eq!(m0.foot_center_separation.len(), 0);
        assert_eq!(m0.lateral_separation.len(), 0);
        assert_eq!(m0.along_separation.len(), 0);
        assert_eq!(m0.same_foot_turn.len(), 0);

        // n = 1
        let p1 = [Dot::new(0.0, 0.0)];
        let m1 = compute(&p1, goal);
        assert_eq!(m1.steps, 1);
        assert_relative_eq!(m1.goal_error, (100.0_f64.powi(2) + 100.0_f64.powi(2)).sqrt());
        assert_eq!(m1.stride.len(), 0);
        assert_eq!(m1.turn_angle.len(), 0);
        assert_eq!(m1.foot_center_separation.len(), 0);
        assert_eq!(m1.lateral_separation.len(), 0);
        assert_eq!(m1.along_separation.len(), 0);
        assert_eq!(m1.same_foot_turn.len(), 0);

        // n = 2
        let p2 = [Dot::new(0.0, 0.0), Dot::new(10.0, 0.0)];
        let m2 = compute(&p2, goal);
        assert_eq!(m2.steps, 2);
        assert_eq!(m2.stride.len(), 1);
        assert_eq!(m2.foot_center_separation.len(), 1);
        assert_eq!(m2.turn_angle.len(), 0);
        assert_eq!(m2.lateral_separation.len(), 0);
        assert_eq!(m2.along_separation.len(), 0);
        assert_eq!(m2.same_foot_turn.len(), 0);

        // n = 3
        let p3 = [
            Dot::new(0.0, 0.0),
            Dot::new(10.0, 0.0),
            Dot::new(20.0, 0.0),
        ];
        let m3 = compute(&p3, goal);
        assert_eq!(m3.steps, 3);
        assert_eq!(m3.stride.len(), 2);
        assert_eq!(m3.foot_center_separation.len(), 2);
        assert_eq!(m3.turn_angle.len(), 1);
        assert_eq!(m3.lateral_separation.len(), 1);
        assert_eq!(m3.along_separation.len(), 1);
        assert_eq!(m3.same_foot_turn.len(), 0);

        // n = 5
        let p5 = [
            Dot::new(0.0, 0.0),   // L0
            Dot::new(0.0, 5.0),   // R0
            Dot::new(10.0, 0.0),  // L1
            Dot::new(10.0, 5.0),  // R1
            Dot::new(10.0, -10.0), // L2 (L0->L1是(10,0)方向0, L1->L2是(0,-10)方向-pi/2, 差值-pi/2)
        ];
        let m5 = compute(&p5, goal);
        assert_eq!(m5.steps, 5);
        assert_eq!(m5.stride.len(), 4);
        assert_eq!(m5.foot_center_separation.len(), 4);
        assert_eq!(m5.turn_angle.len(), 3);
        assert_eq!(m5.lateral_separation.len(), 3);
        assert_eq!(m5.along_separation.len(), 3);
        assert_eq!(m5.same_foot_turn.len(), 1);
        assert_relative_eq!(m5.same_foot_turn[0], -PI / 2.0);
    }

    #[test]
    fn test_crossings_proper_vs_touching() {
        let goal = Dot::new(100.0, 100.0);

        // 例子 1：真交叉 (proper crossing)
        // 偶数点：L0 = (0, 0), L1 = (10, 10) -> 线段 (0,0) 到 (10,10)
        // 奇数点：R0 = (0, 10), R1 = (10, 0) -> 线段 (0,10) 到 (10,0)
        // 路径交替：[L0, R0, L1, R1]
        let path_proper = [
            Dot::new(0.0, 0.0),
            Dot::new(0.0, 10.0),
            Dot::new(10.0, 10.0),
            Dot::new(10.0, 0.0),
        ];
        let m_proper = compute(&path_proper, goal);
        assert_eq!(m_proper.predicate_crossings, 1);
        assert_eq!(m_proper.proper_crossings, 1);

        // 例子 2：只接触不跨立（T 型相触）
        // 偶数点：L0 = (0, 0), L1 = (10, 0)
        // 奇数点：R0 = (5, 0), R1 = (5, 10) (R0 恰在 L0-L1 上)
        let path_touch = [
            Dot::new(0.0, 0.0),
            Dot::new(5.0, 0.0),
            Dot::new(10.0, 0.0),
            Dot::new(5.0, 10.0),
        ];
        let m_touch = compute(&path_touch, goal);
        assert_eq!(m_touch.predicate_crossings, 1);
        assert_eq!(m_touch.proper_crossings, 0);
    }

    #[test]
    fn test_same_foot_turn_wrap() {
        let goal = Dot::new(0.0, 0.0);
        // 构造让 Δψ 超过 (-PI, PI] 的场景：
        // L0 = (0, 0)
        // L1 = L0 + (cos(170°), sin(170°)) -> 步向量 1 角度 170° (2.967 rad)
        // L2 = L1 + (cos(-170°), sin(-170°)) -> 步向量 2 角度 -170° (-2.967 rad)
        // 差值 = -170° - 170° = -340°。未 wrap 时是 -340° (-5.934 rad)
        // wrap 之后是 +20° (0.349 rad)
        let a1 = 170.0_f64.to_radians();
        let a2 = -170.0_f64.to_radians();
        let l0 = Dot::new(0.0, 0.0);
        let l1 = Dot::new(l0.x() + a1.cos(), l0.y() + a1.sin());
        let l2 = Dot::new(l1.x() + a2.cos(), l1.y() + a2.sin());

        let path = [
            l0,
            Dot::new(0.0, 1.0), // R0
            l1,
            Dot::new(1.0, 1.0), // R1
            l2,
        ];
        let m = compute(&path, goal);
        assert_eq!(m.same_foot_turn.len(), 1);
        let expected_wrap = 20.0_f64.to_radians();
        assert_relative_eq!(m.same_foot_turn[0], expected_wrap, epsilon = 1e-9);
    }
}
