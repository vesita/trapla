//! 阈值的**唯一事实源**。
//!
//! 原 C++（HEAD `b53543b`）把同一批阈值散在 `include/robot/robot.hpp:78-79` 的默认参数、
//! `src/robot/foot.cpp:252` 的硬编码 `20.0/180.0*M_PI`，以及 `tests/constraints_test.cpp`
//! 的硬编码期望值里。Rust 重写只允许在本模块出现这些数值字面量；其余模块一律引用常量，
//! 不许再抄一份（验收会 `grep -rn` 数值字面量）。

/// 单步最大跨距（cm）。原 `Robot::max_stride` 默认 40.0。
pub const MAX_STRIDE_CM: f64 = 40.0;

/// 两足最小间距（cm）。原 `min_foot_separation` 默认 2.0。
pub const MIN_SEPARATION_CM: f64 = 2.0;

/// 两足最大间距（cm）。原 `max_foot_separation` 默认 10.0。
pub const MAX_SEPARATION_CM: f64 = 10.0;

/// 单步最大转角（弧度）。原 `max_turn = M_PI * 75.0 / 180.0`。
pub const MAX_TURN_RAD: f64 = std::f64::consts::PI * 75.0 / 180.0;

/// 可站立判据的平面法向角上限（弧度）。原 `foot.cpp:252` 硬编码 20°。
pub const MAX_NORMAL_ANGLE_RAD: f64 = std::f64::consts::PI * 20.0 / 180.0;

/// 间距比较用的浮点容差。
// CONTRACT-UNCERTAIN: 原 C++ 没有这一项（它直接用浮点比较），1e-3 是重写新引入的约定；
// 语义 = 「间距 >= MIN_SEPARATION_CM - EPS 且 <= MAX_SEPARATION_CM + EPS 视为满足」。T3/T4 若不采用请提出。
pub const SEPARATION_EPS: f64 = 1e-3;

/// `active_distance = max_stride * 该比例`。
// CONTRACT-UNCERTAIN: 原 `robot.cpp:276` 出现过 `0.3 * about_R()` 之类的近邻判据，
// 与本常量是否同一件事未确认；0.4 取自 T1 冻结契约，T3/T4 实现时请对照原实现核对。
pub const ACTIVE_DISTANCE_RATIO: f64 = 0.4;

/// 足形几何（单位：格/cm，与原实现一致）。
///
/// 方向约定（**消灭原 `geometry.cpp:116/238` 两个同名 `scale()` 的根源**）：
/// - `length` 沿足的朝向 `rz`；
/// - `width` 垂直 `rz`（原 `Robot::satisfy_spacing` 用的就是 `width/2`，见 `robot.cpp:201`）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FootShape {
    pub length: f64,
    pub width: f64,
    pub scan_step: f64,
}

impl FootShape {
    /// 默认双足足形：长 5.0、宽 3.0、扫描步长 0.5。
    pub const fn biped() -> Self {
        Self {
            length: 5.0,
            width: 3.0,
            scan_step: 0.5,
        }
    }

    /// 半长（沿 `rz`）。
    // 原实现依据：`src/robot/foot.cpp:198,222` 计算覆盖区和四角时使用 `shape.length / 2.0`。
    pub fn half_length(&self) -> f64 {
        self.length / 2.0
    }

    /// 半宽（垂直 `rz`）。
    // 原实现依据：`src/robot/robot.cpp:201` 的 `satisfy_spacing` 计算支撑脚半宽使用 `support_foot.shape.width / 2.0`；
    // `src/robot/foot.cpp:199,223` 计算覆盖区和四角时同样使用 `shape.width / 2.0`。
    pub fn half_width(&self) -> f64 {
        self.width / 2.0
    }
}

/// 规划器的全部可调参数。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotConfig {
    pub shape: FootShape,
    pub max_stride: f64,
    pub min_separation: f64,
    pub max_separation: f64,
    pub max_turn: f64,
    pub max_normal_angle: f64,
    pub active_distance: f64,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            shape: FootShape::biped(),
            max_stride: MAX_STRIDE_CM,
            min_separation: MIN_SEPARATION_CM,
            max_separation: MAX_SEPARATION_CM,
            max_turn: MAX_TURN_RAD,
            max_normal_angle: MAX_NORMAL_ANGLE_RAD,
            active_distance: MAX_STRIDE_CM * ACTIVE_DISTANCE_RATIO,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_defaults_and_shape() {
        let shape = FootShape::biped();
        assert_eq!(shape.half_length(), 2.5);
        assert_eq!(shape.half_width(), 1.5);

        let cfg = RobotConfig::default();
        assert_eq!(cfg.active_distance, 16.0);
    }
}
