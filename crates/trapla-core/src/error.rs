//! 核心错误分类。
//!
//! 分类（本模块）与边界（CLI 里映射退出码）分开：本模块只回答「哪一种失败」，
//! 不带 source、不带位置。IO 错误不进这里，见 `trapla-io` 的 `IoError`。

use std::fmt;

use crate::geom::Dot;

/// trapla-core 的错误分类。
#[derive(Debug)]
#[non_exhaustive]
pub enum CoreError {
    /// 坐标越出栅格边界。
    OutOfBounds { x: f64, y: f64 },
    /// 坐标落在障碍格上。
    Obstacle { x: f64, y: f64 },
    /// 平面退化：可用点不足 3 个，或三点共线 / 无法定面。
    DegeneratePlane,
    /// 搜索不到可行路径。
    NoPath { from: Dot, to: Dot },
    /// 缩放比例非法（<= 0 或非有限）。
    InvalidScale(f64),
    /// 栅格形状与声明不符：`cells.len() != rows * cols`。
    ///
    /// 由 `Grid::new` 在该条件成立时返回（接线是 T2 的活，见 `grid.rs`）。
    GridShapeMismatch {
        rows: usize,
        cols: usize,
        got: usize,
    },
}

/// 核心库的 `Result` 别名。
pub type Result<T> = std::result::Result<T, CoreError>;

impl fmt::Display for CoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutOfBounds { x, y } => write!(f, "坐标越界: ({x}, {y})"),
            Self::Obstacle { x, y } => write!(f, "命中障碍物: ({x}, {y})"),
            Self::DegeneratePlane => write!(f, "退化平面：可用点不足 3 个或三点共线"),
            Self::NoPath { from, to } => write!(
                f,
                "无可行路径: ({}, {}) -> ({}, {})",
                from.x(),
                from.y(),
                to.x(),
                to.y()
            ),
            Self::InvalidScale(s) => write!(f, "非法缩放比例: {s}"),
            Self::GridShapeMismatch { rows, cols, got } => write!(
                f,
                "栅格形状不匹配: {rows}x{cols} = {} 个元素, 实际 {got} 个",
                rows.saturating_mul(*cols)
            ),
        }
    }
}

impl std::error::Error for CoreError {}
