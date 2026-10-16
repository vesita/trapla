//! trapla-io：CSV 读写。
//!
//! 错误类型**单独定义**（[`IoError`]），不把 IO / 格式错误塞进 `trapla-core` 的
//! [`trapla_core::error::CoreError`]；core 错误通过 `From<CoreError>` 折进来。

pub mod csv;

use std::fmt;

pub use trapla_core::error::CoreError;

/// IO 与格式错误分类。
#[derive(Debug)]
#[non_exhaustive]
pub enum IoError {
    /// 底层 IO 失败（路径不存在、无权限等）。
    Io(std::io::Error),
    /// 解析失败：`line` 从 1 开始计。
    Format { line: usize, detail: String },
    /// 矩阵形状与声明不符。
    Shape { expected: usize, got: usize },
    /// 来自 `trapla-core` 的错误（如 `Grid::new` 的形状/缩放校验）。
    Core(CoreError),
}

/// `trapla-io` 的 `Result` 别名。
pub type Result<T> = std::result::Result<T, IoError>;

impl fmt::Display for IoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "IO 错误: {e}"),
            Self::Format { line, detail } => write!(f, "第 {line} 行格式错误: {detail}"),
            Self::Shape { expected, got } => {
                write!(f, "矩阵形状不符: 期望 {expected} 个元素, 实际 {got} 个")
            }
            Self::Core(e) => write!(f, "核心错误: {e}"),
        }
    }
}

impl std::error::Error for IoError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Core(e) => Some(e),
            _ => None,
        }
    }
}

impl From<std::io::Error> for IoError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<CoreError> for IoError {
    fn from(e: CoreError) -> Self {
        Self::Core(e)
    }
}
