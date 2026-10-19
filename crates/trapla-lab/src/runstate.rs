//! 运行阶段状态机：把「场景 → 执行 → 校验 → 汇总 → 门禁」钉成一条**只能按序前进**的流程。
//!
//! 分层（场景层 / 执行层 / 校验层 / 报告层）靠这条状态机强制：跳步、重复进入、乱序进入都返回
//! 错误而不是继续跑 —— 否则一份「跑过了」的报告可能根本没经过校验阶段。

use std::time::Instant;

use serde::{Deserialize, Serialize};

/// 运行阶段。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Stage {
    /// 生成 / 载入场景。
    Scenario,
    /// 交给各规划器执行。
    Execute,
    /// 独立校验。
    Validate,
    /// 汇总。
    Aggregate,
    /// 回归门禁。
    Gate,
}

impl Stage {
    /// 唯一合法的先后顺序。
    pub const ORDER: [Stage; 5] = [
        Stage::Scenario,
        Stage::Execute,
        Stage::Validate,
        Stage::Aggregate,
        Stage::Gate,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Scenario => "scenario",
            Self::Execute => "execute",
            Self::Validate => "validate",
            Self::Aggregate => "aggregate",
            Self::Gate => "gate",
        }
    }
}

/// 状态机错误。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StageError {
    /// 进入/结束的阶段不是下一个该走的阶段。
    OutOfOrder { expected: Stage, got: Stage },
    /// 上一阶段还没结束。
    StillOpen { open: Stage, got: Stage },
    /// 结束一个没有开始的阶段。
    NotStarted { got: Stage },
}

impl std::fmt::Display for StageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OutOfOrder { expected, got } => write!(
                f,
                "阶段乱序：应先进入 `{}`，实际进入 `{}`",
                expected.name(),
                got.name()
            ),
            Self::StillOpen { open, got } => write!(
                f,
                "阶段 `{}` 还没结束，不能进入 `{}`",
                open.name(),
                got.name()
            ),
            Self::NotStarted { got } => {
                write!(f, "阶段 `{}` 没有开始", got.name())
            }
        }
    }
}

impl std::error::Error for StageError {}

/// 一个阶段的耗时记录。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StageEntry {
    pub stage: Stage,
    pub elapsed_ms: f64,
}

/// 阶段游标 + 耗时账。
#[derive(Debug, Clone)]
pub struct StageMachine {
    cursor: usize,
    open: Option<(Stage, Instant)>,
    log: Vec<StageEntry>,
}

impl Default for StageMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl StageMachine {
    pub fn new() -> Self {
        Self {
            cursor: 0,
            open: None,
            log: Vec::new(),
        }
    }

    /// 进入下一个阶段；必须是 [`Stage::ORDER`] 里的下一个。
    pub fn begin(&mut self, stage: Stage) -> Result<(), StageError> {
        if let Some((open, _)) = self.open {
            return Err(StageError::StillOpen { open, got: stage });
        }
        let expected = Stage::ORDER.get(self.cursor).copied();
        match expected {
            Some(expected) if expected == stage => {
                self.open = Some((stage, Instant::now()));
                Ok(())
            }
            Some(expected) => Err(StageError::OutOfOrder {
                expected,
                got: stage,
            }),
            None => Err(StageError::OutOfOrder {
                expected: Stage::Gate,
                got: stage,
            }),
        }
    }

    /// 结束当前阶段并记账。
    pub fn end(&mut self, stage: Stage) -> Result<f64, StageError> {
        match self.open.take() {
            Some((open, started)) if open == stage => {
                let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
                self.log.push(StageEntry { stage, elapsed_ms });
                self.cursor += 1;
                Ok(elapsed_ms)
            }
            _ => Err(StageError::NotStarted { got: stage }),
        }
    }

    pub fn log(&self) -> &[StageEntry] {
        &self.log
    }

    /// 阶段全部走完。
    pub fn finished(&self) -> bool {
        self.cursor >= Stage::ORDER.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn in_order_traversal_finishes() {
        let mut machine = StageMachine::new();
        for stage in Stage::ORDER {
            machine.begin(stage).expect("按序进入");
            machine.end(stage).expect("按序结束");
        }
        assert!(machine.finished());
        assert_eq!(machine.log().len(), Stage::ORDER.len());
    }

    #[test]
    fn out_of_order_is_rejected() {
        let mut machine = StageMachine::new();
        let error = machine.begin(Stage::Validate).expect_err("跳步必须被拒");
        assert_eq!(
            error,
            StageError::OutOfOrder {
                expected: Stage::Scenario,
                got: Stage::Validate,
            }
        );

        machine.begin(Stage::Scenario).expect("第一步");
        let error = machine.begin(Stage::Execute).expect_err("上一阶段还开着");
        assert_eq!(
            error,
            StageError::StillOpen {
                open: Stage::Scenario,
                got: Stage::Execute,
            }
        );
    }

    #[test]
    fn end_without_begin_is_rejected() {
        let mut machine = StageMachine::new();
        assert_eq!(
            machine.end(Stage::Scenario).expect_err("没开始就结束"),
            StageError::NotStarted {
                got: Stage::Scenario
            }
        );
    }
}
