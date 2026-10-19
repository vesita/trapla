//! trapla-lab：**无头**算法测试台（不依赖 Bevy）。
//!
//! 四层，各管一件事：
//!
//! - [`scenario`]（场景层）：确定性程序化地形工厂 + 场景 manifest（JSON）。
//! - [`planner`]（执行层）：[`planner::Planner`] 把候选算法与跑分框架解耦；内置参考实现
//!   [`planner::RobotPlanner`] 与对照坏实现 [`planner::NaiveStraightPlanner`]。
//! - [`validate`]（校验层）：**独立**实现的硬约束校验器（步长 / 转角 / 间距 / 足覆盖 / 法向角），
//!   不调用 `Robot::position_check` 自证。
//! - [`bench`] + [`report`]（报告层）：批量跑分、基线对比、回归门禁与人读表。
//!
//! [`runstate`] 把五阶段（场景 → 执行 → 校验 → 汇总 → 门禁）钉成只能按序前进的状态机。

pub mod bench;
pub mod cli;
pub mod planner;
pub mod report;
pub mod runstate;
pub mod scenario;
pub mod sha256;
pub mod validate;
