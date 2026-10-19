//! 游戏流程状态机（移植自 breeze `src/ui.rs` 的 `GamePhase`）。
//!
//! 两个阶段互斥，用 `States` 表达而不是若干 `bool`：菜单帧与游玩帧不可能同时成立。
//! 玩法系统用 `run_if(in_state(GamePhase::Playing))` 门控，菜单时不跑回放 / 相机 / HUD。

use bevy::prelude::*;

/// 游戏流程：主菜单 → 游玩（可退回主菜单）。
#[derive(States, Debug, Clone, PartialEq, Eq, Hash, Default)]
pub enum GamePhase {
    /// 主菜单：只画 egui 菜单，不加载场景、不跑回放。
    #[default]
    MainMenu,
    /// 游玩：场景已加载，回放 / 相机 / HUD 全部运行。
    Playing,
}
