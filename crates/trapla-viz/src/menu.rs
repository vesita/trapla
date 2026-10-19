//! egui 主菜单（移植自 breeze `src/ui.rs` 的菜单骨架）。
//!
//! - 系统放 [`EguiPrimaryContextPass`]，不是 `Update`：bevy_egui 在这个 schedule 里跑
//!   `Context::run`，放 `Update` 会在首帧 panic（breeze 记过这个坑）。
//! - 不用 `CentralPanel`：它走 legacy panel 路径，与 bevy_egui 的 begin/end 帧模型不兼容；
//!   用锚定屏幕中心的 `Area`（浮动层）。
//! - 选项面全部来自 [`MenuOptions`] 资源，这里不硬编码场景清单。
//! - 菜单文案用 ASCII：bevy_egui 的默认字体不含 CJK，而本批次决定不搬字体资产
//!   （见 `trapla-viz/README.md`「沙盒模式」）。游玩期间的 HUD 仍用 Bevy 文本与系统中文字体。

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};

use crate::phase::GamePhase;
use crate::session::{MenuOptions, MenuSelection, SceneSource};

/// 主菜单插件：只在 `MainMenu` 阶段渲染菜单。
pub struct SandboxMenuPlugin;

impl Plugin for SandboxMenuPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            EguiPrimaryContextPass,
            main_menu.run_if(in_state(GamePhase::MainMenu)),
        );
    }
}

fn main_menu(
    mut egui_ctx: EguiContexts,
    mut options: ResMut<MenuOptions>,
    mut selection: ResMut<MenuSelection>,
    source: Res<SceneSource>,
    mut next_state: ResMut<NextState<GamePhase>>,
    mut exit: MessageWriter<AppExit>,
) {
    let Ok(ctx) = egui_ctx.ctx_mut() else {
        return;
    };

    // 在 UI 闭包里只读资源、写回选择；场景清单先拍成局部副本，避免同时对
    // `options.scenarios` 借用与 `options.selected` 可变借用。
    let entries: Vec<(String, String)> = options
        .scenarios
        .iter()
        .map(|entry| (entry.id.clone(), entry.expected.clone()))
        .collect();
    let mut start = false;

    egui::Area::new(egui::Id::new("trapla_sandbox_menu"))
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .show(ctx, |ui| {
            ui.vertical_centered(|ui| {
                ui.add_space(120.0);
                ui.heading("trapla sandbox");
                ui.label(format!("map {} x {}", options.size, options.size));
                ui.label(match &*source {
                    SceneSource::Files { map, .. } => format!("input: {}", map.display()),
                    SceneSource::Scenario { id } => format!("scenario: {id}"),
                });
                ui.separator();
                for (i, (id, expected)) in entries.iter().enumerate() {
                    ui.selectable_value(&mut options.selected, i, format!("{id}  [{expected}]"));
                }
                ui.separator();
                start = ui.button("Start").clicked();
                if ui.button("Quit").clicked() {
                    exit.write(AppExit::Success);
                }
            });
        });

    let keyboard_start = ctx.input(|i| i.key_pressed(egui::Key::Enter));
    if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
        exit.write(AppExit::Success);
    }

    if !(start || keyboard_start) {
        return;
    }
    match &*source {
        SceneSource::Files { .. } => next_state.set(GamePhase::Playing),
        SceneSource::Scenario { .. } => {
            if let Some(id) = options.selected_id() {
                selection.0 = Some(id.to_string());
                next_state.set(GamePhase::Playing);
            }
        }
    }
}
