//! Bevy 应用：地形 + 落足点回放 + 指标面板。
//!
//! 无头自检不进入本模块（它连 `App` 都不建），但两边共用 [`crate::scene`] 的几何函数，
//! 所以「自检打印的顶点数」就是渲染时真正使用的网格形状。
//!
//! 截图模式必须**等渲染管线编译完成**再截图：Bevy 默认异步编译管线，
//! 若在第 N 帧就截，前若干帧可能只有清屏色而没有网格。这里打开
//! `RenderPlugin::synchronous_pipeline_compilation`（Bevy 给确定性测试用的开关）。

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::render::RenderPlugin;
use bevy::text::{FontSize, FontSource};
use bevy::window::WindowResolution;

use crate::cli::Mode;
use crate::data::{Foot, FootPath, Terrain};
use crate::metrics::MetricsPanel;
use crate::orbit_camera::{
    LookTransform, OrbitCamera, OrbitCameraBundle, OrbitCameraPlugin, Smoother,
};
use crate::scene::{build_terrain_mesh, path_world_points, sample_shape, world_from_map};

/// 已加载好的场景输入（渲染与自检共用）。
pub struct SceneInputs {
    /// 原始高度场。
    pub terrain: Terrain,
    /// 足中心点序列。
    pub path: FootPath,
    /// 指标面板数据。
    pub metrics: MetricsPanel,
    /// 降采样步长。
    pub stride: usize,
    /// 目标点（地图坐标）。
    pub goal: (f64, f64),
}

/// 场景资源。
#[derive(Resource)]
struct Scene {
    terrain: Terrain,
    points: Vec<Vec3>,
    goal: Vec3,
    metrics: MetricsPanel,
    vertex_count: usize,
    triangle_count: usize,
    stride: usize,
    rows: usize,
    cols: usize,
    raw_cells: usize,
    /// 相机初始位姿 `(yaw, pitch, dist, target)`。
    home: (f32, f32, f32, Vec3),
}

/// 回放状态。
#[derive(Resource)]
struct Playback {
    /// 已揭示的点数（`0..=points.len()`）。
    revealed: usize,
    playing: bool,
    speed: f32,
    acc: f32,
}

impl Playback {
    /// 速度档位夹取：`0.25x ..= 8x`。
    fn set_speed(&mut self, v: f32) {
        self.speed = v.clamp(0.25, 8.0);
    }
}

/// 截图流程状态机。
#[derive(Resource)]
struct Shot {
    path: PathBuf,
    after_frames: u32,
    frame: u32,
    requested: bool,
    requested_at: u32,
    settle: u32,
}

/// 回放状态文本标记（每帧刷新）。
#[derive(Component)]
struct StatusText;

/// 1x 档位的揭示速度（点/秒）。
const POINTS_PER_SEC: f32 = 4.0;

/// 脚点球半径（世界单位）。
const FOOT_RADIUS: f32 = 11.0;

/// 截图落盘后额外等待的帧数（确认写入完成）。
const SHOT_SETTLE_FRAMES: u32 = 12;

/// 截图等待上限（帧）。
const SHOT_TIMEOUT_FRAMES: u32 = 900;

/// 中文字体族名（由系统字体发现解析；缺失时字体栈回退到通用 sans-serif）。
const CJK_FONT_FAMILY: &str = "Noto Sans CJK SC";

/// 启动 Bevy 应用（`--screenshot` 或交互模式）。
///
/// `frames` 只在截图模式使用。返回后调用方仍需自行核对 PNG 是否落盘。
pub fn run(inputs: SceneInputs, mode: &Mode, frames: u32) -> Result<(), String> {
    let points = path_world_points(&inputs.terrain, &inputs.path);
    let goal = world_from_map(&inputs.terrain, inputs.goal.0, inputs.goal.1) + Vec3::Y * 6.0;
    let stride = inputs.stride.max(1);
    let (rows, cols) = (inputs.terrain.rows(), inputs.terrain.cols());
    let (n_rows, n_cols) = sample_shape(rows, cols, stride);

    let centroid = centroid(&points);
    let extent = points
        .iter()
        .map(|p| p.distance(centroid))
        .fold(0.0_f32, f32::max)
        .max(120.0);
    let dist = (extent * 2.6).clamp(200.0, 12_000.0);
    let home = (0.55_f32, 0.62_f32, dist, centroid);

    let scene = Scene {
        terrain: inputs.terrain,
        points,
        goal,
        metrics: inputs.metrics,
        vertex_count: n_rows * n_cols,
        triangle_count: n_rows.saturating_sub(1) * n_cols.saturating_sub(1) * 2,
        stride,
        rows,
        cols,
        raw_cells: rows * cols,
        home,
    };

    // 交互模式用异步编译（启动快）；截图模式必须同步，否则前若干帧还没有网格。
    let sync_pipelines = matches!(mode, Mode::Screenshot(_));

    let (shot, title) = match mode {
        Mode::Screenshot(path) => (
            Some(Shot {
                path: path.clone(),
                after_frames: frames.max(1),
                frame: 0,
                requested: false,
                requested_at: 0,
                settle: 0,
            }),
            "trapla-viz 截图模式",
        ),
        _ => (None, "trapla-viz 交互模式（Esc 退出）"),
    };

    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .build()
            .disable::<bevy::audio::AudioPlugin>()
            .set(RenderPlugin {
                synchronous_pipeline_compilation: sync_pipelines,
                ..default()
            })
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: title.to_string(),
                    resolution: WindowResolution::new(1280, 720),
                    ..default()
                }),
                ..default()
            }),
    )
    .add_plugins(OrbitCameraPlugin)
    .insert_resource(ClearColor(Color::srgb(0.09, 0.11, 0.16)))
    .insert_resource(scene)
    .insert_resource(Playback {
        revealed: 0,
        playing: true,
        speed: 1.0,
        acc: 0.0,
    })
    .add_systems(Startup, (setup_scene, setup_ui))
    .add_systems(
        Update,
        (hotkeys, playback_tick, draw_scene, update_status).chain(),
    );

    if let Some(shot) = shot {
        app.insert_resource(shot).add_systems(Update, shot_system);
    }

    app.run();
    Ok(())
}

// ---- 截图流程 --------------------------------------------------------------

fn shot_system(mut commands: Commands, mut shot: ResMut<Shot>, mut exit: MessageWriter<AppExit>) {
    shot.frame += 1;
    if !shot.requested && shot.frame >= shot.after_frames {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(shot.path.clone()));
        shot.requested = true;
        shot.requested_at = shot.frame;
        info!("已请求截图 -> {}", shot.path.display());
    }
    if !shot.requested {
        return;
    }
    if shot.path.exists() {
        shot.settle += 1;
        if shot.settle >= SHOT_SETTLE_FRAMES {
            info!("截图完成: {}", shot.path.display());
            exit.write(AppExit::Success);
        }
    } else if shot.frame > shot.requested_at + SHOT_TIMEOUT_FRAMES {
        error!("截图超时：{} 未生成", shot.path.display());
        exit.write(AppExit::error());
    }
}

// ---- 构建 ------------------------------------------------------------------

fn setup_scene(
    mut commands: Commands,
    scene: Res<Scene>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let terrain = build_terrain_mesh(&scene.terrain, scene.stride);
    debug_assert_eq!(terrain.vertex_count(), scene.vertex_count);
    commands.spawn((
        Mesh3d(meshes.add(terrain.mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.92,
            ..default()
        })),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 16_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.7, 0.0)),
    ));

    // Bevy 0.20 里 `AmbientLight` 挂在相机上（`#[require(Camera)]`），不能再单独 spawn。
    let (yaw, pitch, dist, target) = scene.home;
    let controller = OrbitCamera::new(yaw, pitch, dist, target);
    let eye = controller.eye();
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 1.0,
            far: 60_000.0,
            ..default()
        }),
        AmbientLight {
            color: Color::WHITE,
            brightness: 320.0,
            affects_lightmapped_meshes: false,
        },
        OrbitCameraBundle {
            transform: Transform::from_translation(eye).looking_at(target, Vec3::Y),
            look: LookTransform::new(eye, target, Vec3::Y),
            smoother: Smoother::new(0.6),
            controller,
        },
    ));
}

fn setup_ui(mut commands: Commands, scene: Res<Scene>) {
    let font = |size: f32| TextFont {
        font: FontSource::Family(CJK_FONT_FAMILY.into()),
        font_size: FontSize::Px(size),
        ..default()
    };

    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: px(10),
            left: px(10),
            width: px(560),
            padding: UiRect::all(px(10)),
            flex_direction: FlexDirection::Column,
            row_gap: px(6),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.7)),
        children![
            (
                Text::new(scene.metrics.lines()),
                font(14.0),
                TextColor(Color::WHITE),
            ),
            (
                Text::new(String::new()),
                font(14.0),
                TextColor(Color::srgb(1.0, 0.9, 0.4)),
                StatusText,
            ),
        ],
    ));

    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            bottom: px(10),
            left: px(10),
            width: px(1180),
            ..default()
        },
        Text::new(
            "按键: Space 播放/暂停 | ←/→ 单步 | ↑/↓ 变速 | R 重头 | \
             鼠标左键拖动=环绕 滚轮=缩放 | Q/E 旋转 Z/X 俯仰 F 重置 | Esc 退出",
        ),
        font(13.0),
        TextColor(Color::srgb(0.88, 0.88, 0.88)),
    ));
}

// ---- 系统 ------------------------------------------------------------------

fn hotkeys(
    keys: Res<ButtonInput<KeyCode>>,
    mut playback: ResMut<Playback>,
    scene: Res<Scene>,
    mut exit: MessageWriter<AppExit>,
) {
    let total = scene.points.len();
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
    if keys.just_pressed(KeyCode::Space) {
        playback.playing = !playback.playing;
    }
    if keys.just_pressed(KeyCode::ArrowRight) || keys.just_pressed(KeyCode::Period) {
        playback.playing = false;
        playback.revealed = (playback.revealed + 1).min(total);
    }
    if keys.just_pressed(KeyCode::ArrowLeft) || keys.just_pressed(KeyCode::Comma) {
        playback.playing = false;
        playback.revealed = playback.revealed.saturating_sub(1);
    }
    if keys.just_pressed(KeyCode::ArrowUp) || keys.just_pressed(KeyCode::Equal) {
        let s = playback.speed * 2.0;
        playback.set_speed(s);
    }
    if keys.just_pressed(KeyCode::ArrowDown) || keys.just_pressed(KeyCode::Minus) {
        let s = playback.speed * 0.5;
        playback.set_speed(s);
    }
    if keys.just_pressed(KeyCode::KeyR) {
        playback.revealed = 0;
        playback.acc = 0.0;
        playback.playing = true;
    }
}

fn playback_tick(time: Res<Time>, scene: Res<Scene>, mut playback: ResMut<Playback>) {
    if !playback.playing {
        return;
    }
    let total = scene.points.len();
    playback.acc += time.delta_secs() * POINTS_PER_SEC * playback.speed;
    while playback.acc >= 1.0 && playback.revealed < total {
        playback.acc -= 1.0;
        playback.revealed += 1;
    }
    if playback.revealed >= total {
        playback.playing = false;
        playback.acc = 0.0;
    }
}

fn draw_scene(scene: Res<Scene>, playback: Res<Playback>, mut gizmos: Gizmos) {
    let total = scene.points.len();
    let revealed = playback.revealed.min(total);
    // 已走过的折线。
    for w in scene.points[..revealed].windows(2) {
        gizmos.line(w[0], w[1], Color::srgb(0.9, 0.9, 0.95));
    }
    // 尚未走过的折线（暗色预览）。
    if revealed > 0 && revealed <= total.saturating_sub(1) {
        for w in scene.points[revealed - 1..].windows(2) {
            gizmos.line(w[0], w[1], Color::srgba(0.45, 0.45, 0.5, 0.8));
        }
    }
    // 左右脚不同颜色；已揭示的更亮更大。
    for (i, p) in scene.points.iter().enumerate() {
        let used = i < revealed;
        let color = match (Foot::of_index(i), used) {
            (Foot::Left, true) => Color::srgb(0.2, 0.95, 0.35),
            (Foot::Left, false) => Color::srgba(0.2, 0.95, 0.35, 0.4),
            (Foot::Right, true) => Color::srgb(1.0, 0.55, 0.1),
            (Foot::Right, false) => Color::srgba(1.0, 0.55, 0.1, 0.4),
        };
        let r = if used { FOOT_RADIUS } else { FOOT_RADIUS * 0.6 };
        gizmos.sphere(Isometry3d::from_translation(*p), r, color);
    }
    // 起点 / 当前点 / 目标点。
    if let Some(first) = scene.points.first() {
        gizmos.sphere(
            Isometry3d::from_translation(*first),
            20.0,
            Color::srgb(0.2, 0.5, 1.0),
        );
    }
    if revealed > 0 {
        gizmos.sphere(
            Isometry3d::from_translation(scene.points[revealed - 1]),
            17.0,
            Color::srgb(1.0, 1.0, 0.2),
        );
    }
    gizmos.sphere(
        Isometry3d::from_translation(scene.goal),
        24.0,
        Color::srgb(1.0, 0.15, 0.15),
    );
}

fn update_status(
    scene: Res<Scene>,
    playback: Res<Playback>,
    mut status: Query<&mut Text, With<StatusText>>,
) {
    let Ok(mut t) = status.single_mut() else {
        return;
    };
    t.0 = format!(
        "回放: {}/{} 点  速度 {:.2}x\n地形: {}x{} (原 {} 格) stride={} -> {} 顶点 / {} 三角形",
        playback.revealed,
        scene.points.len(),
        playback.speed,
        scene.rows,
        scene.cols,
        scene.raw_cells,
        scene.stride,
        scene.vertex_count,
        scene.triangle_count,
    );
}

fn centroid(points: &[Vec3]) -> Vec3 {
    if points.is_empty() {
        return Vec3::ZERO;
    }
    let mut sum = Vec3::ZERO;
    for p in points {
        sum += *p;
    }
    sum / points.len() as f32
}
