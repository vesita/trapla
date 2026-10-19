//! Bevy 应用：地形 + 落足点回放 + 指标面板 + 沙盒主菜单。
//!
//! 场景加载挪进 `OnEnter(GamePhase::Playing)`：菜单帧里不读盘、不建网格；
//! `--headless-selftest` 不建 App（走 [`crate::session::load`] + 打印），不受此影响。
//!
//! 截图模式必须**等渲染管线编译完成**再截图：Bevy 默认异步编译管线，
//! 若在第 N 帧就截，前若干帧可能只有清屏色而没有网格。这里打开
//! `RenderPlugin::synchronous_pipeline_compilation`（Bevy 给确定性测试用的开关）。

use std::path::PathBuf;

use bevy::app::AppExit;
use bevy::pbr::{ContactShadows, ScreenSpaceAmbientOcclusion};
use bevy::prelude::*;
use bevy::render::RenderPlugin;
use bevy::render::view::Tonemapping;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::text::{FontSize, FontSource};
use bevy::window::WindowResolution;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use trapla_core::geom::Dot;
use trapla_lab::validate::{RzConvention, ShapeLimits, ValidateOptions, foot_poses};

use crate::cli::Mode;
use crate::data::{Foot, Terrain};
use crate::menu::SandboxMenuPlugin;
use crate::metrics::MetricsPanel;
use crate::orbit_camera::{
    LookTransform, OrbitCamera, OrbitCameraBundle, OrbitCameraPlugin, Smoother,
};
use crate::phase::GamePhase;
use crate::scene::{
    DisplayGrid, Downsample, SURFACE_LIFT_CM, band_color, build_base_mesh, build_terrain_mesh,
    display_grid, height_to_world, path_world_points, sample_shape, world_from_map,
};
use crate::session::{self, MenuOptions, MenuSelection, SceneInputs, SceneSource};

/// 启动 Bevy 应用（交互或截图模式）。
///
/// `--headless-selftest` 不走这里：它不建窗口，见 `main::headless_selftest`。
pub struct RunConfig {
    /// 运行模式（截图路径 / 交互）。
    pub mode: Mode,
    /// 截图模式渲染的帧数。
    pub frames: u32,
    /// 场景来源。
    pub source: SceneSource,
    /// 启动先进入主菜单；`false` 时直接进 `Playing`（截图 / 显式输入路径）。
    pub start_in_menu: bool,
    /// 垂直尺度：1 cm 高度 = 多少世界单位（`1.0` = 真尺度）。
    pub exaggeration: f32,
    /// 足矩形的**绘制**放大倍数（只影响画面，不改足的真实尺寸与点位）。
    pub foot_scale: f32,
    /// 足长方体的厚度（厘米，真实尺寸）。
    pub foot_thickness: f32,
    /// 地面底座厚度（厘米）；`0` = 不加底座。
    pub base_depth: f32,
    /// 显示网格的降采样口径。
    pub downsample: Downsample,
    /// 足朝向重建约定；`Raw` 只用于「不许用步向」的负向对照。
    pub foot_heading: RzConvention,
}

/// 纯绘制选项（不影响加载的数据与几何计数）。
#[derive(Resource, Debug, Clone, Copy)]
struct DrawOptions {
    exaggeration: f32,
    foot_scale: f32,
    foot_thickness: f32,
    base_depth: f32,
    downsample: Downsample,
    foot_heading: RzConvention,
    /// 截图模式的确定性开关：回放一次性揭示整条路径，PNG 变成输入的纯函数。
    deterministic_playback: bool,
}

/// 一只脚的重建姿态：地图坐标（用于按校验器的形状算四角）+ 世界坐标 + 朝向。
struct FootPose {
    /// 足中心（地图坐标）。
    map: Dot,
    /// 足中心（世界坐标）。
    world: Vec3,
    /// 重建出的朝向 `rz`（弧度，= 世界 yaw）。
    rz: f64,
    /// 这一步在点位序列里的下标。
    index: usize,
}

/// 场景资源（`OnEnter(Playing)` 里从 [`SceneInputs`] 构建）。
#[derive(Resource)]
struct Scene {
    terrain: Terrain,
    /// 显示网格（窗口均值降采样）：地形网格、底座、足高、遮挡判断共用它。
    grid: DisplayGrid,
    points: Vec<Vec3>,
    /// 平均每只脚的重建姿态（文件 / 沙盒都一样；沙盒没有路径时为空）。
    foot_poses: Vec<FootPose>,
    /// 校验器用的足形（绘制时按 `--foot-scale` 放大的是它的副本）。
    foot_shape: ShapeLimits,
    /// 起点标记（沙盒场景显式给出；文件模式的起点由路径首点表示）。
    start: Option<Vec3>,
    goal: Vec3,
    metrics: MetricsPanel,
    vertex_count: usize,
    triangle_count: usize,
    stride: usize,
    rows: usize,
    cols: usize,
    raw_cells: usize,
    /// 垂直尺度（同时用于相机取景与 HUD 显示）。
    exaggeration: f32,
    /// 足矩形绘制放大倍数。
    foot_scale: f32,
    /// 足长方体厚度（厘米，真实尺寸）。
    foot_thickness: f32,
    /// 底座厚度（厘米；0 = 不加底座）。
    base_depth: f32,
    /// 采样高度的实际范围（厘米）。
    height_min: f32,
    height_max: f32,
    /// 相机初始位姿 `(yaw, pitch, dist, target)`。
    home: (f32, f32, f32, Vec3),
}

impl Scene {
    fn build(inputs: SceneInputs, draw: DrawOptions) -> Self {
        let SceneInputs {
            terrain,
            path,
            metrics,
            stride,
            goal,
            start,
            ..
        } = inputs;
        let exaggeration = draw.exaggeration;
        let stride = stride.max(1);
        let grid = display_grid(&terrain, stride, draw.downsample);
        let points = path_world_points(&terrain, &grid, &path, exaggeration);
        let lift = Vec3::Y * height_to_world(SURFACE_LIFT_CM, exaggeration);
        let goal = world_from_map(&terrain, &grid, goal.0, goal.1, exaggeration) + lift;
        let start = start.map(|(x, y)| world_from_map(&terrain, &grid, x, y, exaggeration) + lift);
        let (rows, cols) = (terrain.rows(), terrain.cols());
        let (n_rows, n_cols) = sample_shape(rows, cols, stride);
        let (height_min, height_max) = grid.min_max();

        let foot_shape = ValidateOptions::canonical().limits.shape;
        let foot_poses = build_foot_poses(&terrain, &grid, &path, draw, lift);

        let centroid = centroid(&points);
        let extent = if points.is_empty() {
            // 沙盒场景没有规划路径：用地图对角线的一半作为取景半径。
            ((rows as f32).hypot(cols as f32) * 0.5).max(120.0)
        } else {
            points
                .iter()
                .map(|p| p.distance(centroid))
                .fold(0.0_f32, f32::max)
                .max(120.0)
        };
        let dist = (extent * 2.6).clamp(200.0, 12_000.0);
        let home = (0.55_f32, 0.62_f32, dist, centroid);

        Self {
            terrain,
            grid,
            points,
            foot_poses,
            foot_shape,
            start,
            goal,
            metrics,
            vertex_count: n_rows * n_cols,
            triangle_count: n_rows.saturating_sub(1) * n_cols.saturating_sub(1) * 2,
            stride,
            rows,
            cols,
            raw_cells: rows * cols,
            exaggeration,
            foot_scale: draw.foot_scale,
            foot_thickness: draw.foot_thickness,
            base_depth: draw.base_depth,
            height_min,
            height_max,
            home,
        }
    }

    /// 显示表面在某个地图坐标处的高度（世界 Y）。
    fn surface_y(&self, map_x: f64, map_y: f64) -> f32 {
        height_to_world(self.grid.height_at_map(map_x, map_y), self.exaggeration)
    }

    /// 显示表面在某个采样格处的法线（世界坐标，用于把足贴在坡面上）。
    fn surface_normal(&self, map_x: f64, map_y: f64) -> Vec3 {
        let (i, j) = self.grid.cell_of(map_x, map_y);
        let spacing = self.grid.stride as f32;
        let up = self.grid.height(i.saturating_sub(1), j);
        let down = self.grid.height((i + 1).min(self.grid.rows - 1), j);
        let left = self.grid.height(i, j.saturating_sub(1));
        let right = self.grid.height(i, (j + 1).min(self.grid.cols - 1));
        // 世界轴：X = 列、Z = 行、Y = 高度（厘米 × 垂直尺度）。
        let dy_dx = (right - left) / (2.0 * spacing) * self.exaggeration;
        let dy_dz = (down - up) / (2.0 * spacing) * self.exaggeration;
        Vec3::new(-dy_dx, 1.0, -dy_dz).normalize_or_zero()
    }
}

/// 逐点重建落足姿态：朝向规则的唯一实现处在 `trapla-lab::validate::foot_poses`。
///
/// 文件输入没有机器人起始位姿，起点取路径首点 —— 它只影响前两步的初始足位；
/// 从第 3 个点起，同脚相邻点完全决定 `rz`。
fn build_foot_poses(
    terrain: &Terrain,
    grid: &DisplayGrid,
    path: &crate::data::FootPath,
    draw: DrawOptions,
    lift: Vec3,
) -> Vec<FootPose> {
    if path.len() == 0 {
        return Vec::new();
    }
    let dots: Vec<Dot> = path.points().iter().map(|&(x, y)| Dot::new(x, y)).collect();
    let options = ValidateOptions {
        rz: draw.foot_heading,
        ..ValidateOptions::canonical()
    };
    foot_poses(&dots, dots[0], &options)
        .into_iter()
        .enumerate()
        .map(|(index, (map, rz))| FootPose {
            map,
            world: world_from_map(terrain, grid, map.x(), map.y(), draw.exaggeration) + lift,
            rz,
            index,
        })
        .collect()
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

impl Default for Playback {
    fn default() -> Self {
        Self {
            revealed: 0,
            playing: true,
            speed: 1.0,
            acc: 0.0,
        }
    }
}

impl Playback {
    /// 速度档位夹取：`0.25x ..= 8x`。
    fn set_speed(&mut self, v: f32) {
        self.speed = v.clamp(0.25, 8.0);
    }

    /// 截图模式：一次性揭示整条路径，成像与时间/帧数无关。
    fn fully_revealed(total: usize) -> Self {
        Self {
            revealed: total,
            playing: false,
            speed: 1.0,
            acc: 0.0,
        }
    }
}

/// 这次游玩是从菜单进的，还是显式输入直接进的。
///
/// 决定 `Esc` 的语义：菜单进来的回菜单（世界盒），显式输入的直接退出（保持旧行为）。
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
enum SessionKind {
    /// 从主菜单选择进入：`Esc` 回主菜单。
    Menu,
    /// 显式 `--map`/`--path`（或 `--scenario`）直接进入：`Esc` 退出。
    Direct,
}

/// `OnEnter(Playing)` 出生、`OnExit(Playing)` 回收的实体标记。
#[derive(Component)]
struct SceneEntity;

/// 实体长方体足（`index` = 点位序列里的下标，用于步序亮度与左右脚分色）。
#[derive(Component)]
struct FootCube {
    index: usize,
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

/// 足矩形的左右脚配色（蓝 / 橙）：明度与色相都拉开，灰度下也能区分。
fn foot_color(foot: Foot) -> Color {
    match foot {
        Foot::Left => Color::srgb(0.20, 0.72, 1.0),
        Foot::Right => Color::srgb(1.0, 0.42, 0.05),
    }
}

/// 足朝向刻线的颜色（比足色更亮，指向明确）。
const FOOT_HEADING_COLOR: Color = Color::srgb(1.0, 1.0, 0.85);

/// 每几步标一个步序号（egui 屏幕空间叠加）。
const STEP_LABEL_EVERY: usize = 5;

/// 地面网格间距（世界单位 = 厘米）。
const GRID_SPACING: f32 = 10.0;

/// 截图落盘后额外等待的帧数（确认写入完成）。
const SHOT_SETTLE_FRAMES: u32 = 12;

/// 截图等待上限（帧）。
const SHOT_TIMEOUT_FRAMES: u32 = 900;

/// 中文字体族名（由系统字体发现解析；缺失时字体栈回退到通用 sans-serif）。
const CJK_FONT_FAMILY: &str = "Noto Sans CJK SC";

/// 主菜单帧的相机位姿：菜单不加载场景，先给一个固定远视角。
const MENU_HOME: (f32, f32, f32) = (0.55, 0.62, 500.0);

/// 启动 Bevy 应用。返回后调用方仍需自行核对 PNG 是否落盘。
pub fn run(config: RunConfig) -> Result<(), String> {
    let sync_pipelines = matches!(&config.mode, Mode::Screenshot(_));

    // 交互模式用异步编译（启动快）；截图模式必须同步，否则前若干帧还没有网格。
    let (shot, title) = match &config.mode {
        Mode::Screenshot(path) => (
            Some(Shot {
                path: path.clone(),
                after_frames: config.frames.max(1),
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
    // EguiPlugin 必须先于 SandboxMenuPlugin：菜单系统要往它注册的 schedule 里加。
    .add_plugins(EguiPlugin::default())
    .add_plugins(OrbitCameraPlugin)
    .add_plugins(SandboxMenuPlugin)
    // A5 后处理：只开这四件（TAA / AutoExposure / MotionBlur 一律不加，它们帧相关）。
    // 插件本身已由 `PbrPlugin` 注册（0.20 实测：再加一次会 panic「plugin was already added」），
    // 这里只给相机挂组件。
    .insert_resource(ClearColor(Color::srgb(0.09, 0.11, 0.16)))
    .insert_resource(MenuOptions::builtin())
    .insert_resource(DrawOptions {
        exaggeration: config.exaggeration,
        foot_scale: config.foot_scale,
        foot_thickness: config.foot_thickness,
        base_depth: config.base_depth,
        downsample: config.downsample,
        foot_heading: config.foot_heading,
        deterministic_playback: sync_pipelines,
    })
    .init_resource::<MenuSelection>()
    .insert_resource(config.source)
    .add_systems(Startup, spawn_camera)
    .add_systems(OnEnter(GamePhase::Playing), enter_playing)
    .add_systems(OnExit(GamePhase::Playing), exit_playing)
    .add_systems(
        Update,
        (
            // 相机在 `Startup` 出生；直接进 `Playing`（截图 / 显式输入）时 `OnEnter` 早于
            // `Startup`，所以归位放在 `Scene` 刚插入的那一帧，这时的相机一定已经存在。
            align_camera_to_scene.run_if(resource_added::<Scene>),
            hotkeys,
            playback_tick,
            update_foot_colors,
            draw_scene,
            update_status,
        )
            .chain()
            .run_if(in_state(GamePhase::Playing)),
    )
    // 步序号是屏幕空间叠加（Bevy 0.20 无世界空间 3D 文字），必须放 egui 的 pass。
    .add_systems(
        EguiPrimaryContextPass,
        step_number_overlay.run_if(in_state(GamePhase::Playing)),
    );

    if config.start_in_menu {
        app.init_state::<GamePhase>();
    } else {
        app.insert_state(GamePhase::Playing);
    }

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

// ---- 出生 / 回收 -----------------------------------------------------------

/// 相机在 `Startup` 就位（主菜单也需要一台相机来渲染 egui）；
/// 进入 `Playing` 后由 [`align_camera_to_scene`] 按场景的取景位姿归位。
fn spawn_camera(mut commands: Commands) {
    let (yaw, pitch, dist) = MENU_HOME;
    let controller = OrbitCamera::new(yaw, pitch, dist, Vec3::ZERO);
    let eye = controller.eye();
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 1.0,
            far: 60_000.0,
            ..default()
        }),
        // Bevy 0.20 里 `AmbientLight` 挂在相机上（`#[require(Camera)]`），不能再单独 spawn。
        AmbientLight {
            color: Color::WHITE,
            brightness: 320.0,
            affects_lightmapped_meshes: false,
        },
        // A5：固定色调映射 + 距离雾 + 接触阴影 + SSAO（都不用帧相关输入）。
        // `Msaa::Off` 是 SSAO 的硬前提（0.20 下挂 SSAO 而 MSAA 非 Off 会每帧刷 ERROR）；
        // 补锯齿的 TAA 帧相关，按确定性要求排除。
        Msaa::Off,
        Tonemapping::TonyMcMapface,
        ContactShadows::default(),
        ScreenSpaceAmbientOcclusion::default(),
        DistanceFog {
            color: Color::srgb(0.09, 0.11, 0.16),
            directional_light_color: Color::NONE,
            directional_light_exponent: 0.0,
            falloff: FogFalloff::Linear {
                start: 1_000.0,
                end: 4_000.0,
            },
        },
        OrbitCameraBundle {
            transform: Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y),
            look: LookTransform::new(eye, Vec3::ZERO, Vec3::Y),
            smoother: Smoother::new(0.6),
            controller,
        },
    ));
}

/// 进入 `Playing`：消费菜单选择 → 加载场景 → 建网格 / HUD（相机归位见 [`align_camera_to_scene`]）。
fn enter_playing(
    mut commands: Commands,
    mut source: ResMut<SceneSource>,
    mut selection: ResMut<MenuSelection>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    draw: Res<DrawOptions>,
    mut exit: MessageWriter<AppExit>,
) {
    // 菜单写选择、进状态时消费（breeze 的 MenuAction 模式）。
    let from_menu = selection.0.take();
    let entered_from_menu = from_menu.is_some();
    if let Some(id) = from_menu {
        *source = SceneSource::Scenario { id };
    }

    let inputs = match session::load(&source) {
        Ok(inputs) => inputs,
        Err(e) => {
            error!("加载场景失败: {e}");
            exit.write(AppExit::error());
            return;
        }
    };
    info!(
        "进入 Playing: {} 路径点数={} stride={}（来源: {}）",
        inputs.map_label,
        inputs.path.len(),
        inputs.stride,
        if entered_from_menu {
            "菜单"
        } else {
            "显式输入"
        }
    );
    let scene = Scene::build(inputs, *draw);
    let revealed = if draw.deterministic_playback {
        scene.points.len()
    } else {
        0
    };

    spawn_terrain(&mut commands, &mut meshes, &mut materials, &scene);
    spawn_feet(&mut commands, &mut meshes, &mut materials, &scene, revealed);
    spawn_ui(&mut commands, &scene, entered_from_menu);

    commands.insert_resource(if entered_from_menu {
        SessionKind::Menu
    } else {
        SessionKind::Direct
    });
    let playback = if draw.deterministic_playback {
        // 截图模式：路径一次揭示完，PNG 只由输入决定（与帧数 / 墙钟无关）。
        Playback::fully_revealed(scene.points.len())
    } else {
        Playback::default()
    };
    commands.insert_resource(scene);
    commands.insert_resource(playback);
}

/// 退出 `Playing`：回收这一局的所有实体与资源（回菜单后不残留旧地形）。
fn exit_playing(mut commands: Commands, entities: Query<Entity, With<SceneEntity>>) {
    info!("退出 Playing: 回收 {} 个场景实体", entities.iter().count());
    for entity in &entities {
        commands.entity(entity).despawn();
    }
    commands.remove_resource::<Scene>();
    commands.remove_resource::<Playback>();
}

/// 新场景就位时把相机移到它的取景位姿（`Scene` 插入的那一帧执行一次），
/// 并让距离雾的范围跟着取景距离走（沙盒的近景不该被雾吃掉）。
fn align_camera_to_scene(
    scene: Res<Scene>,
    mut cameras: Query<
        (
            &mut OrbitCamera,
            &mut LookTransform,
            &mut Smoother,
            &mut DistanceFog,
        ),
        With<Camera3d>,
    >,
) {
    for (mut controller, mut look, mut smoother, mut fog) in &mut cameras {
        let (yaw, pitch, dist, target) = scene.home;
        *controller = OrbitCamera::new(yaw, pitch, dist, target);
        let eye = controller.eye();
        *look = LookTransform::new(eye, target, Vec3::Y);
        smoother.reset();
        fog.falloff = FogFalloff::Linear {
            start: dist * 1.2,
            end: dist * 4.0,
        };
    }
}

fn spawn_terrain(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scene: &Scene,
) {
    let terrain = build_terrain_mesh(
        &scene.terrain,
        scene.stride,
        scene.exaggeration,
        scene.grid.mode,
    );
    debug_assert_eq!(terrain.vertex_count(), scene.vertex_count);
    commands.spawn((
        SceneEntity,
        Mesh3d(meshes.add(terrain.mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.92,
            ..default()
        })),
    ));

    // ④ 地面底座：侧壁 + 底盖，双面材质（不必纠结绕序），侧壁按高度上色。
    if scene.base_depth > 0.0 {
        let base = build_base_mesh(
            &scene.terrain,
            &scene.grid,
            scene.exaggeration,
            scene.base_depth,
        );
        commands.spawn((
            SceneEntity,
            Mesh3d(meshes.add(base)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::WHITE,
                perceptual_roughness: 0.95,
                cull_mode: None,
                ..default()
            })),
        ));
    }

    commands.spawn((
        SceneEntity,
        DirectionalLight {
            illuminance: 16_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.7, 0.0)),
    ));
}

/// ① 实体长方体足：与校验器同一套形状（长 5 × 宽 3 × 厚 `--foot-thickness`），
/// 朝向来自 `foot_poses` 的 `rz`，再按显示表面的法线贴到坡面上。
/// `--foot-scale` 只放大绘制尺寸；足的真实尺寸与点位不变。
fn spawn_feet(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    scene: &Scene,
    revealed: usize,
) {
    if scene.foot_poses.is_empty() {
        return;
    }
    let size = Vec3::new(
        scene.foot_shape.width as f32 * scene.foot_scale,
        scene.foot_thickness * scene.foot_scale,
        scene.foot_shape.length as f32 * scene.foot_scale,
    );
    let cube = meshes.add(Cuboid::new(size.x, size.y, size.z));
    let total = scene.foot_poses.len();

    for pose in &scene.foot_poses {
        let surface = scene.surface_y(pose.map.x(), pose.map.y());
        let normal = scene.surface_normal(pose.map.x(), pose.map.y());
        // 先绕 Y 转出朝向，再整体抬到坡面法线上（`+Y` 对齐 `normal`）。
        let rotation =
            Quat::from_rotation_arc(Vec3::Y, normal) * Quat::from_rotation_y(pose.rz as f32);
        let material = materials.add(StandardMaterial {
            base_color: foot_display_color(pose.index, total, revealed),
            perceptual_roughness: 0.55,
            ..default()
        });
        commands.spawn((
            SceneEntity,
            FootCube { index: pose.index },
            Mesh3d(cube.clone()),
            MeshMaterial3d(material),
            Transform::from_translation(Vec3::new(
                pose.world.x,
                surface + size.y / 2.0,
                pose.world.z,
            ))
            .with_rotation(rotation),
        ));
    }
}

/// 足的颜色：左右脚高对比分色 × 步序亮度（越靠后越亮）；未揭示的压暗。
fn foot_display_color(index: usize, total: usize, revealed: usize) -> Color {
    let used = index < revealed;
    let t = if total > 1 {
        index as f32 / (total - 1) as f32
    } else {
        1.0
    };
    let brightness = if used { 0.55 + 0.45 * t } else { 0.18 };
    tint(foot_color(Foot::of_index(index)), brightness)
}

fn spawn_ui(commands: &mut Commands, scene: &Scene, from_menu: bool) {
    let font = |size: f32| TextFont {
        font: FontSource::Family(CJK_FONT_FAMILY.into()),
        font_size: FontSize::Px(size),
        ..default()
    };
    // 底部提示按进入方式给 `Esc` 的正确语义；显式输入（截图 / 旧交互路径）保持原文
    // —— 文案属于截图画面的一部分，不因新增菜单而改动旧路径的像素。
    let help = if from_menu {
        "按键: Space 播放/暂停 | ←/→ 单步 | ↑/↓ 变速 | R 重头 | \
         鼠标左键拖动=环绕 滚轮=缩放 | Q/E 旋转 Z/X 俯仰 F 重置 | Esc 返回菜单"
    } else {
        "按键: Space 播放/暂停 | ←/→ 单步 | ↑/↓ 变速 | R 重头 | \
         鼠标左键拖动=环绕 滚轮=缩放 | Q/E 旋转 Z/X 俯仰 F 重置 | Esc 退出"
    };

    commands.spawn((
        SceneEntity,
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
        SceneEntity,
        Node {
            position_type: PositionType::Absolute,
            right: px(12),
            bottom: px(36),
            padding: UiRect::all(px(8)),
            flex_direction: FlexDirection::Column,
            align_items: AlignItems::Center,
            row_gap: px(4),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.65)),
        children![
            (
                Text::new(format!(
                    "高程 {:.0}–{:.0} cm（数据实际范围，低处拉伸）  垂直 ×{:.2}  足 ×{:.1}",
                    scene.height_min, scene.height_max, scene.exaggeration, scene.foot_scale
                )),
                font(12.0),
                TextColor(Color::srgb(0.92, 0.92, 0.92)),
            ),
            (
                Node {
                    flex_direction: FlexDirection::Row,
                    height: px(14),
                    ..default()
                },
                children![
                    band_swatch(0),
                    band_swatch(1),
                    band_swatch(2),
                    band_swatch(3),
                    band_swatch(4),
                    band_swatch(5),
                    band_swatch(6),
                    band_swatch(7),
                ],
            ),
            (
                Text::new(format!(
                    "低 {:.0} cm  ←  色带  →  高 {:.0} cm",
                    scene.height_min, scene.height_max
                )),
                font(11.0),
                TextColor(Color::srgb(0.78, 0.78, 0.78)),
            ),
        ],
    ));

    commands.spawn((
        SceneEntity,
        Node {
            position_type: PositionType::Absolute,
            bottom: px(10),
            left: px(10),
            width: px(1180),
            ..default()
        },
        Text::new(help),
        font(13.0),
        TextColor(Color::srgb(0.88, 0.88, 0.88)),
    ));
}

/// 色标里的一格：取该段的中点颜色（`band_color` 是色带插值的唯一实现处）。
fn band_swatch(i: usize) -> (Node, BackgroundColor) {
    let t = (i as f32 + 0.5) / 8.0;
    let [r, g, b] = band_color(t);
    (
        Node {
            width: px(30),
            height: px(14),
            ..default()
        },
        BackgroundColor(Color::srgb(r, g, b)),
    )
}

// ---- 系统 ------------------------------------------------------------------

fn hotkeys(
    keys: Res<ButtonInput<KeyCode>>,
    mut playback: ResMut<Playback>,
    scene: Res<Scene>,
    session: Res<SessionKind>,
    mut next_state: ResMut<NextState<GamePhase>>,
    mut exit: MessageWriter<AppExit>,
) {
    let total = scene.points.len();
    if keys.just_pressed(KeyCode::Escape) {
        match *session {
            SessionKind::Menu => next_state.set(GamePhase::MainMenu),
            SessionKind::Direct => {
                exit.write(AppExit::Success);
            }
        }
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

    // A4：地面网格（贴地形最低处），给空间尺度一个参考；gizmo 不参与光照与遮挡。
    let cells_x = ((scene.cols.saturating_sub(1)) as f32 / GRID_SPACING)
        .ceil()
        .max(1.0) as u32;
    let cells_z = ((scene.rows.saturating_sub(1)) as f32 / GRID_SPACING)
        .ceil()
        .max(1.0) as u32;
    gizmos.grid_3d(
        Isometry3d::from_translation(Vec3::new(
            0.0,
            height_to_world(scene.height_min, scene.exaggeration),
            0.0,
        )),
        UVec3::new(cells_x, 1, cells_z),
        Vec3::new(GRID_SPACING, 1.0, GRID_SPACING),
        Color::srgba(0.55, 0.62, 0.72, 0.28),
    );

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

    // ① 足长方体本身是真实 mesh（见 `spawn_feet`）；这里只补一根朝向刻线，
    // 让「朝哪」在长方体的长轴之外还有一个更醒目的指示。
    if !scene.foot_poses.is_empty() {
        let half_len = scene.foot_shape.length as f32 * scene.foot_scale / 2.0;
        let top_offset = scene.foot_thickness * scene.foot_scale;
        for pose in &scene.foot_poses {
            let brightness = if pose.index < revealed { 1.0 } else { 0.45 };
            let top = Vec3::new(
                pose.world.x,
                scene.surface_y(pose.map.x(), pose.map.y()) + top_offset,
                pose.world.z,
            );
            let heading = Vec3::new(pose.rz.sin() as f32, 0.0, pose.rz.cos() as f32);
            gizmos.line(
                top,
                top + heading * half_len * 1.35,
                tint(FOOT_HEADING_COLOR, brightness),
            );
        }
    }

    // 起点：文件模式由路径首点表示；沙盒场景显式标出。
    if let Some(first) = scene.points.first() {
        gizmos.sphere(
            Isometry3d::from_translation(*first),
            20.0,
            Color::srgb(0.2, 0.5, 1.0),
        );
    } else if let Some(start) = scene.start {
        gizmos.sphere(
            Isometry3d::from_translation(start),
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

/// 按亮度缩放颜色（保留色相 / alpha）。
fn tint(color: Color, factor: f32) -> Color {
    let s = color.to_srgba();
    Color::srgba(s.red * factor, s.green * factor, s.blue * factor, s.alpha)
}

/// 回放推进时更新足长方体的颜色（步序亮度 / 未揭示压暗）。
///
/// 只在 `Playback` 变化的那一帧写材质：截图模式下一帧都不写。
fn update_foot_colors(
    playback: Res<Playback>,
    scene: Res<Scene>,
    feet: Query<(&FootCube, &MeshMaterial3d<StandardMaterial>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !playback.is_changed() {
        return;
    }
    let total = scene.foot_poses.len();
    for (foot, handle) in &feet {
        if let Some(mut material) = materials.get_mut(&handle.0) {
            material.base_color = foot_display_color(foot.index, total, playback.revealed);
        }
    }
}

/// 相机看得见这个点吗：先剔背后的点，再沿相机→点的线段在高度场里步进做遮挡判断。
///
/// 高度场是规则网格，遮挡判断不需要深度缓冲 —— 步进采样就够（要标注的足只有个位数）。
fn visible_from_camera(scene: &Scene, eye: Vec3, forward: Vec3, target: Vec3) -> bool {
    let to_target = target - eye;
    if to_target.length_squared() <= f32::EPSILON || forward.dot(to_target.normalize()) <= 0.0 {
        return false;
    }
    const STEPS: usize = 128;
    let half_cols = (scene.terrain.cols() as f32 - 1.0) / 2.0;
    let half_rows = (scene.terrain.rows() as f32 - 1.0) / 2.0;
    for k in 1..STEPS {
        let t = k as f32 / STEPS as f32;
        let p = eye.lerp(target, t);
        let map_x = p.z + half_rows;
        let map_y = p.x + half_cols;
        if map_x < 0.0
            || map_y < 0.0
            || map_x >= scene.terrain.rows() as f32
            || map_y >= scene.terrain.cols() as f32
        {
            continue;
        }
        let surface = scene.surface_y(map_x as f64, map_y as f64);
        if p.y < surface - 0.01 {
            return true;
        }
    }
    false
}

/// 步序号：屏幕空间叠加（Bevy 0.20 没有世界空间 3D 文字）。
///
/// 每 [`STEP_LABEL_EVERY`] 步标一个数字，外加最后一步。**先做可见性判断**：
/// 只给相机前方、且没有被高度场挡住的落足标注 —— 否则远处的步数字会浮在山脊上方的天空里。
/// 输入只有场景与相机，不含时间/帧数，因此不影响截图确定性。
fn step_number_overlay(
    mut egui_ctx: EguiContexts,
    scene: Res<Scene>,
    cameras: Query<(&Camera, &GlobalTransform)>,
) {
    let Ok(ctx) = egui_ctx.ctx_mut() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let eye = camera_transform.translation();
    let forward: Vec3 = camera_transform.forward().into();
    let ppp = ctx.pixels_per_point();
    let screen = ctx.viewport_rect();
    let painter = ctx.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("trapla_step_numbers"),
    ));
    let last = scene.foot_poses.len().saturating_sub(1);
    for pose in &scene.foot_poses {
        if pose.index % STEP_LABEL_EVERY != 0 && pose.index != last {
            continue;
        }
        if !visible_from_camera(&scene, eye, forward, pose.world) {
            continue;
        }
        let Ok(viewport) = camera.world_to_viewport(camera_transform, pose.world) else {
            continue;
        };
        let pos = egui::pos2(viewport.x / ppp, viewport.y / ppp);
        if !screen.contains(pos) {
            continue;
        }
        painter.text(
            pos + egui::vec2(12.0, -12.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{}", pose.index + 1),
            egui::FontId::monospace(11.0),
            egui::Color32::from_rgb(255, 235, 120),
        );
    }
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
