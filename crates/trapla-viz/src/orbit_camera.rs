//! 环绕相机：把「眼睛 / 注视点」与场景 `Transform` 解耦，中间用指数平滑连接。
//!
//! ## 模式来源
//!
//! 分层照 `smooth-bevy-cameras` crate：`look_transform`（`LookTransform` / `Smoother` /
//! 同步系统）与 `controllers::orbit`（控制器只写 `LookTransform`，由同步系统落到
//! `Transform`）分成两层。本模块按同一分层重写成本 crate 自己的代码，**不引入该 crate、
//! 也不引入任何 path 依赖**，改动只影响本仓。
//!
//! ## 角度约定
//!
//! 与 breeze 的 `LookAngles` 一致（两边混用时手感才不会相反）：
//!
//! - `yaw` 绕 Y 轴；`pitch` 为正 = 相机在目标**上方**（俯视目标）。
//! - 鼠标右移 → 视角右转（`yaw -= dx`）；鼠标上移 → 抬头（`pitch += dy`，上移时 `dy < 0`）。
//! - 单位方向向量 = `Ry(yaw) * Z` 再绕水平轴转 `pitch`，即
//!   `(cos(pitch)·sin(yaw), sin(pitch), cos(pitch)·cos(yaw))`。

use bevy::input::mouse::{AccumulatedMouseMotion, MouseWheel};
use bevy::prelude::*;

/// 相机实体 bundle：控制器 + 注视位姿 + 平滑器 + 场景 `Transform`。
///
/// `Transform` 由 [`look_transform_system`] 每帧从 `LookTransform` 同步，不要手改。
#[derive(Bundle)]
pub struct OrbitCameraBundle {
    /// 控制器状态。
    pub controller: OrbitCamera,
    /// 目标位姿。
    pub look: LookTransform,
    /// 平滑器。
    pub smoother: Smoother,
    /// 场景变换（同步目标）。
    pub transform: Transform,
}

/// 眼睛与它注视的目标点。
#[derive(Component, Debug, Clone, Copy, PartialEq)]
pub struct LookTransform {
    /// 相机位置。
    pub eye: Vec3,
    /// 注视点。
    pub target: Vec3,
    /// 上方向。
    pub up: Vec3,
}

impl LookTransform {
    /// 新建。
    pub fn new(eye: Vec3, target: Vec3, up: Vec3) -> Self {
        Self { eye, target, up }
    }

    /// 眼睛到目标点的距离。
    pub fn radius(&self) -> f32 {
        (self.target - self.eye).length()
    }
}

impl From<LookTransform> for Transform {
    fn from(t: LookTransform) -> Self {
        let dir = (t.target - t.eye).normalize_or_zero();
        Transform::from_translation(t.eye).looking_at(t.eye + dir, t.up)
    }
}

/// 对 [`LookTransform`] 做指数平滑。
///
/// `lag_weight` 取值 `0.0..1.0`，越大越「粘」（更平滑、跟手更慢）；
/// 首帧直接从目标位姿开始，不会从原点飞过来。
#[derive(Component, Debug, Clone, Copy)]
pub struct Smoother {
    lag_weight: f32,
    lerp: Option<LookTransform>,
    enabled: bool,
}

impl Smoother {
    /// 新建；`lag_weight` 会被夹到 `0.0..1.0`。
    pub fn new(lag_weight: f32) -> Self {
        Self {
            lag_weight: lag_weight.clamp(0.0, 0.999),
            lerp: None,
            enabled: true,
        }
    }

    /// 清空平滑状态（下一帧不插值）。
    pub fn reset(&mut self) {
        self.lerp = None;
    }

    /// 与上一帧结果做指数插值，返回本帧实际位姿。
    pub fn smooth(&mut self, new_tfm: &LookTransform) -> LookTransform {
        debug_assert!(self.lag_weight < 1.0);
        let old = self.lerp.unwrap_or(*new_tfm);
        let lead = 1.0 - self.lag_weight;
        let lerp = LookTransform {
            eye: old.eye * self.lag_weight + new_tfm.eye * lead,
            target: old.target * self.lag_weight + new_tfm.target * lead,
            up: new_tfm.up,
        };
        self.lerp = Some(lerp);
        lerp
    }
}

/// 绕目标点环绕的相机控制器（状态，不含输入）。
#[derive(Component, Debug, Clone, Copy)]
pub struct OrbitCamera {
    /// 绕 Y 轴的方位角（弧度）。
    pub yaw: f32,
    /// 俯仰角（弧度），`0` 为水平、正值在目标上方。
    pub pitch: f32,
    /// 到目标点的距离。
    pub dist: f32,
    /// 注视点。
    pub target: Vec3,
    /// 初始位姿，供「复位」使用。
    pub home: (f32, f32, f32, Vec3),
    /// 旋转灵敏度（弧度/像素）。
    pub rotate_sensitivity: f32,
}

/// 俯仰角夹取范围：避免视线与 `Vec3::Y` 平行（look-at 无解）。
const PITCH_MIN: f32 = 0.05;
const PITCH_MAX: f32 = 1.5;

/// 距离夹取范围。
const DIST_MIN: f32 = 60.0;
const DIST_MAX: f32 = 20_000.0;

impl OrbitCamera {
    /// 以初始位姿新建，并把它记为「复位位姿」。
    pub fn new(yaw: f32, pitch: f32, dist: f32, target: Vec3) -> Self {
        Self {
            yaw,
            pitch: pitch.clamp(PITCH_MIN, PITCH_MAX),
            dist: dist.clamp(DIST_MIN, DIST_MAX),
            target,
            home: (yaw, pitch, dist, target),
            rotate_sensitivity: 0.005,
        }
    }

    /// 当前单位视线方向（眼睛相对目标的偏移方向）。
    pub fn unit_vector(&self) -> Vec3 {
        Vec3::new(
            self.pitch.cos() * self.yaw.sin(),
            self.pitch.sin(),
            self.pitch.cos() * self.yaw.cos(),
        )
    }

    /// 当前眼睛位置。
    pub fn eye(&self) -> Vec3 {
        self.target + self.unit_vector() * self.dist
    }

    /// 回到初始位姿。
    pub fn reset(&mut self) {
        let (yaw, pitch, dist, target) = self.home;
        self.yaw = yaw;
        self.pitch = pitch.clamp(PITCH_MIN, PITCH_MAX);
        self.dist = dist.clamp(DIST_MIN, DIST_MAX);
        self.target = target;
    }

    fn rotate(&mut self, dx: f32, dy: f32) {
        // 鼠标右移 → 视角右转；鼠标上移（dy<0）→ 抬头。
        self.yaw -= dx * self.rotate_sensitivity;
        self.pitch = (self.pitch + dy * self.rotate_sensitivity).clamp(PITCH_MIN, PITCH_MAX);
    }

    fn zoom(&mut self, notches: f32) {
        let factor = 0.9_f32.powf(notches);
        self.dist = (self.dist * factor).clamp(DIST_MIN, DIST_MAX);
    }
}

/// 控制器插件：输入 → 控制器 → `LookTransform` → 平滑 → `Transform`。
pub struct OrbitCameraPlugin;

impl Plugin for OrbitCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (orbit_input, drive_look_transform, look_transform_system).chain(),
        );
    }
}

/// 输入：鼠标左键拖动环绕、滚轮缩放、Q/E 旋转、Z/X 俯仰、F 复位。
fn orbit_input(
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut cameras: Query<(&mut OrbitCamera, &mut Smoother)>,
) {
    // 键盘步进量（弧度/帧）：按键是离散事件，直接给固定增量。
    let key_yaw = (keys.just_pressed(KeyCode::KeyQ) as i8 - keys.just_pressed(KeyCode::KeyE) as i8)
        as f32
        * 0.15;
    // X 抬高俯仰（相机升高、更俯视），Z 降低俯仰。
    let key_pitch = (keys.just_pressed(KeyCode::KeyX) as i8
        - keys.just_pressed(KeyCode::KeyZ) as i8) as f32
        * 0.10;
    let reset = keys.just_pressed(KeyCode::KeyF);
    let drag = buttons.pressed(MouseButton::Left);
    let notches: f32 = wheel.read().map(|ev| ev.y.signum()).sum();

    for (mut cam, mut smoother) in cameras.iter_mut() {
        if drag {
            cam.rotate(motion.delta.x, motion.delta.y);
        }
        if key_yaw != 0.0 || key_pitch != 0.0 {
            cam.yaw += key_yaw;
            cam.pitch = (cam.pitch + key_pitch).clamp(PITCH_MIN, PITCH_MAX);
        }
        if notches != 0.0 {
            cam.zoom(notches);
        }
        if reset {
            cam.reset();
            smoother.reset();
        }
    }
}

/// 控制器状态 → 目标注视位姿。
fn drive_look_transform(mut cameras: Query<(&OrbitCamera, &mut LookTransform)>) {
    for (cam, mut look) in cameras.iter_mut() {
        look.eye = cam.eye();
        look.target = cam.target;
        look.up = Vec3::Y;
    }
}

/// 目标注视位姿 → 平滑 → 场景 `Transform`。
fn look_transform_system(mut cameras: Query<(&LookTransform, &mut Transform, &mut Smoother)>) {
    for (look, mut transform, mut smoother) in cameras.iter_mut() {
        if smoother.enabled {
            *transform = smoother.smooth(look).into();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cam() -> OrbitCamera {
        OrbitCamera::new(0.0, 0.5, 100.0, Vec3::ZERO)
    }

    /// 正 pitch = 相机在目标上方（俯视）；这正是踩过「视角反了」的那个约定。
    #[test]
    fn positive_pitch_puts_camera_above_target() {
        let c = cam();
        assert!(c.eye().y > c.target.y);
        assert!(c.unit_vector().y > 0.0);
    }

    /// 鼠标右移 → 视角右转（yaw 减）。
    #[test]
    fn mouse_right_turns_view_right() {
        let mut c = cam();
        let y0 = c.yaw;
        c.rotate(10.0, 0.0);
        assert!(c.yaw < y0, "yaw 应随右移减小，实际 {y0} -> {}", c.yaw);
    }

    /// 鼠标上移（dy < 0）→ 抬头（pitch 减小）。
    #[test]
    fn mouse_up_looks_up() {
        let mut c = cam();
        let p0 = c.pitch;
        c.rotate(0.0, -10.0);
        assert!(c.pitch < p0, "pitch 应随上移减小，实际 {p0} -> {}", c.pitch);
    }

    /// 俯仰被夹在极点之外，避免视线与 `Vec3::Y` 平行。
    #[test]
    fn pitch_clamped_away_from_poles() {
        let mut c = cam();
        c.rotate(0.0, -100_000.0);
        assert!(c.pitch >= PITCH_MIN);
        c.rotate(0.0, 100_000.0);
        assert!(c.pitch <= PITCH_MAX);
        assert!(c.pitch < std::f32::consts::FRAC_PI_2);
    }

    /// 距离被夹在合法区间内。
    #[test]
    fn distance_clamped() {
        let mut c = cam();
        c.zoom(1000.0);
        assert!(c.dist >= DIST_MIN);
        c.zoom(-1000.0);
        assert!(c.dist <= DIST_MAX);
    }

    /// 复位回到初始位姿。
    #[test]
    fn reset_restores_home() {
        let mut c = cam();
        let home = c.home;
        c.rotate(50.0, 20.0);
        c.zoom(3.0);
        c.target = Vec3::splat(9.0);
        c.reset();
        assert_eq!((c.yaw, c.pitch, c.dist, c.target), home);
    }

    /// 平滑器首帧直接到位（不从原点飞），之后才产生滞后。
    #[test]
    fn smoother_jumps_on_first_frame_then_lags() {
        let mut s = Smoother::new(0.6);
        let a = LookTransform::new(Vec3::new(0.0, 0.0, 10.0), Vec3::ZERO, Vec3::Y);
        assert_eq!(s.smooth(&a), a);
        let b = LookTransform::new(Vec3::new(0.0, 0.0, 20.0), Vec3::ZERO, Vec3::Y);
        let mid = s.smooth(&b);
        assert!(mid.eye.z > a.eye.z && mid.eye.z < b.eye.z);
        // 反复调用会收敛到目标
        let mut last = mid;
        for _ in 0..200 {
            last = s.smooth(&b);
        }
        assert!((last.eye.z - b.eye.z).abs() < 1e-3);
    }
}
