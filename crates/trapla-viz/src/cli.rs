//! 命令行参数面。
//!
//! 所有输入路径都由调用方显式给出，不依赖当前工作目录猜测；一个路径都不给时
//! 交互模式进沙盒菜单（场景由 `trapla-lab` 内置清单生成）。

use std::path::PathBuf;

use clap::Parser;

use crate::scene::{DEFAULT_EXAGGERATION, EXAGGERATION_MAX, EXAGGERATION_MIN};

/// 足矩形默认绘制放大倍数（只影响画面）。
pub const DEFAULT_FOOT_SCALE: f32 = 6.0;

/// 足长方体默认厚度（厘米，**真实尺寸**；绘制放大由 `--foot-scale` 负责）。
pub const DEFAULT_FOOT_THICKNESS: f32 = 1.5;

/// 地面底座默认厚度（厘米，地形最低处往下）。
pub const DEFAULT_BASE_DEPTH: f32 = 60.0;

/// 底座厚度上限（厘米）。
pub const MAX_BASE_DEPTH: f32 = 10_000.0;

/// 足矩形绘制放大倍数上限（防止把整张地图糊住）。
pub const MAX_FOOT_SCALE: f32 = 200.0;

/// `--help` 末尾附上的按键表。
///
/// 与 [`crate::viz`] 里实际的按键处理一一对应；改按键必须同时改这里。
pub const CONTROLS_HELP: &str = "\
回放按键:
  Space           播放 / 暂停
  Right 或 .      单步前进
  Left  或 ,      单步后退
  Up  或 =        加速（x2，上限 8x）
  Down 或 -       减速（x0.5，下限 0.25x）
  R               重头开始
相机操作:
  鼠标左键拖动     环绕（orbit）
  鼠标滚轮         缩放（zoom）
  Q / E           绕竖直轴左 / 右旋转
  Z / X           降低 / 抬高俯仰角
  F               重置相机
沙盒菜单（交互模式未显式给 --map/--path 时）:
  鼠标点击        选择内置场景
  Enter           开始
  Esc             退出程序
退出:
  Esc             回主菜单（从菜单进入沙盒时）或退出程序";

/// 可视化前端的运行模式。
///
/// 用枚举而不是若干 `bool`，避免「同时开两个模式」这种非法组合被构造出来。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Mode {
    /// 只加载数据 + 构建场景 + 打印摘要，不创建窗口。
    HeadlessSelftest,
    /// 创建窗口渲染若干帧后截图落盘，然后退出。
    Screenshot(PathBuf),
    /// 默认交互模式。
    Interactive,
}

/// `trapla-viz` 的命令行参数。
#[derive(Debug, Parser)]
#[command(
    name = "trapla-viz",
    version,
    about = "trapla 可视化：3D 地形 + 落足点回放 + 指标面板（Bevy）",
    after_help = CONTROLS_HELP
)]
pub struct Cli {
    /// 地图 CSV（无表头纯数值；x=行、y=列、值域 0~255）。与 --path 同时给，或用 --scenario
    #[arg(long, value_name = "csv")]
    pub map: Option<PathBuf>,

    /// 落足点序列 CSV（表头 x,y；偶数下标=左脚，奇数下标=右脚）。与 --map 同时给
    #[arg(long, value_name = "csv")]
    pub path: Option<PathBuf>,

    /// 指标 JSON（`trapla plan --metrics` 产出）；缺省或字段缺失时面板显示「未提供」
    #[arg(long, value_name = "json")]
    pub metrics: Option<PathBuf>,

    /// 目标点，格式 x,y；缺省时用路径最后一个点作为目标标记
    #[arg(long, value_name = "x,y")]
    pub goal: Option<Xy>,

    /// 地形降采样步长；缺省（不传）时自动取满足顶点数 <= 100000 的最小步长
    #[arg(long, value_name = "N")]
    pub stride: Option<usize>,

    /// 用 trapla-lab 内置场景代替 --map/--path（id 见菜单或 trapla-lab 内置清单）
    #[arg(long, value_name = "id")]
    pub scenario: Option<String>,

    /// 垂直尺度：1 cm 高度 = 多少世界单位；1.0 = 真尺度（默认），越大越夸张
    #[arg(long, value_name = "倍", default_value_t = DEFAULT_EXAGGERATION)]
    pub exaggeration: f32,

    /// 足矩形的绘制放大倍数：只影响画面，不改足的真实尺寸与点位
    #[arg(long, value_name = "倍", default_value_t = DEFAULT_FOOT_SCALE)]
    pub foot_scale: f32,

    /// 足长方体的厚度（厘米，真实尺寸；绘制放大由 --foot-scale 负责）
    #[arg(long, value_name = "cm", default_value_t = DEFAULT_FOOT_THICKNESS)]
    pub foot_thickness: f32,

    /// 地面底座厚度（厘米，地形最低处往下）；0 = 不加底座
    #[arg(long, value_name = "cm", default_value_t = DEFAULT_BASE_DEPTH)]
    pub base_depth: f32,

    /// 无头自检时额外打印「显示网格相邻顶点法线夹角中位数」（斑点的量化指标）
    #[arg(long)]
    pub mesh_metrics: bool,

    /// 负向对照：显示网格用最近邻降采样（改动前的口径，会把高频混叠成斑点）
    #[arg(long, hide = true)]
    pub nearest_sampling: bool,

    /// 负向对照：用 raw（步向）而不是 smoothed（提交语义）重建足朝向；正常不该用
    #[arg(long, hide = true)]
    pub raw_foot_heading: bool,

    /// 启动先进入主菜单；截图模式下用它验证「跳过菜单」开关确实生效
    #[arg(long)]
    pub menu: bool,

    /// 无头自检：加载全部输入、构建场景、打印摘要后退出，不创建窗口
    #[arg(long)]
    pub headless_selftest: bool,

    /// 截图模式：创建窗口渲染 N 帧后把 PNG 存到该路径
    #[arg(long, value_name = "png")]
    pub screenshot: Option<PathBuf>,

    /// 截图模式渲染的帧数
    #[arg(long, value_name = "N", default_value_t = 60)]
    pub frames: u32,
}

impl Cli {
    /// 由参数推导运行模式；互斥选项同时给出时返回错误而不是静默取其一。
    pub fn mode(&self) -> Result<Mode, String> {
        match (self.headless_selftest, &self.screenshot) {
            (true, Some(_)) => Err(
                "`--headless-selftest` 与 `--screenshot` 互斥：前者不建窗口，后者要建窗口"
                    .to_string(),
            ),
            (true, None) => Ok(Mode::HeadlessSelftest),
            (false, Some(p)) => {
                if self.frames == 0 {
                    return Err("`--frames` 必须 >= 1".to_string());
                }
                Ok(Mode::Screenshot(p.clone()))
            }
            (false, None) => Ok(Mode::Interactive),
        }
    }

    /// 是否显式给了场景来源（文件或内置场景）。
    pub fn has_explicit_inputs(&self) -> bool {
        self.map.is_some() || self.path.is_some() || self.scenario.is_some()
    }

    /// 垂直尺度：必须有限且落在允许区间内（fail-closed：0 或负数会让地形退化成平面）。
    pub fn exaggeration(&self) -> Result<f32, String> {
        let v = self.exaggeration;
        if !v.is_finite() || !(EXAGGERATION_MIN..=EXAGGERATION_MAX).contains(&v) {
            return Err(format!(
                "`--exaggeration` 必须在 {EXAGGERATION_MIN}..={EXAGGERATION_MAX} 之间，收到 {v}"
            ));
        }
        Ok(v)
    }

    /// 足部绘制放大倍数：必须有限且为正、不超过 [`MAX_FOOT_SCALE`]。
    pub fn foot_scale(&self) -> Result<f32, String> {
        let v = self.foot_scale;
        if !v.is_finite() || v <= 0.0 || v > MAX_FOOT_SCALE {
            return Err(format!(
                "`--foot-scale` 必须在 0..={MAX_FOOT_SCALE} 之间且为正，收到 {v}"
            ));
        }
        Ok(v)
    }

    /// 足长方体厚度（厘米）：有限且为正。
    pub fn foot_thickness(&self) -> Result<f32, String> {
        let v = self.foot_thickness;
        if !v.is_finite() || v <= 0.0 || v > 100.0 {
            return Err(format!(
                "`--foot-thickness` 必须在 0..=100 之间且为正，收到 {v}"
            ));
        }
        Ok(v)
    }

    /// 底座厚度（厘米）：有限且非负、不超过 [`MAX_BASE_DEPTH`]。
    pub fn base_depth(&self) -> Result<f32, String> {
        let v = self.base_depth;
        if !v.is_finite() || v < 0.0 || v > MAX_BASE_DEPTH {
            return Err(format!(
                "`--base-depth` 必须在 0..={MAX_BASE_DEPTH} 之间，收到 {v}"
            ));
        }
        Ok(v)
    }
}

/// 命令行上的 `x,y` 点。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Xy {
    /// 行坐标。
    pub x: f64,
    /// 列坐标。
    pub y: f64,
}

impl std::str::FromStr for Xy {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (x, y) = s
            .split_once(',')
            .ok_or_else(|| format!("期望 `x,y` 格式，收到 `{s}`"))?;
        let x = x
            .trim()
            .parse::<f64>()
            .map_err(|e| format!("x 解析失败: {e}"))?;
        let y = y
            .trim()
            .parse::<f64>()
            .map_err(|e| format!("y 解析失败: {e}"))?;
        Ok(Self { x, y })
    }
}
