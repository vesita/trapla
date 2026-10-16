//! 命令行参数面。
//!
//! 所有输入路径都由调用方显式给出，不依赖当前工作目录猜测。

use std::path::PathBuf;

use clap::Parser;

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
退出:
  Esc             退出";

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
    /// 地图 CSV（无表头纯数值；x=行、y=列、值域 0~255）
    #[arg(long, value_name = "csv")]
    pub map: PathBuf,

    /// 落足点序列 CSV（表头 x,y；偶数下标=左脚，奇数下标=右脚）
    #[arg(long, value_name = "csv")]
    pub path: PathBuf,

    /// 指标 JSON（`trapla plan --metrics` 产出）；缺省或字段缺失时面板显示「未提供」
    #[arg(long, value_name = "json")]
    pub metrics: Option<PathBuf>,

    /// 目标点，格式 x,y；缺省时用路径最后一个点作为目标标记
    #[arg(long, value_name = "x,y")]
    pub goal: Option<Xy>,

    /// 地形降采样步长；缺省（不传）时自动取满足顶点数 <= 100000 的最小步长
    #[arg(long, value_name = "N")]
    pub stride: Option<usize>,

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
