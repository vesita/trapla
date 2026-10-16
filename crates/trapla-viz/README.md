# trapla-viz

`trapla` 的可视化前端：把 `map.csv` 的高度场、`path_result.csv` 的落足点序列和
`trapla plan --metrics` 的指标画在一起，并且能被无头自检与截图两种非交互方式验证。

- 引擎：**Bevy 0.20.0**（当前最新稳定版）。选它的理由：全 Rust，几何沿用与 `trapla-core`
  同源的 `glam`（`trapla-core/src/geom.rs` 只准用 f64 变体），且自带截图、UI 文字与 gizmo 折线，
  不需要额外渲染栈。
- 只依赖 Bevy / clap / serde_json，**不依赖** `trapla-core`、`trapla-io`：可视化前端的自检
  不被其它单元的进度阻塞（`trapla-io` 的 `read_matrix` / `read_xy` 目前是 `todo!()`），
  CSV 口径按 `data/csv/` 实测格式在本 crate 内实现。

## 构建与运行

本 crate 在 `crates/*` glob 下，Bevy 的编译会长时间占住 workspace 的 target 目录。
**所有 cargo 命令都必须带独立 target 目录**（下同）：

```bash
CARGO_TARGET_DIR=$HOME/.cache/trapla-viz-target cargo build -p trapla-viz
```

三种模式（`--map` / `--path` 必填，都用显式路径，不靠当前工作目录猜）：

```bash
BIN=$HOME/.cache/trapla-viz-target/debug/trapla-viz

# 1) 无头自检：加载 + 构建场景 + 打印摘要，退出 0，不创建窗口
$BIN --map data/csv/map.csv --path data/csv/path_result.csv --headless-selftest

# 2) 截图：开窗渲染 N 帧后落盘 PNG，然后退出
$BIN --map data/csv/map.csv --path data/csv/path_result.csv \
     --screenshot /tmp/viz.png --frames 60

# 3) 交互（默认）：窗口 + 回放 + 相机
$BIN --map data/csv/map.csv --path data/csv/path_result.csv
```

按键表在 `--help` 末尾（`Space` 播放/暂停、`←/→` 单步、`↑/↓` 变速、`R` 重头、
鼠标左键拖动环绕、滚轮缩放、`Q/E` 旋转、`Z/X` 俯仰、`F` 复位、`Esc` 退出）。

## 输入

| 参数 | 格式 | 说明 |
|---|---|---|
| `--map <csv>` | 无表头纯数值，1000 行 × 3000 列，值域 0~255 | `x` 是行、`y` 是列 |
| `--path <csv>` | 表头 `x,y`，顺序足中心点 | 偶数下标 = 左脚，奇数下标 = 右脚 |
| `--metrics <json>` | `trapla plan --metrics` 的 JSON | 可省略；字段缺失只影响该行 |
| `--goal <x,y>` | 目标点 | 省略时用路径最后一个点作目标标记 |
| `--stride <N>` | 地形降采样步长 | 省略时自动取满足顶点数 ≤ 100000 的最小步长 |

降采样后的世界坐标：`X = 列 - (cols-1)/2`、`Z = 行 - (rows-1)/2`、`Y = 最近邻高度 × 0.05`。

指标 JSON 字段映射：`steps` / `total_length` / `goal_error` / `proper_crossings` /
`same_foot_turn`（取中位数）/ `lateral_separation`（取最小与中位），口径见 `task-11`。
文件不存在、JSON 损坏、单个字段缺失都只在面板上显示「未提供」，不影响退出码。

## 相机

`src/orbit_camera.rs` 把相机拆成两层：控制器（`OrbitCamera`）只写 `LookTransform`，
`Smoother` 做指数平滑，同步系统再把它写进场景 `Transform`。角度约定：`pitch` 为正 =
相机在目标上方（俯视）；鼠标右移 = 视角右转，鼠标上移 = 抬头。

## 日志噪声

debug 构建下中文 `Text` 会刷 `ICU4X data error: No segmentation model for complex script:
Chinese/Japanese`：这条输出来自 `icu_provider` 在 debug 下的裸 `eprintln!` shim，
**绕过 Bevy tracing 与 `RUST_LOG`**，压不掉、也不影响渲染。需要干净输出就跑 release：

```bash
CARGO_TARGET_DIR=$HOME/.cache/trapla-viz-target cargo build -p trapla-viz --release
```

## 确定性

截图模式打开 `RenderPlugin::synchronous_pipeline_compilation`：Bevy 默认异步编译渲染管线，
若在第 N 帧就截，网格可能还没画出来（实测同一命令在不同管线缓存状态下会得到只有清屏色
的图）。交互模式保持异步编译，启动更快。

窗口分辨率取窗口系统给出的实际值，截图 PNG 尺寸即为该窗口的物理像素尺寸。

## 无头自检打印什么

地图形状 → 降采样步长与顶点算式 → `Mesh::count_vertices()` 实测 → 路径点数与左右脚点数
（附 `ceil(n/2)` / `floor(n/2)` 对照）→ 目标点 → 指标值或「未提供」。退出码固定为 0。
