# trapla-viz

`trapla` 的可视化前端：把 `map.csv` 的高度场、`path_result.csv` 的落足点序列和
`trapla plan --metrics` 的指标画在一起，并且能被无头自检与截图两种非交互方式验证。

- 引擎：**Bevy 0.20.0**（当前最新稳定版）。选它的理由：全 Rust，几何沿用与 `trapla-core`
  同源的 `glam`（`trapla-core/src/geom.rs` 只准用 f64 变体），且自带截图、UI 文字与 gizmo 折线，
  不需要额外渲染栈。
- 依赖 Bevy / bevy_egui / clap / serde_json，外加 `trapla-lab` 的 path 依赖（只用于沙盒场景库）。
  高度场与落足点 CSV 的读取口径仍在本 crate 内实现（`src/data.rs`），**不依赖** `trapla-io`：
  可视化前端的自检不被其它单元的进度阻塞。
- Bevy 开两个非默认 feature：`system_font_discovery`（HUD 中文字体走系统字体）与
  `complex_script_segmentation`（消掉 debug 下 CJK 文本分段的 ICU4X 告警，见「日志噪声」）。

## 构建与运行

本 crate 在 `crates/*` glob 下，Bevy 的编译会长时间占住 workspace 的 target 目录。
**所有 cargo 命令都必须带独立 target 目录**（下同）：

```bash
CARGO_TARGET_DIR=$HOME/.cache/trapla-viz-target cargo build -p trapla-viz
```

四种入口。输入路径都用显式参数、不靠当前工作目录猜；不给 `--map`/`--path`/`--scenario`
时交互模式进沙盒菜单（见「沙盒模式」）：

```bash
BIN=$HOME/.cache/trapla-viz-target/debug/trapla-viz

# 1) 无头自检：加载 + 构建场景 + 打印摘要，退出 0，不创建窗口
$BIN --map data/csv/map.csv --path data/csv/path_result.csv --headless-selftest

# 2) 截图：开窗渲染 N 帧后落盘 PNG，然后退出
$BIN --map data/csv/map.csv --path data/csv/path_result.csv \
     --screenshot /tmp/viz.png --frames 60

# 3) 交互（默认）：窗口 + 回放 + 相机
$BIN --map data/csv/map.csv --path data/csv/path_result.csv

# 4) 沙盒：不给输入进菜单选内置场景；或直接指定场景（可配合截图看地形）
$BIN
$BIN --scenario bumpy --screenshot /tmp/viz_bumpy.png --frames 60
```

按键表在 `--help` 末尾（`Space` 播放/暂停、`←/→` 单步、`↑/↓` 变速、`R` 重头、
鼠标左键拖动环绕、滚轮缩放、`Q/E` 旋转、`Z/X` 俯仰、`F` 复位；`Esc` 在显式输入下退出、
从沙盒菜单进入时回主菜单，主菜单里 `Esc` 退出程序）。

## 输入

| 参数 | 格式 | 说明 |
|---|---|---|
| `--map <csv>` | 无表头纯数值，1000 行 × 3000 列，值域 0~255 | `x` 是行、`y` 是列；与 `--path` 必须同时给 |
| `--path <csv>` | 表头 `x,y`，顺序足中心点 | 偶数下标 = 左脚，奇数下标 = 右脚；与 `--map` 必须同时给 |
| `--metrics <json>` | `trapla plan --metrics` 的 JSON | 可省略；字段缺失只影响该行 |
| `--goal <x,y>` | 目标点 | 省略时用路径最后一个点作目标标记 |
| `--stride <N>` | 地形降采样步长 | 省略时自动取满足顶点数 ≤ 100000 的最小步长 |
| `--scenario <id>` | `trapla-lab` 内置场景 id | 代替 `--map`/`--path`；尺寸固定 200×200 |
| `--exaggeration <倍>` | 垂直尺度：1 cm 高度 = 多少世界单位 | 默认 `1.0` = 真尺度；越大起伏越夸张，范围 0.01–200 |
| `--foot-scale <倍>` | 足长方体的**绘制**放大倍数 | 默认 6；只影响画面，不改足的真实尺寸与点位 |
| `--foot-thickness <cm>` | 足长方体厚度（真实尺寸） | 默认 1.5（3×5×1.5 cm 的足） |
| `--base-depth <cm>` | 地面底座厚度（地形最低处往下） | 默认 60；`0` = 不加底座 |
| `--mesh-metrics` | 开关 | 无头自检时**额外**打印两行斑点指标；默认不打印，V1 输出因此逐行不变 |
| `--menu` | 开关 | 启动先进主菜单；交互模式不给输入时默认如此 |

降采样后的世界坐标：`X = 列 - (cols-1)/2`、`Z = 行 - (rows-1)/2`、
`Y = 最近邻高度 × exaggeration`（默认 1.0，即 1 cm = 1 世界单位；历史默认曾是 0.05）。

指标 JSON 字段映射：`steps` / `total_length` / `goal_error` / `proper_crossings` /
`same_foot_turn`（取中位数）/ `lateral_separation`（取最小与中位），口径见 `task-11`。
文件不存在、JSON 损坏、单个字段缺失都只在面板上显示「未提供」，不影响退出码。

## 沙盒模式

交互模式**不给** `--map`/`--path`/`--scenario` 时，启动进入主菜单（`GamePhase::MainMenu`），
从 `trapla-lab` 的内置场景清单里选一个（200×200，8 个场景，含 `flat`/`slope`/`step`/`bumpy`/
`corridor`/`obstacles`/`fault`/`oob`），`Start`（或 `Enter`）后由 app 生成地图并进入 `Playing`。

- **生成路径与文件输入同源**：场景先经 `trapla-lab` 的 `scenario::write_map_csv` 落成与
  `map.csv` 同格式的临时文件，再用同一个读取函数读回，不存在两套地形口径。
- **加载发生在 `OnEnter(Playing)`**：菜单帧不读盘、不建网格；`--map`/`--path` / `--scenario`
  直接进入时同一套加载代码走一遍。
- **菜单的选项面是资源**（`MenuOptions`，从 `scenario::builtin` 构建），UI 代码里没有硬编码清单；
  菜单写选择（`MenuSelection`）、`OnEnter` 消费后决定加载哪个场景。
- **截图 / 无头自检默认跳过菜单**（直接进 `Playing`），所以截图判据与引入菜单前可比；
  `--menu` 是显式反向开关：带上它截图，画面就是菜单本身（`--menu` 下 `Playing` 从未进入）。
- **`Esc` 的语义按进入方式分**：从菜单进入 → 回主菜单（世界盒循环）；显式输入 → 退出程序
  （保持加菜单之前的行为）。
- **菜单文案是 ASCII**：`bevy_egui` 的默认字体不含 CJK，本批不搬字体资产（思源宋体 24 MB，
  且游玩中的中文 HUD 已由系统字体渲染正常），所以菜单用英文短标签，避免豆腐块。
- 沙盒场景**没有规划路径**：`Playing` 里只画起点（蓝）/ 目标点（红）与地形，回放计数为 `0/0`。

地形几何确实随场景改变（`--scenario flat|bumpy|corridor` 的栅格指纹与网格不同），但材质仍是
单一顶点色、高度缩放 0.05，200×200 场景的起伏在世界单位里不到 1，**看起来仍是一片平面** ——
观感改造属于下一批（地形着色 / 足姿表达 / 约束可视化）。

## 视觉元素说明

每个元素都是**输入的纯函数**（不含时间、帧数、随机数），所以截图可以逐字节复现。

| 元素 | 实现 | 关键约定 |
|---|---|---|
| 垂直尺度 | `--exaggeration`（默认 1.0 = 真尺度） | `map.csv` 峰谷差约 96 cm，历史默认 0.05 把它压成 4.8 世界单位、而水平跨度 3000 → 几何上是一张纸；1.0 让起伏可见。`-p trapla-viz` 的单测断言它只改世界 Y，不改顶点数 |
| 地形色带 | `scene::band_color`：8 段 hypsometric 色带 + 低处拉伸（`t^0.35`） | 按**数据实际 min/max** 归一化（不是写死 0–255）；`map.csv` 只在 27–123 之间，线性归一化会让视野里全落在绿色段 |
| 显示网格降采样 | `scene::display_grid`：**窗口均值**（每个采样点取它覆盖的 `stride²` 原始格的平均） | 最近邻会把原图高频混叠成地形上的一层斑点（`--nearest-sampling` 是隐藏的对照开关）；两种口径的顶点 / 三角形数完全相同，V1 打印不变 |
| hillshade | `scene::hillshade_illumination`：坡度用显示网格的**厘米高度**中心差分（水平间距 = `stride` 厘米）× 视觉放大系数 | 不用被 `HEIGHT_SCALE` 压过的世界几何法线，也不用 1 格梯度（顶点间距 6 格时那样会把高频放大成斑点） |
| 实体长方体足 | `Cuboid`（长 5 × 宽 3 × 厚 `--foot-thickness` 厘米），按 `foot_poses` 的 `rz` 绕 Y 定向，再按显示表面的法线贴上坡面 | 尺寸是真实足尺寸 × `--foot-scale`（纯绘制放大）；左右脚高对比蓝 / 橙；白色刻线指示长轴方向；未揭示的压暗 |
| 步序 | 屏幕空间 egui 数字（每 5 步 + 最后一步）；足色随步序变亮 | Bevy 0.20 没有世界空间 3D 文字 → `Camera::world_to_viewport` 投影。**标注前先做可见性判断**：剔掉相机背后的点，再沿相机→足的线段在高度场里步进做遮挡判断，被山脊挡住的步不标 |
| 地面网格 | `Gizmos::grid_3d`，间距 10 cm，贴地形最低处 | 纯 gizmo，不进 `TerrainMesh` |
| 地面底座 | `--base-depth`（厘米，默认 60；0 = 关）：四圈侧壁 + 底盖，侧壁按边缘高度上色、越往下越暗 | 双面材质（`cull_mode: None`），不影响地形网格的顶点 / 三角形数 |
| HUD 色标 | 右下角色带 + 实际高度范围 + 当前垂直 / 足部倍数 | 色标取每段中点颜色，与顶点色同一份插值实现 |
| 后处理 | `DistanceFog` + `ContactShadows` + `ScreenSpaceAmbientOcclusion` + 固定 `Tonemapping` | 只开这四件；插件由 `PbrPlugin` 自带（再 `add_plugins` 会 panic）。SSAO 要求 `Msaa::Off`，所以屏幕空间抗锯齿关闭；补锯齿的 TAA 帧相关，按确定性要求排除 |

### 斑点：量化与前后对比

`--headless-selftest --mesh-metrics` 会额外打印两行（默认不打印，所以 V1 输出逐行不变）：

```
显示网格法线: 相邻顶点夹角中位数 = 25.4604°（降采样=窗口均值）
显示网格斑纹: 二阶差分中位数 = 0.3333 cm（降采样=窗口均值）
```

同一命令加 `--nearest-sampling`（隐藏，改动前的口径）得到对照：

```
显示网格法线: 相邻顶点夹角中位数 = 33.6901°（降采样=最近邻）
显示网格斑纹: 二阶差分中位数 = 1.0000 cm（降采样=最近邻）
```

两条都是 `map.csv` 实测。法线夹角被宏观起伏主导，只降 24%；二阶差分（高频能量）是斑点的直接量化，
**降 67%**。这两个数已用一份独立的 Python 探针（按同一定义重算采样网格与法线）交叉核对，逐位一致。

足朝向的规则**只有一个家**：`trapla-lab/src/validate.rs` 的 `foot_poses`（校验器与可视化共用，
`validate_path` 也从它取 `rz`）。`--raw-foot-heading`（默认隐藏）把约定换成 `raw`（步向），
只用于负向对照 —— 它画出来的长方体朝向与默认的 `smoothed` 明显不同，证明没有随手拿步向当朝向。

## 相机

`src/orbit_camera.rs` 把相机拆成两层：控制器（`OrbitCamera`）只写 `LookTransform`，
`Smoother` 做指数平滑，同步系统再把它写进场景 `Transform`。角度约定：`pitch` 为正 =
相机在目标上方（俯视）；鼠标右移 = 视角右转，鼠标上移 = 抬头。

## 日志噪声

debug 构建下中文 `Text` 曾刷 `ICU4X data error: No segmentation model for complex script:
Chinese/Japanese`（debug 下 `--screenshot --frames 60` 一次 403–408 行）。根因是 parley 未开
`complex-scripts`，中日分词模型没编进二进制；Bevy 的 `complex_script_segmentation` feature
会把它带上（`bevy_text` → `parley/complex-scripts` → 载入 cjdict），本 crate 已开启，
**同一命令的该告警计数从 403/408 降到 0**。`icu_provider` 在 debug 下走裸 `eprintln!` shim，
绕过 Bevy tracing 与 `RUST_LOG`，所以只能靠 feature 消除，压不掉也不影响渲染。

## 确定性

截图模式是**输入的纯函数**，同一条命令重复跑 PNG 的 sha256 相同（实测三次一致），
改 `--frames` 也不改变成像：

- 回放**一次性揭示整条路径**（`Playback::fully_revealed`）。之前按墙钟逐帧揭示点数，
  同一条命令两次会得到 2357393 / 2357342 两种字节数 —— 那与墙钟有关，不能当判据。
- 截图模式打开 `RenderPlugin::synchronous_pipeline_compilation`：Bevy 默认异步编译渲染管线，
  若在第 N 帧就截，网格可能还没画出来（实测同一命令在不同管线缓存状态下会得到只有清屏色
  的图）。交互模式保持异步编译，启动更快。
- 帧相关效果一律不用：TAA、AutoExposure、MotionBlur、`FogVolume.jitter > 0` 都不在本 crate 里。
- SSAO 要求 `Msaa::Off`（0.20 实测：MSAA 非 Off 时每帧刷 ERROR 且效果失效），因此没有 MSAA。

窗口分辨率取窗口系统给出的实际值，截图 PNG 尺寸即为该窗口的物理像素尺寸。

## 无头自检打印什么

地图形状 → 降采样步长与顶点算式 → `Mesh::count_vertices()` 实测 → 路径点数与左右脚点数
（附 `ceil(n/2)` / `floor(n/2)` 对照）→ 目标点 → 指标值或「未提供」。退出码固定为 0。
