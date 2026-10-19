# Trapla — 双足机器人在线落足点规划（Rust）

给定一张三维地形图（灰度高度图导出的 CSV）、起点与终点，规划出一条满足机器人物理约束的
双足落足点序列，并输出点位 CSV 与猫步指标 JSON。

本仓是 Rust 重写版：C++ 原实现已按计划移除（见 `doc/baseline.md`），Rust 侧已通过函数级
golden 逐条对照与独立复核（见 `doc/port_verification.md`）。

## 目录

- [功能特性](#功能特性)
- [构建](#构建)
- [运行](#运行)
- [测试](#测试)
- [可视化](#可视化)
- [目录结构](#目录结构)
- [已知限制](#已知限制)
- [文档索引](#文档索引)

## 功能特性

- 分层规划：在缩放图上做 A*（曼哈顿启发 + 地形陡峭度代价），把引导点还原回原图，再按物理约束生成落足点序列。
- 约束检查（阈值全部来自 `crates/trapla-core/src/config.rs`，唯一事实源）：
  步长 < 40、双足间距 2–10（几何口径见 `doc/project.md`）、转向角 < ±75°、足平面法线与重力夹角 < ±20°。
- 确定性随机源：同一 `--seed` 的输出**逐字节可复现**（原 C++ 用 `std::random_device`，两次运行结果不同）。
- 猫步指标：跨足/同足转角、横向间距、折线交叉数、终点误差等，随 `--metrics` 落盘。
- Python 工具箱 `trapla-data`：生成地图、JPEG↔CSV、路径与引导点绘图（细节见 `python/README.md`）。
- 3D 可视化 `trapla-viz`：地形 + 落足点回放 + 指标面板（Bevy）。

## 构建

```bash
cargo build --release            # 默认成员：trapla-core / trapla-io / trapla-cli
cargo build --release -p trapla-viz   # 可视化前端（Bevy，单独构建；首次较慢）
```

Rust 版本由 `rust-toolchain.toml` 固定（edition 2024）。

## 运行

```bash
cargo run -q -p trapla-cli -- plan \
  --map data/csv/map.csv --start 0,0 --goal 500,500 \
  --out /tmp/path_result.csv --seed 42 --metrics /tmp/metrics.json
```

`trapla plan`：

| 参数 | 说明 |
|---|---|
| `--map <csv>` | 输入地图 CSV（**无表头**纯数值；x=行、y=列） |
| `--start <x,y>` | 起点（行,列） |
| `--goal <x,y>` | 终点（行,列） |
| `--out <csv>` | 输出点位 CSV（表头 `x,y`，坐标固定 6 位小数） |
| `--seed <u64>` | 随机种子（缺省 0）；相同 seed 输出逐字节相同 |
| `--metrics <json>` | 指标 JSON；`--repeat 1` 写对象，`N>1` 写长度 N 的数组（每项带 seed） |
| `--repeat <N>` | 重复规划 N 次，依次用 seed, seed+1, …, seed+N-1 |

`trapla analyze`（分析已有路径）：

```bash
cargo run -q -p trapla-cli -- analyze --path data/csv/path_result.csv --goal 500,500 --out /tmp/metrics.json
```

### 退出码（与 `--help` 同源）

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 1 | 输入文件不可读、格式错，**或坐标越出栅格边界**（参数错误同此码） |
| 2 | 规划失败（无路径） |
| 3 | 其它内部错误 |

> 越界坐标报 **1**（输入错）而不是 2：这条语义由 T9a 定下，实测 120 组合为 `exit=0 72 / exit=1 48 / exit=2 0`。

## 测试

```bash
cargo test --workspace --exclude trapla-viz                  # Rust 全部测试
cargo clippy --workspace --exclude trapla-viz --all-targets -- -D warnings
uv run pytest python/tests -q                                # Python 工具箱测试
```

Rust 测试分四层：

- `crates/trapla-core/src/**` 内的单元测试（阈值、几何原语、错误路径）；
- `crates/trapla-core/tests/geom_grid_golden.rs`、`terrain_astar_golden.rs`、`robot_golden.rs`：**golden 夹具对照**
  （22 个夹具，来源与用途见 `doc/architecture.md`）；
- `crates/trapla-core/tests/port_verification.rs`：T7 独立复核（不依赖作者测试的逐行对照 + 洗牌/seed 命题 + NoPath 分层）；
- `crates/trapla-io/tests/`、`crates/trapla-cli/tests/`：CSV 往返与 CLI 端到端。

## 算法测试台

无头算法测试台 `crates/trapla-lab/`（确定性场景、**独立**硬约束校验、批量跑分与回归门禁）的用法、
判据、场景清单与加算法的方式见 `doc/lab.md`。

## 可视化

```bash
cargo run -q -p trapla-viz -- --map data/csv/map.csv --path /tmp/path_result.csv \
  --metrics /tmp/metrics.json --goal 500,500

cargo run -q -p trapla-viz -- --map data/csv/map.csv --path /tmp/path_result.csv --headless-selftest
cargo run -q -p trapla-viz -- --map data/csv/map.csv --path /tmp/path_result.csv --screenshot /tmp/viz.png --frames 60
```

- `--headless-selftest`：不建窗口，加载全部输入 + 构建场景 + 打印摘要；无 `DISPLAY` 的 CI 用这个。
- `--screenshot <png>`：建窗口渲染 `--frames` 帧后截图落盘。两个选项**互斥**。
- 按键表见 `trapla-viz --help` 末尾。

## 目录结构

```
.
├── crates/
│   ├── trapla-core/         # 规划内核：几何/栅格/地形/A*/机器人/指标/config
│   │   └── tests/golden/    # 22 个 C++ 基线冻结夹具（数据）
│   ├── trapla-io/           # CSV 读写与 IO 错误分类
│   ├── trapla-cli/          # 命令行：plan / analyze / version
│   └── trapla-viz/          # Bevy 可视化（交互 / 无头自检 / 截图）
├── python/
│   ├── trapla_scripts/      # trapla-data 工具箱（gen-map / jpg2csv / csv2jpg / plot-*）
│   └── tests/               # pytest
├── data/                    # 输入数据（map.csv / map.jpg）与既有输出
├── doc/                     # 文档（见下）
├── scripts/sync/            # 跨设备同步未提交改动（与语言无关，见 doc/sync.md）
├── Cargo.toml / Cargo.lock / rust-toolchain.toml
└── pyproject.toml / uv.lock
```

## 已知限制

1. **走猫步尚未修好**：已做「足印分道 + 提交路径朝向解耦」，主判据 `proper_crossings` 目标 = 0、
   当前实测 = 4；横向间距 min 目标 ≥ 3.0、当前实测 0.82。判据与基线见 `doc/catwalk_diagnosis.md` §④、
   `doc/catwalk_design.md`；现状登记在 `doc/known_divergences.md` D4。
2. **对照口径**：与历史基线（C++）可比的是函数级 golden 与行为判据；逐点路径**不是**对照口径
   （随机源算法与消耗顺序不同）。
3. **边界语义有一处与文档不一致**：`satisfy_spacing` 实现用带 1e-3 容差的闭区间，
   文档写「大于 2 且小于 10」；移植期按实现对齐，分歧登记在 `doc/known_divergences.md`。
4. `trapla-viz` 需要图形环境；无 GPU/窗口时用 `--headless-selftest`。
5. 越界目标一律 `exit=1`，不会被裁剪到边界。

## 文档索引

| 文档 | 内容 |
|---|---|
| `doc/architecture.md` | Rust 架构、数据流、阈值唯一事实源、golden 夹具、**有意分歧索引**、猫步现状 |
| `doc/project.md` | 系统需求、对象与约束、算法设计（Rust 版） |
| `doc/problem_flow.md` | 问题分析：状态空间、约束建模、分层规划设计 |
| `doc/scripting.md` | Python 工具箱与同步脚本的**指针**（细节唯一家在 `python/README.md`） |
| `doc/baseline.md` | C++ 基线为何/如何冻结、已移除、在哪个 git 修订里复现 |
| `doc/known_divergences.md` | 有意分歧 D1–D5 与三处代码级修正的登记 |
| `doc/catwalk_diagnosis.md` | 走猫步的量化诊断与判据表（含历史基线数字） |
| `doc/catwalk_design.md` / `doc/catwalk_fix.md` | 猫步修复方案与实施记录 |
| `doc/nopath_diagnosis.md` | `exit=2` 的分层诊断（**当时快照**；现行口径见文首） |
| `doc/port_verification.md` | T7 独立移植复核报告（22 夹具逐行结论、性能主张复现、阻塞项） |
| `doc/sync.md` | `scripts/sync/` 的跨设备同步机制 |
| `python/README.md` | `trapla-data` 子命令、数据契约、依赖与测试 |
