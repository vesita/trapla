# 双足机器人在线落足点规划：项目说明

本文写**系统需求、对象与约束、算法设计**。Rust 模块划分、数据流与有意分歧索引见
`doc/architecture.md`；命令与测试怎么跑见 `readme.md`；问题的建模推导见 `doc/problem_flow.md`。

## 1. 项目概述

双足机器人在不平坦路面上交替迈步时，落足点受步长、足平面姿态、双足间距、转向角等因素制约。
本项目的目标：给定地形图与起点、终点，**在线**规划出一条满足全部约束的落足点序列。

## 2. 系统需求

### 2.1 输入条件

| 输入 | 取值 / 形式 |
|---|---|
| 三维地形图 | 灰度高度图导出的 CSV（无表头，x=行、y=列，值域 0–255 cm），`data/csv/map.csv` 为 1000×3000 |
| 足尺寸 | 3×5 cm（半宽 1.5、半长 2.5），采样步长 0.5 |
| 步长 | 严格小于 40 cm |
| 双足间距 | 2–10 cm（几何口径见 §3.2.4） |
| 最大转向角 | 严格小于 ±75° |
| 足平面法线与重力夹角 | 严格小于 ±20° |
| 起点 / 终点 | 二维投影坐标 `(x, y)`，须落在地图内（越界报输入错 `exit=1`） |
| 随机种子 | `--seed`，决定缩放图 A* 的邻居洗牌顺序 |

观测范围（运动方向前方 200×200）与「实际落足点偏差」属于在线运行时的假设，
规划器按「已知完整地形」离线规划，不建模观测噪声。

### 2.2 输出要求

1. **双足落足点序列**：足中心位置坐标（`--out` 的 CSV，表头 `x,y`，6 位小数），偶数下标为左脚。
2. **猫步指标 JSON**（`--metrics`）：步数、总长、终点误差、跨距/转角/间距序列、折线交叉数等
   （定义见 `crates/trapla-core/src/metrics.rs`，字段即口径）。
3. **可视化回放**：`trapla-viz` 在地形上回放落足点与指标面板。
4. **可复现性**：同一 `--seed` 两次运行的输出逐字节相同。

## 3. 系统设计

### 3.1 对象特征及约束

#### 3.1.1 双足机器人

- 足为 3×5 cm 的矩形；足位姿 = 中心点 `(x, y)` + 朝向 `rz`。
- 约束：步长 < 40；双足间距 2–10；转向角（新朝向与支撑脚朝向之差，**先归一到 (-π, π]**）< ±75°；
  足平面法线与重力夹角 < ±20°。
- 全部阈值的唯一事实源是 `crates/trapla-core/src/config.rs`。

#### 3.1.2 环境地形

- 离散高度栅格（CSV），格值即高度；`+inf` 记障碍。
- 地形约束三层：① 障碍不可通行；② 足覆盖区域必须全部在图内且拟合平面不陡于阈值（可站立）；
  ③ 缩放图上的陡峭度作为 A* 代价（`PENALTY_K`），不是硬断路。

#### 3.1.3 路径规划

- 输出是一条落足点序列（不是连续轨迹）。
- 规划分两层：缩放图上的粗路径 + 原图上的足步生成（§3.2.1）。

### 3.2 算法设计

#### 3.2.1 分层路径规划

1. **高层**：把地图按 `scale = 1/40` 降采样（窗口均值，含障碍则整格记障碍），在缩放图上跑 A*，
   得到引导点序列。缩放用 `Grid::scale_down`，区块还原用 `Grid::restore_block`（返回扫描用的角点闭区间）。
2. **中层**：把缩放区块映射回原图坐标，形成引导折线。
3. **底层**：`Robot::find_path` 逐步生成落足点：选摆动脚 → 生成候选（极坐标网格：半径
   `active_distance..max_stride`、角度相对支撑脚朝向 ±75°、步进 15°）→ `position_check` 过滤
   （步长/转向/间距/可站立）→ `will_feet_cross` 防交叉 → 按 `direction_determine` 的加权指向选一个候选。
   候选表为空时返回 `CoreError::NoPath`。

#### 3.2.2 A* 搜索算法

两个入口，启发式不同（各自与对应基线一致）：

| 入口 | 用途 | 启发式 | 代价 |
|---|---|---|---|
| `astar::scale_star` | 规划主路径（缩放图） | 欧氏距离 | 曼哈顿格代价 + 陡峭度 × `PENALTY_K` |
| `astar::a_star_search` | 单尺度搜索/测试 | 曼哈顿距离 | 邻格高度差（见 `Grid::cost`） |

邻居洗牌用确定性 `rng::Rng`（同 seed 可复现）；「不同 seed 可给出相同路径」是正常结果
（地形代价使最优解唯一时）。

#### 3.2.3 足部姿态计算

- **覆盖格**：足矩形按采样步长 0.5 扫描，四舍五入到整数格后去重（顺序为 `(x, y)` 升序）。
- **接触平面**：`geom::fit_plane_up` —— 贪心 **up-fit**（保证采样点不落在平面**上方**），
  **不是**最小二乘法；三点共线等退化情形返回 `DegeneratePlane`。
- **姿态评估**：`Plane::normal_angle()` 给出法线与竖直方向的夹角；`Foot::standable` 要求
  全部覆盖格在图内、无障碍，且夹角 < `MAX_NORMAL_ANGLE_RAD`。

#### 3.2.4 双足间距约束检查

几何口径（与实现一致）：

1. 取支撑脚**纵向中心线**（过支撑脚中心、沿支撑脚朝向的直线）；
2. 计算摆动脚新位置的四角到该直线的垂距，取最小值；
3. 实际间距 = 该最小垂距 − 支撑脚半宽（1.5）；
4. 判据：`spacing` 满足 `[min − 1e-3, max + 1e-3]` 的**闭区间**。

> 第 4 条的 1e-3 容差与闭区间来自实现对齐；它与旧文档「大于 2 且小于 10」的差别登记在
> `doc/known_divergences.md`。

## 4. 实现（Rust）

| 系统概念 | Rust 位置 |
|---|---|
| 阈值 | `crates/trapla-core/src/config.rs`（唯一事实源） |
| 几何/平面拟合 | `crates/trapla-core/src/geom.rs` |
| 栅格、降采样、区块还原 | `crates/trapla-core/src/grid.rs` |
| 地形（法向角、陡峭度） | `crates/trapla-core/src/terrain.rs` |
| A*、指向点、衰减 | `crates/trapla-core/src/astar.rs` |
| 足几何与滑移 | `crates/trapla-core/src/foot.rs` |
| 规划主循环 | `crates/trapla-core/src/robot.rs` |
| 指标 | `crates/trapla-core/src/metrics.rs` |
| CSV/IO 与错误分类 | `crates/trapla-io/src/csv.rs`、`error.rs` |
| CLI 与退出码 | `crates/trapla-cli/src/{main,plan,analyze}.rs` |

## 5. 代码结构

三个 crate 的职责、数据流与依赖方向见 `doc/architecture.md` §1–§2；顶层目录树见 `readme.md`。

## 6. 构建

```bash
cargo build --release
```

Rust 版本由 `rust-toolchain.toml` 固定。`trapla-viz` 依赖 Bevy，单独构建：
`cargo build --release -p trapla-viz`。

## 7. 运行与测试

```bash
# 规划
cargo run -q -p trapla-cli -- plan --map data/csv/map.csv --start 0,0 --goal 500,500 \
  --out /tmp/path_result.csv --seed 42 --metrics /tmp/metrics.json

# 分析已有路径
cargo run -q -p trapla-cli -- analyze --path /tmp/path_result.csv --goal 500,500

# 测试
cargo test --workspace --exclude trapla-viz
uv run pytest python/tests -q
```

参数、退出码与测试分层见 `readme.md`。
