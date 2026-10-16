# Trapla 架构（Rust 现状）

本文是当前实现的**唯一架构入口**：三个 crate 的职责、数据流、阈值的唯一事实源、golden 夹具的
来源与用途、以及**全部已登记的有意分歧**。测试怎么跑、命令怎么敲见 `readme.md`；
每个分歧的完整证据见 `doc/known_divergences.md`。

## 1 三个 crate 的职责

| crate | 职责 | 关键模块 |
|---|---|---|
| `trapla-core` | 规划内核：没有任何 IO/CLI 依赖，纯计算 | `config`（阈值唯一事实源）、`geom`（点/线/平面/拟合）、`grid`（栅格、邻域、代价、降采样、区块还原）、`terrain`（法向角、拟合平面、陡峭度）、`astar`（曼哈顿 A* + 缩放图 A* + 衰减/指向点）、`foot`（足几何、覆盖格、可站立、滑移）、`robot`（规划主循环）、`metrics`（猫步指标）、`rng`（确定性随机源）、`error`（错误分类） |
| `trapla-io` | CSV 读写与 IO 错误分类 | `csv`（矩阵/`x,y` 点位读写）、`IoError`（`Io`/`Format{line}`/`Shape`/`Core`） |
| `trapla-cli` | 命令行入口与退出码映射 | `plan`、`analyze`、`version`；退出码 0/1/2/3（见 `readme.md`） |
| `trapla-viz` | Bevy 可视化：3D 地形 + 落足点回放 + 指标面板 | `cli`（交互 / `--headless-selftest` / `--screenshot`）、`scene`、`metrics` |

`trapla-core` 的错误分类（`error.rs`）与边界上报分离：分类只回答「哪一种失败」
（`OutOfBounds` / `Obstacle` / `DegeneratePlane` / `NoPath` / `InvalidScale` / `GridShapeMismatch`），
退出码由 `trapla-cli` 映射。

## 2 数据流

```
data/csv/map.csv（无表头 1000×3000 数值）
        │  trapla plan --map --start x,y --goal x,y --out --seed [--metrics] [--repeat]
        ▼
   Grid ──scale_down(1/40)──► 缩放图 ──astar::scale_star（曼哈顿启发 + 陡峭度代价 × PENALTY_K）
        │                                        │ 引导点（还原到原图）
        │                                        ▼
        │                               Robot::find_path：选摆动脚 → 生成候选（步长/转向/间距/可站立）
        │                                        │ 每步：direction_determine 加权、will_feet_cross 防交叉
        ▼                                        ▼
 path_result.csv（表头 x,y，6 位小数）      metrics JSON（PathMetrics：步数/转角/间距/交叉数/终点误差…）
        │                                        │
        └──────────────► trapla-viz（--map --path --metrics）回放与面板
                         trapla analyze --path --goal [--out] 只重算指标
```

- `trapla plan` 的 `--start/--goal` 都是 `(x, y)`，**x 是行、y 是列**；越界直接 `exit=1`。
- 同 `--seed` 的输出逐字节可复现；`--repeat N` 用 `seed..seed+N-1` 各跑一次。
- 输入输出路径一律显式给出，不按可执行文件位置拼路径（原 C++ 的 `../data/csv/map.csv` 约定已废除）。

## 3 阈值的唯一事实源

全部物理阈值只写在 `crates/trapla-core/src/config.rs`；其它模块（含测试）一律引用它，不得写死字面量：

| 常量 | 值 | 含义 |
|---|---|---|
| `MAX_STRIDE_CM` | 40.0 | 最大步长（严格小于） |
| `MIN_SEPARATION_CM` / `MAX_SEPARATION_CM` | 2.0 / 10.0 | 双足间距区间 |
| `MAX_TURN_RAD` | 75° | 最大转向角（严格开区间，差角先归一化到 (-π, π]） |
| `MAX_NORMAL_ANGLE_RAD` | 20° | 足平面法线与重力夹角上限 |
| `SEPARATION_EPS` | 1e-3 | 间距比较容差（见 D-间距） |
| `ACTIVE_DISTANCE_RATIO` | 0.4 | 摆动脚朝向的激活距离 = 0.4 × 最大步长 |

`astar::PENALTY_K = 3.0`（陡峭度代价权重）是**算法参数**而非物理阈值，按其定义写在 `astar.rs`。

## 4 golden 夹具：来源与用途

- 位置：`crates/trapla-core/tests/golden/`，22 个文件（21 个 CSV + `golden_summary.txt`）。
- 来源：Rust 重写前在 C++ 原实现上冻结（把随机源换成固定种子后）导出的函数级 golden；
  C++ 实现与其 harness 保存在 git 修订 **`0038315`**，取法见 `doc/baseline.md`。
- 用途：三个 golden 测试文件按表头名逐行对照，容差 1e-9——
  `geom_grid_golden.rs`（几何/栅格）、`terrain_astar_golden.rs`（地形/A*/衰减）、`robot_golden.rs`（足/机器人约束）。
- **独立对照**：`crates/trapla-core/tests/port_verification.rs` 是 T7 复核人另写的一套（不依赖上述测试），
  22 个夹具逐行给结论。结论：未登记差异 0；报告见 `doc/port_verification.md`。
- 夹具是**冻结的 C++ 行为 oracle**：出现有意分歧时**不改夹具**，而是在测试与
  `doc/known_divergences.md` 里逐行登记。

## 5 有意分歧索引

**D1–D5**（完整证据、手算与强制点见 `doc/known_divergences.md`）：

| 编号 | 分歧 | 一句话 |
|---|---|---|
| D1 | `satisfy_turn` 加角度归一化 | C++ 不归一化，`support.rz ≈ ±π` 时把合法小转角折成 ±2π 伪大角；Rust 先归一到 (-π, π] 再判开区间。夹具 3 行（case 16/24/26）翻转，未改夹具。 |
| D2 | `--start` 保真度 | C++ 的 `main` 读了起点却从不传给规划（只传 goal）；Rust **真的使用** `--start`。这是 CLI 契约层面的差异，行为**有意保留**。 |
| D3 | NoPath 分层与越界语义 | 早期「51 个 exit=2」已过期；现行口径见 `doc/nopath_diagnosis.md` 文首：失败全部是越界目标，T9a 后报 `exit=1`。 |
| D4 | 地形约束真正生效 | `Grid::restore_block` 换算方向「乘→除」、陡峭度由断路改为代价（`PENALTY_K`）、按缩放节点记忆化；`restore.csv` 12 行、`steep_extend.csv` 14 行有意分歧。记忆化 18/18 输出 SHA 不变。 |
| D5 | `standable.csv` 的 `normal_angle` 列退休 | C++ `cover()` 的 `unordered_set` 迭代序 + 顺序敏感 `up_fit` ⇒ 该列不是可复现 oracle；`standable` 布尔 45/45 一致，夹具不改、测试只断言布尔。 |

**三处代码级有意改动**（不在 D 编号里，但同等重要）：

1. **空候选不再越界崩溃**：原 C++ 在候选表为空时取 `candidates[0]`，越界 `SIGABRT(134)`；
   Rust 返回 `CoreError::NoPath`（`exit=2`）。崩溃输入本来就没有基线输出可对齐，修它不损失对齐能力。
2. **书账错位修正**：原实现把 46/46 次候选应用到了错误的脚；Rust 修正后，早期「3 个 L2（`find_target`
   空候选）」失败不再出现。
3. **猫步 D+B 策略**：足印分道（`LANE_HALF_WIDTH_CM = 13.0`）+ 提交路径朝向解耦
   （`HEADING_SMOOTHING = 0.5`）；效果现状见下节。

**另一处 CLI 语义收紧（T9a）**：`--start/--goal` 越界由「规划失败 `exit=2`」改为「输入错 `exit=1`」。

## 6 走猫步的现状

- 已做：D+B 策略（足印分道 + 朝向解耦），`same_foot_turn` 中位从基线的 15–23.76° 降到 ≈0.04°。
- **未修好**：主判据 `proper_crossings`（左右脚折线严格交叉对数）目标 **= 0**，实测 **4**；
  横向间距 min 目标 **≥ 3.0**，实测 **0.82**；步数相对基线 +24%（指引 ≤ +10%）。
- 判据表与基线区间：`doc/catwalk_diagnosis.md` §④；方案与实施：`doc/catwalk_design.md`、`doc/catwalk_fix.md`；
  现状登记：`doc/known_divergences.md` D4「仍未达成 / 未决」。

## 7 确定性与错误边界

- 随机性只有一个落点：缩放图 A* 的邻居洗牌（`rng::Rng`），由 `--seed` 定种。
  「同 seed 可复现」与「洗牌确实消耗 rng」两条命题都有测试与独立验证（见 `doc/port_verification.md`）。
  注意：**不同 seed 可以给出相同路径**（地形代价使最优解唯一时），这不是缺陷。
- IO/格式/越界分别映射到 `exit=1`，无路径 `exit=2`，其它内部错误 `exit=3`；`trapla-io` 的 `IoError`
  在边界处转成 CLI 的退出码，不在内核里 `panic`。

## 8 关于 `crates/**` 里的 C++ 溯源注释

C++ 实现已整体移除（见 `doc/baseline.md`），但 `crates/**` 的文档注释里**有意保留**了对原函数的引用，
形如 `/// 检查新位置是否满足步长约束（原 \`satisfy_stride\`，\`src/robot/robot.cpp:228-230\`）`。
这些是**移植审计线索**，不是死引用：`doc/known_divergences.md` 的每条有意分歧、以及 golden 夹具的
分类，都靠它们指回原实现。所以「全仓 grep 不到 `.cpp`」**不是**本项目的验收口径；有效口径是三条：

- `readme.md` 与本文**不含** C++ 路径引用；
- 凡提到 C++ 路径的历史文档，文首统一声明「这些是历史修订 `0038315` 里的路径」并给出取法
  `git show 0038315:<路径>`；
- `crates/**` 的溯源注释按上述方式保留。
