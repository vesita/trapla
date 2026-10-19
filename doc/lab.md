# 算法测试台（trapla-lab）

无头（不依赖 Bevy）的算法测试环境：确定性场景 → 批量执行 → **独立**硬约束校验 → 汇总与回归门禁。
代码在 `crates/trapla-lab/`，命令名 `trapla-lab`。四层：场景层（`scenario`）/ 执行层（`planner`）/
校验层（`validate`）/ 报告层（`bench` + `report`），运行阶段由 `runstate` 的状态机按序推进。

## 怎么跑

```bash
cargo run -q -p trapla-lab -- list                       # 场景清单
cargo run -q -p trapla-lab -- scenario --out /tmp/lab/maps
cargo run -q -p trapla-lab -- plan --map /tmp/lab/maps/flat.csv --start 10,10 --goal 150,150 \
  --seed 1 --out /tmp/lab/path.csv --validate            # 用指定规划器跑一次并自校验
cargo run -q -p trapla-lab -- validate --path /tmp/lab/path.csv --start 10,10 \
  --map /tmp/lab/maps/flat.csv --goal 150,150            # 独立校验一条点位序列
cargo run -q -p trapla-lab -- bench --size 200 --seeds 1,2 \
  --out /tmp/lab/bench.json --csv /tmp/lab/bench.csv
cargo run -q -p trapla-lab -- bench --seeds 1,2 --baseline /tmp/lab/bench.json   # 回归门禁
```

退出码：`0` 通过；`1` 输入错；`2` `validate` 有硬约束越界 / `bench` 门禁不过；`3` 其它内部错。
`--rz` / `--coord-quantum` / `--overrides` 在 `--help` 里。

## 判据

硬约束（`validate` 判的就是这一组，`crates/trapla-core/src/config.rs` 是阈值唯一事实源）：

| 约束 | 口径 |
|---|---|
| 步长 | **同一只脚**的位移 = `\|p[i+2] − p[i]\|`（首步用初始足位），严格 `< 40` |
| 转角 | `wrap_angle(步向 − 支撑脚 rz)` 严格落在 `(-75°, 75°)`，`wrap` 归一到 `(-π, π]` |
| 双足间距 | 支撑脚纵向中心线 → 摆动脚新位置四角的最小垂距 − 1.5，夹紧 `≥ 0`，闭区间 `[2−1e-3, 10+1e-3]` |
| 足覆盖 | 覆盖格全部在图内，且拟合平面法向角严格 `< 20°` |

`--metrics` 里的 `stride`/`foot_center_separation` 是**相邻落足点**的跨距，几何上不是步长
（相邻两点属于两只不同的脚，它超过 40 也不代表违反 40 的步长约束）；校验器只用点位序列自己算
同脚位移，不读 `metrics.stride`。

软指标（`bench` 汇总，取自 `trapla_core::metrics`）：`proper_crossings`、`lateral_min`、`steps`、耗时。

门禁（`bench` 退出码）：参考实现必须达成每个场景的期望类别、硬约束零越界；对照坏实现必须在
全部场景 × seed 上**至少被判出一次**硬越界（否则说明校验器在空转）；给了 `--baseline` 再逐场景比
`steps_mean`（阈值 `--max-step-regression`，缺省 +10%）、`proper_crossings`、`lateral_min`、硬越界数，
基线缺项按失败处理。

## rz 重建约定（校验器的前提）

点位序列只有 `(x, y)`，朝向必须重建，`validate` 会显式打印用的是哪一种：

- `smoothed`（缺省）：提交语义 `Robot::next_committed`——位移 `≥ active_distance` 时按 0.5 向步向插值；
- `raw`：`Robot::next` 语义——`rz = 步向`，这是**候选检查**用的朝向，不是落足后的朝向；
- `sticky`：`rz` 永不更新，纯敏感性对照。

实测只有 `smoothed` 能复现 `Robot::feet()` 的真实末态（正常路径逐位一致；停摆路径末端 ≤2e-3 rad；
`raw` 偏差可达 5.5 rad）。这条差异也是当前实现会违反硬约束的机制：`find_path` 用 `next` 的朝向判候选、
用 `next_committed` 的朝向落足，两者不一致时「判过的姿态」与「执行的姿态」不是同一个。

奇偶脚映射：偶数下标是左脚（`doc/project.md` §2.2），`--first-moved right` 可换；`bench` 里由
`Robot::feet()` 与序列末点比对得出（报告里的 `parity_evidence`），不靠约定。

坐标量化：CSV 固定 6 位小数 ⇒ 量化步长 5e-7。`--coord-quantum` 给出线性容差
（`2 × quantum × √2`），只有**超出该容差**的越界才算硬越界（否则报告成量化界内的边界值 `~`）；
`--coord-quantum 0` 走严格判据。`validate` 还会换 `rz` 约定与奇偶脚映射各再判一次，打印结论是否翻转。

## 场景清单

`scenario::builtin(size)` 生成；尺寸缺省 200×200，`--size` 可改。坐标按尺寸成比例。

| id | 类别 | 期望 | 参数 |
|---|---|---|---|
| `flat` | 平坦 | 可规划 | 全 0 |
| `slope` | 缓坡 | 可规划 | `h = 0.08·(x+y)`（法向 ≈6.45°） |
| `step` | 台阶 | 可规划 | `x ≥ size/2` 处抬升 1.5 |
| `bumpy` | 颠簸 | 可规划 | 振幅 1.2、周期 6/7 的周期起伏 |
| `corridor` | 窄桥/走廊 | 可规划 | 中间 30% 宽度可走，两侧墙高 120 |
| `obstacles` | 随机障碍场 | 可规划 | 密度 5% 的 `+inf`，起点/终点附近 3 格净空 |
| `fault` | 台阶（全高断层） | 预期无路径 | `size/2 ± size/8` 一条 20 格厚 `+inf` 断层 |
| `oob` | 输入边界 | 预期输入错 | 目标点在图外 |

场景生成只依赖 `(id, 尺寸, seed)`；manifest（`scenario --out` 里的 `manifest.json`）带每张图的
sha256 指纹，用来判「两次生成是否同一张图」。

**地形设计约束**：`find_path` 的缩放层固定 `scale = 1/40`，一个缩放节点覆盖原图约 41×41 格，
障碍占比 `≥ 0.3` 即判死。所以「比 41 格窄的通道/断层」在缩放层会被整片判死，走廊与断层的宽度
都按这个下限设计。

## 怎么加算法

实现 `trapla_lab::planner::Planner`：

```rust
fn name(&self) -> &'static str;                     // 报告与基线里的稳定名字
fn description(&self) -> &'static str;
fn role(&self) -> PlannerRole;                      // Reference = 门禁要求合规；Fixture = 对照坏实现
fn plan(&self, request: &PlanRequest<'_>) -> PlanResult;
```

在 `cli.rs::planner_by_name` 注册名字即可被 `--planner` / `--planners` 选中。
`PlanResult` 里 `parity_evidence` 与 `final_feet` 是可选的证据位：能给出机器人真实状态的实现填上，
`tests/rz_reconstruction.rs` 这类守卫测试会用它对照重建结果。

## 当前状态（本机实测，2026-10-19）

硬约束越界为 0：`data/csv/map.csv` 上 11/11 组合（5 个 seed + 3 组起终点 × 2 seed）严格越界 0；
`bench --size 200 --seeds 1,2` 的 8 个内置场景 robot 行 hard 全 0、门禁通过（退出码 `0`），
对照坏实现仍被判出 3620 步越界。修前的现象与修正（候选判定改用提交姿态 + 收尾落足门禁）见
`doc/known_divergences.md` D6：同一条 seed 42 / `0,0 → 500,500` 的路径修前 62 点 / 16 步越界，修后 66 点 / 0 步。

复现与重测：`trapla-lab plan --map data/csv/map.csv --start 0,0 --goal 500,500 --seed 42 --validate`
（修后 66 点、严格越界 0、退出码 `0`，路径 sha256 `6edc8e24…`）、
`trapla-lab bench --size 200 --seeds 1,2`（门禁通过、退出码 `0`）。

`bfs` 在 tick 耗尽或目标越界时静默返回摆动脚原位置（`robot.rs` 的缺陷保留路径），停摆因此可能表现为
「返回成功」而不是无路径；`bench` 不把「返回成功」当作达成期望，而是用「末端双足中心距目标
< `active_distance`」判 `plannable` 是否达成。
