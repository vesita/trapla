# 已知有意分歧登记（known divergences）

本文件登记 **Rust 重写有意偏离 C++ 基线**的地方。每条都给出：现象、判据（一条能判对错的命令或测试）、
为什么这是有意的、影响面、以及**尚未修**的部分。目的：避免后来者把有意修正当成 bug 改回去，
也避免把 C++ 基线当成逐点 oracle 去对齐。

- 基线来源：C++ 仓库 `b53543b`；冻结证据修订 `0038315`；夹具在 `crates/trapla-core/tests/golden/`。
- 夹具是**冻结的 C++ 行为 oracle**：出现有意分歧时**不改夹具**，而是在测试里逐行登记 + 在本文档留证。
- 登记时间：2026-10-16。

---

## D1 `satisfy_turn` 角度归一化（T6b 修复，代号 T6b-D1）

### 现象与根因

C++ `src/robot/robot.cpp:232-235` 的 `satisfy_turn` 直接比较 `atan2` 差值：

```cpp
auto angle = get_swing_foot().position.angle(new_pos) - get_support_foot().rz;
return angle < max_turn && angle > -max_turn;
```

Rust 曾在 `crates/trapla-core/src/robot.rs` 忠实移植同一式子。`support.rz` 接近 ±π 时，
朝另一侧的候选方向会被 `atan2` 折回同一圈的另一端，差值变成 ±2π 量级的**伪大角**
（实测 raw = −342.0277°），于是合法的小转角候选被拒；候选集清空后 `find_target` 返回
`CoreError::NoPath`，整条规划以 `exit=2` 失败。

### 修正

`crates/trapla-core/src/robot.rs` 现在先用 `crate::metrics::wrap_angle` 把差角归一到 `(-π, π]`，
再判严格开区间。语义**仍是「严格开区间」**，只是把差角换算到同一圈内。
`metrics.rs` 只被引用，未被修改。

### 与冻结夹具的分歧（case 16 / 24 / 26）

`crates/trapla-core/tests/golden/satisfy_turn.csv` 有 42 行，其中 3 行因归一化翻转。
夹具**一个字节都没有改**（它记的是未归一化的 C++ 行为），这 3 行在
`tests/robot_golden.rs::test_golden_satisfy_turn` 里逐行登记。

手算（`raw = atan2(new − swing) − support.rz`，再 mod 2π 归一到 `(-π, π]`；
`max_turn = 75° = 1.3089969389957472 rad`）：

| case | 夹具期望 | raw | 归一化后 Δ | 归一化后期望 | 算式 |
|---|---|---|---|---|---|
| 16 | 拒绝 | −342.0277° = −5.969509944433961 rad | **+17.9723°** = +0.313675362745625 rad | 通过 | raw + 360° |
| 24 | 拒绝 | +301.5635° = +5.263276023707633 rad | **−58.4365°** = −1.019909283471954 rad | 通过 | raw − 360° |
| 26 | 拒绝 | +295.8224° = +5.163075289848182 rad | **−64.1776°** = −1.120110017331404 rad | 通过 | raw − 360° |

为什么这是**有意**的语义修正：

1. 三个 raw 值都是 `atan2` 的同一圈伪值（|raw| > 300°），物理上不可能是「转向 342°」；
2. 归一化后的 |Δ| = 17.97° / 58.44° / 64.18°，都在文档 `doc/project.md:16`「最大转向角 < ±75°」
   的允许范围内——被拒是伪值造成的误判；
3. 约束的语义对象是**同圈内的差角**，不是未归一化的原始差值。

### 强制点（判据）

- `cargo test -p trapla-core --test robot_golden::test_golden_satisfy_turn`
  —— 39 行照夹具断言，3 行按上表手算断言（先断言夹具期望仍是 `0`，过期即变红）；
- `cargo test -p trapla-core --test robot_golden::test_satisfy_turn_wrap_boundary`
  —— 7 例：`±178°/±170°` 同侧小角、两个真跨圈例（raw ±348° → Δ ∓/±12°）、
  夹具 case 16 原例、归一化后仍超界（+82°）、以及归一化后 +74.999°/+75.001° 的开区间边界；
- **负向对照（实测）**：去掉 `wrap_angle` 后
  `cargo test -p trapla-core --test robot_golden` → `2 failed`（`test_satisfy_turn_wrap_boundary`
  的 case3 与 `test_golden_satisfy_turn` 的 case 16）；e2e
  `--start 700,700 --goal 500,1000 --seed 0` 由 `exit=0` 变 `exit=2`。

### 影响面（配对实测，2026-10-16）

对 4 个起点 × 10 个目标 × 3 个 seed = **120 个组合**，用修前/修后两个二进制逐一配对：

| 修前 → 修后 | 组合数 |
|---|---|
| 0 → 0 | 64 |
| **2 → 0** | **5** |
| 2 → 2 | 51 |
| 0 → 2（回归） | **0** |

`2 → 0` 的 5 个组合：
`(700,700)→(500,1000) seed0`、`(700,700)→(300,900) seed1`、`(700,700)→(200,1500) seed0`、
`(500,500)→(200,1500) seed1`、`(500,500)→(200,1500) seed2`。

> 说明：任务书里点名的 `(700,700)→(900,1200)` 与 `(0,0)→(900,1200)` 在**修前就已经是 `exit=0`**
> （作者在修复前的 14 个 seed 上实测同样全为 0），因此它们**不能**作为本修复的判别用例；
> 判别用例改用上表 5 个组合。详见 T6b 报告。

---

## D2 CLI `--start` 的保真度（登记，行为**未改**）

### C++ 侧事实（读码 + 手算）

- `src/main.cpp:56` 构造 `Robot robot;`，用的是默认参数（`include/robot/robot.hpp:78-79`：
  `max_stride=40, max_turn=75°, max_sep=10, min_sep=2, foot_len=5, foot_wid=3`）；
- `src/main.cpp:61-81` 虽然读了 `argv[1..4]` 当起点，但 `src/main.cpp:85` 只把 **goal** 传给
  `robot.find_path(ground, goal)` ——**起点从未生效**；
- 因此 C++ 的有效初始位姿由 `src/robot/robot.cpp:22-26` 决定：
  `feet[0]=(3/2+1, 5/2+1−0.6·2−0.4·10)=(2.5, −1.7)`，`feet[1]=(2.5, 3.5)`，两脚 `rz=0`，
  `now_which_foot_to_move=Left`；`src/robot/robot.cpp:241-245` 的 `position()` 取两足中点
  = **(2.5, 0.9)**；`as_index()` = `(round(2.5), round(0.9))` = **(3, 1)**。

### Rust 侧

`Robot::new(config, start)` **真的使用 `start`**（`robot.rs:29-44`）：`sep = 0.6·2 + 0.4·10 = 5.2`，
`left=(x, y−2.6)`、`right=(x, y+2.6)` —— 与 C++ 的脚布局**逐字一致**。
所以 `--start 2.5,0.9` 能精确复现 C++ 的初始位姿（两足坐标、rz、which 与 A* 起点 idx=(3,1) 全同）。

### 对比（同 seed=3、goal=(900,1200)）

| 运行 | 首个落点 | 点数 | exit |
|---|---|---|---|
| C++ 基线（`0038315:baseline/frozen/seed-3_alt_run1.csv`，默认位姿起步） | (33.4096, 11.7822) | 130 | 0 |
| Rust `--start 2.5,0.9` | (35.408965, 17.300000) | 109 | 0 |
| Rust `--start 0,0` | (9.000000, 16.452559) | 112 | 0 |

**结论：即使把初始位姿精确对齐到 C++ 的有效起点，首个落点与点数仍不一致。**

### 仍然存在的保真度缺口（列出，**本任务不修**）

1. **RNG 算法与消耗顺序不同**：Rust 用 `rng.rs` 的自研确定性 PRNG，C++ 用
   `std::mt19937`（由 `TRAPLA_SEED` 定种）。两者在 `scale_star` 里的邻居洗牌序列不同，
   A* 的 tie-break 与 `direction_determine` 的引导点随之不同 → **端到端逐点相同做不到**
   （这是 Lead 已认定的前提，不是缺陷）。
2. **猫步修正**：`robot.rs:446-453` 用 `pend_dots[1]`，C++ 用 `pend_dots[0]`（见 `doc/catwalk_fix.md`）。
3. **空候选处理**：C++ `candidates[0]` 越界 `SIGABRT(134)`，Rust 返回 `CoreError::NoPath`。
4. **D1 的 `satisfy_turn` 归一化**本任务新增的分歧。
5. **`--start` 语义本身**：C++ 忽略、Rust 采纳 —— 属 CLI/core 契约，是否改由 Lead 决定。

---

## D3 尚未定位的 `NoPath` 缺口（登记，本任务不修）

配对实测显示，120 个组合中有 **51 个在修前/修后都是 `exit=2`** —— 归一化对它们没有任何影响，
所以它们**不是** `satisfy_turn` 引起的。按目标聚合失败组合：

| goal | 失败的 (start, seed) 组合数 |
|---|---|
| (1500, 200) / (1500, 1500) / (1200, 900) / (1000, 500) | 各 12 |
| (900, 300) | 3 |

候选原因（**读码推断，未验证**）：`robot.rs:402-412` 的 `scale_star` 返回 `None`（`find_path`
里另一条 `NoPath` 出口），或候选在 spacing/stride 约束下为空。处置：登记待查，不在 T6b 修。

---

## 变更记录

| 日期 | 内容 |
|---|---|
| 2026-10-16 | 建立；登记 D1（satisfy_turn 归一化）、D2（`--start` 保真度）、D3（未定位 NoPath 缺口） |
