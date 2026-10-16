# T7 独立移植复核报告

- 复核人：`recon-trapla`（**T6b/T0 作者**；不是 T2/T3/T4/T5 的作者——见 §11 的独立性声明）。
- 复核任务：`task-14`（原始 8 项）+ `task-25`（增补，**取代 task-14 的第 3、4 项**，并新增性能复核）。
- 复核范围：git 提交区间 **`0622578..HEAD`**（HEAD = `a53e19d`）。复核时 `git status --porcelain` 干净（只有本报告与 `port_verification.rs` 两个新增物）。
- 独立对照实现：`crates/trapla-core/tests/port_verification.rs`（复核人新写，不依赖作者测试；作者文件一律未改）。
- 结论每条标 `实测`（命令 + 原始输出 + 退出码）或 `读码`（`文件:行`）。

---

## 0. 复核范围

```
$ git log --oneline 0622578..HEAD
a53e19d docs(known_divergences): D4 补记 T6g 记忆化的实测耗时（4.1x/4.9x，18 个 SHA 不变）
6baffb4 fix(core): 地形约束真正生效（restore_block 方向 + 陡峭度断路改代价 + 记忆化）

$ git diff 0622578..HEAD --stat
 crates/trapla-core/src/astar.rs                  |  54 ++++-
 crates/trapla-core/src/grid.rs                   |  37 ++--
 crates/trapla-core/src/terrain.rs                |  98 +++++----
 crates/trapla-core/tests/geom_grid_golden.rs     | 250 +++++++++++++++++----
 crates/trapla-core/tests/terrain_astar_golden.rs | 262 ++++++++++++++++++++++-
 doc/known_divergences.md                         | 216 +++++++++++++++++++
 6 files changed, 811 insertions(+), 106 deletions(-)
```

范围内**没有**动：`tests/golden/**`（夹具）、`trapla-io`、`trapla-cli`、`trapla-viz`、`data/`、`python/`（逐项 `git diff` 为空）。
未提交改动：只有 `?? crates/trapla-core/tests/port_verification.rs`（本复核的交付物）；无作者未提交改动。

---

## 1. 构建 / 测试 / clippy（task-14 第 1 项）

**口径偏离（按 task-25 硬约束）**：task-14 要求 `--workspace`，task-25 明令「不许跑 `--workspace` 级 cargo（workspace 里有 Bevy 0.20）」。
workspace 的 `default-members = ["trapla-core","trapla-io","trapla-cli"]`（`Cargo.toml:6`），`trapla-viz` 不在其中，
所以下面用**默认成员**跑，等价于 task-14 想要的范围减去 viz。**viz 未构建/未测试 ⇒ §11 记为未验证。**

```
$ cargo build
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.30s        BUILD_EXIT=0

$ cargo test
     Running unittests src/main.rs (trapla)             running 0 tests   ok
     Running tests/cli.rs (trapla-cli)                  running 6 tests   ok. 6 passed
     Running unittests src/lib.rs (trapla_core)         running 8 tests   ok. 8 passed
     Running tests/geom_grid_golden.rs                  running 24 tests  ok. 24 passed
     Running tests/robot_golden.rs                      running 17 tests  ok. 17 passed
     Running tests/terrain_astar_golden.rs              running 22 tests  ok. 22 passed
     Running unittests src/lib.rs (trapla_io)           running 0 tests   ok
     Running tests/csv_roundtrip.rs (trapla-io)         running 8 tests   ok. 8 passed
   Doc-tests trapla_core / trapla_io                    running 0 tests   ok
                                                                             TEST_EXIT=0

$ cargo clippy --all-targets -- -D warnings
    Finished `dev` profile ... in 0.89s                                      CLIPPY_EXIT=0
```

`geom_grid_golden` 24 / `terrain_astar_golden` 22 / `robot_golden` 17（不是 task-14 里写的 22/19/16，那是 D4 之前的计数）。

加入本次复核的 `tests/port_verification.rs` 之后重跑（即最终门禁，**包含**新测试目标）：

```
$ cargo clippy --all-targets -- -D warnings
    Finished `dev` profile ... in 0.24s                                    CLIPPY_EXIT=0
$ cargo test
    11 个 "test result: ok"，无 FAILED；port_verification 3 passed        TEST_EXIT=0
```

---

## 2. golden 逐条对照（task-14 第 2 项，核心）

独立对照在 `crates/trapla-core/tests/port_verification.rs`：
按**表头名**解析、容差 1e-9、逐行比；`up_fit` 的输入列用**独立重写的单步 up_fit**（公开 API 只返回平面、不暴露被原地
改写的 `three`）。运行：

```
$ cargo test -p trapla-core --test port_verification -- --nocapture
===== T7 独立 golden 对照台账 =====
夹具                                               总行     一致      无对照    登记分歧     待裁决     未登记分歧
grounds.csv                                     387    387        0       0       0         0
golden_summary.txt                               22     22        0       0       0         0
normal_angle.csv                                 45     45        3       0       0         0
up_fit.csv                                       45     45        0       0       0         0
trip.csv                                         45     44        0       1       0         0
corner.csv                                       40     40        0       0       0         0
cover.csv                                      1072   1072        0       0       0         0
standable.csv                                    45     43       42       0       2         0
slide.csv                                        45     45        0       0       0         0
satisfy_spacing.csv                              55     55        0       0       0         0
satisfy_stride.csv                               40     40        0       0       0         0
satisfy_turn.csv                                 42     39        0       3       0         0
restore.csv                                      40      8       20      12       0         0
restore_dot.csv                                  60     30       30       0       0         0
central_restore.csv                              70     70        0       0       0         0
index_scale.csv                                 220    180       40       0       0         0
scale_graph.csv+scale_graph_cells.csv          5217   5217        0       0       0         0
steep_extend.csv                                 46     32        0      14       0         0
geometric_decay.csv                              76     76        0       0       0         0
direction_determine.csv+direction_guides.csv     40     40        0       0       0         0
未登记分歧合计 = 0
```
`test result: ok. 3 passed; 0 failed` / `EXIT=0`（三个独立测试：`golden_fixture_audit`、`independent_shuffle_consumes_rng_and_same_seed_reproduces`、`nopath_layering_at_head`）。

### 2.1 22 个夹具逐个结论（合并行在上面已拆开）

| # | 夹具 | 结论 | 不一致条数 | 备注 |
|---|---|---|---|---|
| 1 | `grounds.csv` | 通过（输入夹具） | 0 | 解析出 7 张图 `0:5x7 1:8x8 2:6x9 3:7x7 4:6x10 5:10x10 6:5x5` |
| 2 | `golden_summary.txt` | 通过 | 0 | 每个 `rows=/cases=/cells_rows=/guides_rows=` 都与实际夹具一致 |
| 3 | `normal_angle.csv` | 通过 | 0（3 行无对照） | case 3/4/5 三点共线，C++ 是未初始化 UB 值，本仓返回 `DegeneratePlane`（已登记，见 §3） |
| 4 | `up_fit.csv` | 通过 | 0 | 平面 + 调用后 `three` 全部逐位一致（独立重写算法复核） |
| 5 | `trip.csv` | 有意分歧 1 行 | 1 | case 26 初始三点共线（C++ UB）；case 1/21 同类但数值恰好一致 |
| 6 | `corner.csv` | 通过 | 0 | 40/40 |
| 7 | `cover.csv` | 通过 | 0 | 1072/1072（按 (x,y) 升序的集合逐点） |
| 8 | `standable.csv` | **待裁决 2 行** | 2 | 见 §2.2，`normal_angle` 列对 >3 点顺序敏感 |
| 9 | `slide.csv` | 通过 | 0 | 45/45，结果枚举与输出角点一致 |
| 10 | `satisfy_spacing.csv` | 通过 | 0 | 55/55；独立复算的 `spacing` 与夹具逐位一致 |
| 11 | `satisfy_stride.csv` | 通过 | 0 | 40/40（含 distance、严格开区间边界） |
| 12 | `satisfy_turn.csv` | 有意分歧 3 行 | 3 | case 16/24/26（D1 归一化） |
| 13 | `restore.csv` | 有意分歧 12 行 | 12 | 20 个 `int` 行中 8 行与夹具同、12 行因 D4 换算方向翻转；20 个 `dot` 行无对应入口 |
| 14 | `restore_dot.csv` | 通过 | 0 | 30 个 `int` 行全对；30 个 `dot` 行是 C++ 的 `SqDot` 重载（本仓无此入口） |
| 15 | `central_restore.csv` | 通过 | 0 | 70/70（`int` 按截断、`dot` 按浮点） |
| 16 | `index_scale.csv` | 通过 | 0 | 180/180；40 个 `scale_double` 行是被消灭的同名重载，无对照入口 |
| 17 | `scale_graph.csv` | 通过 | 0 | 35/35 尺寸行 |
| 18 | `scale_graph_cells.csv` | 通过 | 0 | 5217/5217 格逐值 |
| 19 | `steep_extend.csv` | 有意分歧 14 行 | 14 | 13 行因 `min_x` 笔误修正（x0>x1，见 §3.5），1 行 case 7 因 >10 断路改代价 |
| 20 | `geometric_decay.csv` | 通过 | 0 | 76/76（`n>10` 的 1 行只比对前 10 项并核长度） |
| 21 | `direction_determine.csv` | 通过 | 0 | 40/40 |
| 22 | `direction_guides.csv` | 通过（输入夹具） | 0 | 110 行 guides 与 `n` 列一一对应 |

**22/22 都有结论；未登记分歧 = 0；待裁决 = 2（standable case 14/43）。**

### 2.2 待裁决 R1（本次复核新发现，未登记）

`standable.csv` 有 3 行 `all_inside=1`（case 8/14/43）。case 8 的 `normal_angle` 与夹具一致；**case 14/43 不一致**：

| case | 夹具 `normal_angle` | 本仓（`Terrain::normal_angle`） | `standable` 布尔 |
|---|---|---|---|
| 14 | 0.687411969678680 | 1.509366844445 | 两边都 false ✔ |
| 43 | 1.356023943959926 | 1.383402917548 | 两边都 false ✔ |

成因已用 Python 独立复算定位（**实测**）：`Foot::cover()` 在 C++ 返回的是 `std::unordered_set` 的**迭代器顺序**，
而 `Ground::trip`→`up_fit` 是**顺序敏感的贪心**；本仓 `cover()` 是 `(x,y)` 升序。同一覆盖点集在不同顺序下：

| 顺序 | case 8 | case 14 | case 43 |
|---|---|---|---|
| scan(插入序) | 1.34544 | 1.43646 | 1.11916 |
| sorted(x,y) | 1.00685 ← 夹具同 | 1.50937 | 1.38340 |
| sorted(y,x) | 1.53503 | 1.48118 | 1.11916 |
| reverse_scan | 1.00685 ← 夹具同 | 1.44078 | 1.35602 ← 夹具同 |
| **夹具值** | 1.00685 | **0.68741（以上都不等）** | 1.35602 |

⇒ `standable.csv` 的 `normal_angle` 列**对 >3 个覆盖点不构成可复现规格**（case 14 的夹具值来自 libstdc++ 的哈希序，
任何自然顺序都不复现）。这不影响语义输出（`standable` 布尔全 45 行一致），但按 task-25 判据属于「差异超出已登记范围」，
**列入阻塞项/待裁决**（B1）。复现命令：

```bash
cargo test -p trapla-core --test port_verification golden_fixture_audit -- --nocapture   # 看 [待裁决] 行
```

---

## 3. 有意分歧逐条核实（task-25 取代第 3 项）

### D1 `satisfy_turn` 角度归一化 —— **是，恰好只差 16/24/26 三行**
- 独立对照：42 行中 39 行与夹具一致；3 行翻转，且**翻转集合恰好是 `{16,24,26}`**。
- 逐行归一化值（我复核人重算，与夹具的 `angle` 列先核对过）：case 16 raw −5.969509944434 → +0.313675362746；
  case 24 raw +5.263276023708 → −1.019909283472；case 26 raw +5.163075289848 → −1.120110017331。
- 夹具未被修改（范围内 `git diff -- tests/golden` 为空）。

### D2 `--start` 保真度 —— **描述与实测一致**
- 读码（C++ 侧）：`src/main.cpp:56` `Robot robot;` 用默认参数；`src/main.cpp:85` 只把 goal 传给 `find_path`，start 从未生效。
- 实测（Rust 侧）：`--start 2.5,0.9 --goal 900,1200 --seed 3` 与 C++ 默认位姿等价（两足 `(2.5,−1.7)/(2.5,3.5)`、rz=0、中点 (2.5,0.9)、A* 起点 idx=(3,1)），
  但首落点与点数仍不同（(35.408965,17.300000)/109 vs C++ (33.4096,11.7822)/130）⇒ 还有其它保真度缺口（RNG 算法、猫步修正、空候选处理），
  `doc/known_divergences.md` D2 列的 5 条与实测一致。

### D3 51 个 NoPath 的分层 —— **数字已过期：当前是 48，不是 51**
- 独立重做（`port_verification.rs::nopath_layering_at_head`，只用公开 API）：120 组合中失败 **48** 个，
  **全部是 L1**（`scale_star == None`），**L2 = 0**；48 个全部落在 4 个越界目标上（每个 12 = 4 起点 × 3 seed）：

```
D3 独立分层：120 组合中 find_path 失败 48 个；L1(scale_star=None)=48；L2(scale_star=Some 但 find_path 失败)=0
```

- CLI 层重扫（当前二进制）同样是 **72 exit=0 / 48 exit=2**：
```
exit=0: 72  exit=2: 48
  12 1500,200   12 1500,1500   12 1200,900   12 1000,500     （goal.x ≥ 1000 = 越界）
```
- ⇒ `doc/nopath_diagnosis.md` 的「51（48 L1 + 3 L2 真缺陷）」**锚在旧二进制**（其 §0 自述冻结于 `41ddc8bf…`）。
  当前 HEAD 上那 3 个 L2 已不复现。**是**「只剩 48 个越界目标」；**否**于「仍为 51/含 3 真缺陷」。

### D4 地形约束生效 —— **是，且记忆化是纯性能改动**
- `restore.csv`：独立复算「除」语义 20/20 与实现一致；与夹具 8 行同、**12 行不同**，作者测试也断言 `(8,12)`。
- `steep_extend.csv`：14 行分歧，其中 **13 行是 `min_x` 笔误修正**（这些行钳位后 x0>x1；C++ 只扫 x0 一列），
  另 **1 行是 case 7**（`>10` 判死改代价，夹具 −1 → 本仓 10.0149）。
- **记忆化前后输出逐字节相同（我重跑，9/9）**：3 输入 × 3 seed，`path_result.csv` 与 metrics JSON 各 18 个 SHA 全部相同：

```
start=0,0 goal=500,500      seed=42/7/2024   path=b18700c8d1e3e80d metrics=0d7300cf2ace286c   IDENTICAL ×3
start=500,500 goal=900,1200 seed=42/7/2024   path=7be908073fc1f94f metrics=29fbda9173a035d3   IDENTICAL ×3
start=0,0 goal=300,900      seed=42/7/2024   path=c1dce78dda489968 metrics=607ae0950358403a   IDENTICAL ×3
IDENTICAL=9 DIFFERENT=0
```
- 对照二进制：`/tmp/current-trapla`，`sha256 = 561fc0ac1bc7460a3e783f1f46d88040706500ff92be9e926b453b3d691b8351`（与 lead 给的 `561fc0ac…` 一致；
  它与当前二进制输出相同但慢 ~4–5×，可判定它就是 T6g 记忆化**之前**的那个构建）。
- `PENALTY_K = 3.0`（`crates/trapla-core/src/astar.rs:175`），`grid.restore_block` 为除（`grid.rs:167-177`）——读码确认。

### 三处代码级有意改动
| 项 | 核实结论 | 证据 |
|---|---|---|
| ① 空候选越界 → `Err(NoPath)`（原 C++ `exit=134`） | **是**，且当前 `nopath_layering_at_head` 显示 L2 路径已不再触发 | `robot_golden.rs:test_empty_candidates_returns_no_path`；`port_verification.rs` 的 L2=0 |
| ② 书账错位修正（原 46/46 次候选应用到错误的脚） | **是**（读码 + 行为）：当前 `find_path` 用 `pend_dots[1]`；本复核未逐帧对拍 46/46 这个数字（见 §11 未验证） | `doc/catwalk_fix.md`；`robot.rs` 的 D+B 注释 |
| ③ 猫步 D+B（`LANE_HALF_WIDTH_CM=13.0` + `HEADING_SMOOTHING=0.5`） | **是**（常量与用法在读码层面存在）；但**效果未达标**（见 §5.2） | `doc/catwalk_design.md`/`catwalk_fix.md`；作者已在 `known_divergences.md` D4 自认「三判据仍未达标」 |

---

## 4. 可复现性新口径 + 独立洗牌命题（task-25 取代第 4 项）

**口径 1：同 seed 两次相同（path + metrics 两个 SHA）——实测**

```
start=0,0 goal=500,500 seed=42
b18700c8d1e3e80dc408b5f0e4c50c62c63b4725e85df89143da36be64fb8575  r1.csv
b18700c8d1e3e80dc408b5f0e4c50c62c63b4725e85df89143da36be64fb8575  r2.csv
0d7300cf2ace286cc2c82347e3408ad6d71f10a7c104176f1aca462cb74529de  r1.json
0d7300cf2ace286cc2c82347e3408ad6d71f10a7c104176f1aca462cb74529de  r2.json
start=500,500 goal=900,1200 seed=7  → 同样两两相同
```

**口径 2：洗牌仍然消耗 rng —— 我独立构造的最小验证**（`port_verification.rs::independent_shuffle_consumes_rng_and_same_seed_reproduces`）
验证的是两条命题，且只验证这两条：

1. **同 seed 两个独立 `Rng` 实例跑 `scale_star` 逐点相同**（`assert_eq!(pa.path, pb.path)`）；
2. **`scale_star` 确实推进了 `rng`**：一个实例跑过 `scale_star`、另一个没跑，随后各洗同一个 16 元素数组，结果**必须不同**
   （`assert_ne!(v1, v2)`）——这条把「洗牌被短路」这种会让口径 1 假性通过的情况排除掉。

另外显式验证了**旧口径不成立**：平坦 16×16 图、`scale=1/4`，`seed 7` 与 `seed 1234` 可以给出**相同**路径 —— 所以
「换 seed 结果应不同」本来就不是本仓性质（`assert_eq!(p7.path.first(), Some(&start))`，路径相同与否只打印不断言）。

---

## 5. 端到端（task-14 第 5 项）

### 5.1 `--start 0,0 --goal 500,500 --seed 42`

```
$ cargo run -q -p trapla-cli -- plan --map data/csv/map.csv --start 0,0 --goal 500,500 \
      --out /tmp/t7/e2e/s42.csv --seed 42 --metrics /tmp/t7/e2e/s42.json
EXIT=0 ; wc -l = 63（1 表头 + 62 点）
head: x,y / 32.908965,16.400000 / 27.455844,28.055844
```

3 个 seed 全部 exit=0、62 点；指标（Rust `metrics`）：

| seed | steps | proper_crossings | same_foot_turn 中位 | 横向间距 min | 横向间距中位 |
|---|---|---|---|---|---|
| 42 | 62 | 4 | 0.000775 rad ≈ 0.044° | 0.820491 | 8.821577 |
| 7 | 62 | 4 | 同上 | 同 | 同 |
| 2024 | 62 | 4 | 同上 | 同 | 同 |

（三个 seed 输出完全相同 —— 与 §4 的「不同 seed 可同」一致。）

### 5.2 与 `doc/catwalk_diagnosis.md` §④ 的修前基线范围对照

C++ 基线（`git show 0038315:baseline/catwalk_metrics.json`，goal=(500,500) 的 3 个 default 用例）：
`proper_crossings = 4/5/4`；横向间距 min `0.238/1.488/0.241`；中位 `10.615/11.671/10.195`；点数 `50/107/58`。

| 判据（§④） | 修前基线 | 修后预期 | 当前 HEAD（seed 42/7/2024） | 达成 |
|---|---|---|---|---|
| `proper_crossings` | 3–10 | **= 0** | **4** | **否**（主判据） |
| 同足步向摆动中位 | 15.00–23.76° | < 8° | 0.044° | 是 |
| 横向间距 min | 0.00–3.14 | ≥ 3.0 | **0.820** | **否** |
| 横向间距中位 | 10.20–12.62 | 维持 [5,13] | 8.82 | 是 |
| 步数 | 50–140 | 不增加 >10% | 62（vs 基线 50，+24%） | 否（观察项） |

⇒ 与作者在 `known_divergences.md` D4 的自述一致：**猫步三判据未达标**（作者已登记，不是隐瞒）。

---

## 6. 越界 / 删除 / 弱化断言（task-14 第 6 项，做在**提交区间**上）

- 范围只有 6 个文件（见 §0），没有超出 D4 的写域；夹具 / 其它 crate / `data/` / `python/` 均未动。
- 弱化断言扫描（范围 diff 的新增行）：`#[ignore]` **0**、`assert!(true)` **0**、`unwrap_or(true)` **0**、`dbg!`/TODO **0**。
- 删除的 assert 行 4 条，逐条核对**全部是替换成更强的断言**，不是弱化：
  - `assert_eq!(int_rows.len(), 20)` → `assert_eq!(golden_int.len(), 20)`（同断言换表）；
  - `assert_eq!(got, *want_int, ...)` → 改为与**独立复算**的 `want_corrected` 比，并新增 `(same_as_golden, divergent) == (8,12)`
    与结构性不变量（`近端 ≤ 远端`、落在 5×7 内）、`dot_equals_int == 13`；
  - `assert_ne!(got, want_dot, ...)` → 删（该断言建立在「本仓不复现双语义」之上，D4 后已由 12 行分歧表覆盖）；
  - `assert!(steep_extend(...).is_none())` → `assert_close(...expect("陡峭不是不可行"), 24.0)`（更强：钉住具体分值）。
- 新增 assert 行 35 条。

---

## 7. 测试质量抽查（task-14 第 7 项，随机 5 个）

| 测试 | 判定 | 理由 |
|---|---|---|
| `geom_grid_golden.rs::grid_cost_is_manhattan_plus_target_height` | 合格（轻微保留） | 期望值是手算字面量（`1.0+2.0` 等），不是用被测访问器当 oracle；但测试名与实现公式同名，若公式与期望同时写错会一起通过 |
| `terrain_astar_golden.rs::rng_frozen_seed_42_sequence` | 合格 | 冻结常量 + 非退化守卫（非 0、无重复），是真正的外部 oracle |
| `terrain_astar_golden.rs::steep_extend_is_pure_so_memoization_is_safe` | 合格 | 命题式（重复求值一致）；样本只 4 个节点，偏弱但诚实 |
| `terrain_astar_golden.rs::terrain_steepness_changes_the_route` | 合格 | K 扫描 + 路线差异命题，含打印 |
| `cli.rs::help_exits_zero_and_documents_exit_codes` | 合格 | 端到端行为断言 |

未发现「把实现抄一遍当断言」的测试。`robot_golden.rs::test_golden_*` 系列全部由夹具驱动（含 D1 的 3 行手算常量），也合格。

---

## 8. 文档一致性 + 悬空引用（task-14 第 8 项）

`doc/` + `readme.md` 中提到 C++ 文件/构建的地方（计数 = `src/`/`include/`/`.cpp`/`.hpp`/cmake/C++ 命中行）：

| 文件 | 命中行 | 与 Rust 现状 |
|---|---|---|
| `doc/project.md` | 58 | **仍是 C++ 时代的项目说明**（"项目使用CMake"、目录结构列 `src/`+`include/`、技术实现按 C++ 类描述） |
| `doc/catwalk_diagnosis.md` | 34 | 有意：诊断针对 C++ 基线，标注了 `src/...:行` |
| `doc/known_divergences.md` | 27 | 有意：分歧对照必须引 C++ 出处 |
| `doc/baseline.md` | 12 | 有意：基线记录 |
| `doc/catwalk_fix.md` | 6 | 有意：引 C++ 行为 |
| `doc/nopath_diagnosis.md` | 5 | 有意（但数字已过期，见 D3） |
| `readme.md` | 6 | **未更新**：`readme.md:5`「目前仍然存在走猫步的问题」（已做 D+B，且 §5.2 显示主判据仍未达标但形态已变）、`readme.md:88-103` CMake 构建/安装、`readme.md:111` `python3 scripts/run_tests.py`（路径本来就是错的） |
| `doc/scripting.md` / `doc/sync.md` / `doc/problem_flow.md` | 0 | 脚本/同步/问题分析文档，不提 C++ 源文件路径 |

**仍指向 `src/` / `include/` 的悬空引用（C++ 移除后会断）**：目前 C++ 源码**仍在树里**
（`git ls-files src include | wc -l` = **30**），所以这些引用现在还能解析；C++ 移除后，上表里
`readme.md`、`doc/project.md`、`doc/catwalk_diagnosis.md`、`doc/known_divergences.md`、`doc/baseline.md`、
`doc/catwalk_fix.md`、`doc/nopath_diagnosis.md` 里的 `src/...`/`include/...` 路径会全部失效。
**待办（后续任务）**：`readme.md` 整体重写为 Rust 工作流；`doc/project.md` 标注为历史/或重写。

---

## 9. 性能主张复核（task-25 新增）

对照二进制：`/tmp/current-trapla`（`sha256 = 561fc0ac1bc7460a…`，= 记忆化前构建，§3-D4 已验证输出相同）。

| 用例 | 记忆化后（current）中位 | 记忆化前（pre-memo）中位 | 提速 |
|---|---|---|---|
| `(0,0)->(500,500)` seed 42 | 0.032 s | 0.132 s | **4.13×** |
| `(500,500)->(900,1200)` seed 42 | 0.064 s | 0.326 s | **5.09×** |

- 「4.1× / 4.9×」：第一例 **4.13× 属实**；第二例我取 3 次中位是 **5.09×**（0.326/0.064），作者原值 0.318/0.065 = 4.89× ——
  差异来自运行间抖动（pre-memo 第二例单次 0.319–0.343 s），量级结论一致。
- 「第一例距 4× 目标差约 7%」：**未能复现**。我测得 4.13× **高于** 4× 目标约 3%；按作者自己写的 4.1× 也高于 4×。
  该句若指别的配对请给出算式；就当前数据应表述为「第一例仅在 4× 线附近（+3%），未稳定达标」——与「未稳定达标」的定性一致。
- 「18 个 SHA 逐项不变」= §3-D4 的 9 组 ×（path + metrics）= **18/18 相同，属实**。

---

## 10. 结论段

**移植（Rust 与原 C++ 的函数级对齐）：可判定为完成。**
- 22/22 夹具都有对照结论，**未登记差异 = 0**；有意分歧全部能归到已登记的 D1–D4 与三处代码级改动；
- 构建/测试（默认成员）/clippy 全绿；同 seed 可复现；`--workspace` 之外的 target（viz）按 task-25 禁令未跑（未验证）；
- 记忆化是纯性能改动（18/18 SHA 相同），性能主张方向属实。

**优化（猫步）：不可判定为完成。**
- §5.2 显示主判据 `proper_crossings` 目标 0、实测 4；横向间距 min 目标 ≥3、实测 0.820；步数 +24%。
- 作者已在 `known_divergences.md` D4 自述未达标，属**已知未达**，不是隐瞒；但按 T7 口径仍是未完成项。

### 阻塞项 / 待裁决（按 task-25：「差异超出已登记范围」即阻塞）

| # | 项 | 复现命令 | 影响 |
|---|---|---|---|
| **B1** | `standable.csv` 的 `normal_angle` 列对 >3 个覆盖点**顺序敏感**，case 14/43 与本仓不一致且未登记（成因：C++ `unordered_set` 迭代序 + 贪心 `up_fit`） | `cargo test -p trapla-core --test port_verification golden_fixture_audit -- --nocapture`（看 `[待裁决]`） | 不影响 `standable` 布尔（45/45 一致）；影响「22 夹具逐行可对照」的完整性。需裁决：登记该列不可复现 / 换成集合不变量 / 从夹具退休该列 |
| **B2** | 猫步三判据未达标（`proper_crossings` 4≠0；横向 min 0.82<3.0；步数 +24%） | `cargo run -q -p trapla-cli -- plan --map data/csv/map.csv --start 0,0 --goal 500,500 --out /tmp/v.csv --seed 42 --metrics /tmp/v.json` 后看 `proper_crossings`/`lateral_separation` | 影响「优化完成」判定，不影响移植 |
| **B3** | 文档过期：`doc/nopath_diagnosis.md` 的 51（48 L1+3 L2）当前为 48（全 L1）；`readme.md` 仍按 C++ 写 | `port_verification.rs::nopath_layering_at_head` 与 §8 表 | 影响读者判断，不影响代码 |
| **B4** | 48 个 `NoPath` 全是**越界目标**（`goal.x ≥ 1000`），产品语义上更像 `exit=1`（输入错）而非 `exit=2`（无路径） | `awk '$4==2{print $2}' /tmp/t7/scan120.txt \| sort \| uniq -c` | 待产品裁决（`nopath_diagnosis.md` §8.2 已列） |

---

## 11. 未验证 / 独立性声明

**独立性声明**：我是 T6b（D1/`satisfy_turn` 归一化）与 T0/T0b（基线、夹具修复）的作者。因此：
- 对 D1 的核实**不是**独立复核（我只是不改夹具、用独立脚本重算 raw/wrap），其余 D2–D4 与三处代码级改动我不是作者；
- 本报告的所有结论都给了可重跑命令，lead 若要求更强的对抗性，请换一位非 T0/T6b 作者复核 D1。

**未验证项**：
1. `--workspace` / `trapla-viz`：按 task-25 禁令未构建、未测试（Bevy 0.20 不拉起）。
2. 「原 46/46 次候选应用到错误的脚」这个**具体计数**没有逐帧对拍（只核实了当前实现改用 `pend_dots[1]` 与行为层面）；
   要复现需要作者当时的分支快照。
3. 猫步 §④ 里的 `will_feet_cross` 命中次数、p10 等次要列未逐项重算（只算了主/辅五个）。
4. `standable.csv` 的 `normal_angle` 对 case 14 的 C++ 哈希序值：我只证明「它不是任何自然顺序」，未反推出 libstdc++ 的桶序。
5. 性能计时是 CLI 自报的墙钟（同机、无隔离）；只做了 3 次中位，未做更严格的统计。

## 附：本次复核新增/未改

- 新增：`crates/trapla-core/tests/port_verification.rs`（独立对照 + 洗牌命题 + D3 分层）、本文件。
- **未修改任何作者写的源码或测试**；未 git commit/push；未动夹具与 `data/`；未跑 `--workspace` 级 cargo。
