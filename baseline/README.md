# T0 C++ 对照基线（确定性化 + 函数级 golden + 猫步指标）

本目录是 trapla 重写 Rust 之前的 **C++ 对照基线**。它只做两件事：把 `scale_star` 的随机源钉死，
以及把与随机无关的纯函数输出冻结成可逐条比对的 CSV。**不改算法、不改阈值、不改语义**。

- 基线 commit：`b53543b`（`refs/heads/main`，仓库根 `/home/vesita/coding/my/trapla`）
- worktree：`/home/vesita/coding/my/trapla-baseline`，分支 `baseline-seeded`
- 唯一的 C++ 改动：`baseline/seed.patch`（`src/utils/index.cpp`，+16/-2）
- 本目录是**待入库**的证据目录。`baseline/.gitignore` 只排除 harness 的编译产物
  （`/harness/golden_dump`、`*.o` 等），`.csv`/`.json`/`.patch`/`.md`/`.cpp`/`.py`/`.txt` 与
  `test_logs/*.out`（原始测试输出）都保持可入库。

> **关于 `git status --porcelain`**：T0 期间同一 main 工作区有并发的 T1（Rust workspace 骨架）在写
> `Cargo.toml` / `Cargo.lock` / `rust-toolchain.toml` / `.gitignore` / `crates/`。因此该命令不为空：
> 其中 `?? baseline/` 是 T0 的证据目录（预期如此），其余是 T1 的产物。
> 校正后的判据是「**T0 引起的改动只出现在 `baseline/` 与 worktree 内**」，判别方法见第 7 节。

---

## 1. 从零复现（逐条可复制）

以下命令假定仓库根为 `$MAIN`，worktree 为 `$WT`：

```bash
MAIN=/home/vesita/coding/my/trapla
WT=/home/vesita/coding/my/trapla-baseline
```

### 1.1 建 worktree 并打确定性化补丁

```bash
git -C "$MAIN" worktree add "$WT" -b baseline-seeded
git -C "$WT" apply "$MAIN/baseline/seed.patch"
git -C "$WT" diff --stat          # 应只有 src/utils/index.cpp | 18 ++++++++++++++++--
```

补丁做的事（`src/utils/index.cpp`）：

- 新增 `baseline_rng_seed()`：默认返回 `1`，可用环境变量 `TRAPLA_SEED` 覆盖；
- 把 `Intex::get_neighbour_shuffled(int, int)` 里的 `std::random_device rd; std::mt19937 g(rd);`
  换成 `static std::mt19937 g(baseline_rng_seed());`。

这是 `scale_star` 路径上**唯一**的随机调用点：`src/aStar/aStar.cpp:100` 的
`current.get_neighbour_shuffled(sr, sc)`。其余 7 处 `get_*_shuffled` 在生产链路中无调用（grep 可复核）。

### 1.2 构建

```bash
cmake -S "$WT" -B "$WT/build" && cmake --build "$WT/build" -j
```

### 1.3 冻结运行（同 seed 两次、sha256 相同）

```bash
mkdir -p /tmp/frozen/seed-3/data/csv /tmp/frozen/seed-3/bin
cp "$WT/build/trapla" /tmp/frozen/seed-3/bin/
cp "$MAIN/data/csv/map.csv" /tmp/frozen/seed-3/data/csv/
cd /tmp/frozen/seed-3/bin
TRAPLA_SEED=3 ./trapla > /dev/null 2>&1            # 默认 (0,0)->(500,500)
cp ../data/csv/path_result.csv /tmp/frozen/seed-3/a.csv
TRAPLA_SEED=3 ./trapla > /dev/null 2>&1
cp ../data/csv/path_result.csv /tmp/frozen/seed-3/b.csv
sha256sum /tmp/frozen/seed-3/a.csv /tmp/frozen/seed-3/b.csv   # 两行必须相同
```

**注意**：`trapla` 的起点参数是死代码——`main.cpp:68-69` 读了 `start` 却从未传给 `find_path`
（只有 `goal` 生效，见 `main.cpp:85`）。所以"输入"实际就是 goal：默认 `(500,500)`、备用 `(900,1200)`。
另外每次运行前要删掉 `data/csv/path_result.csv`，否则崩溃的 run 会留下上一次的残留文件。

### 1.4 golden 导出（从仓库根，只读编译）

```bash
cd "$MAIN"
g++ -std=c++17 -I include baseline/harness/golden_dump.cpp \
    src/utils/*.cpp src/ground/*.cpp src/robot/*.cpp src/aStar/*.cpp src/csv/*.cpp \
    -lm -o /tmp/golden_dump
/tmp/golden_dump baseline/golden > baseline/golden/golden_summary.txt
```

实测：编译退出码 0（11:11:02），运行退出码 0（11:11:34），行数清单见 `baseline/golden/golden_summary.txt`。
注意编译命令比"最小形式"多了 `src/csv/*.cpp`：`ground.cpp` 里的 `Ground(std::string)` 引用 `CSVReader`，
即使 harness 只用 `Ground(int,int)`，链接期仍需 `csv/reader.o`。

### 1.5 猫步指标

```bash
cd "$MAIN"
python3 baseline/tools/catwalk_metrics.py       # 写 baseline/catwalk_metrics.json 并打印摘要
```

### 1.6 跑测试

```bash
for t in main_test constraints_test aStar_test direction_test sequence_test walk_test; do
  ( cd "$WT" && ./build/$t "$WT" > /tmp/$t.out 2>&1 ); echo "$t EXIT=$?"
done
```

预期：`0 1 0 0 0 1`（`constraints_test`、`walk_test` 基线即红，见 `baseline/TEST_STATUS.md`）。

### 1.7 复现猫步分足探针（可选，需要临时插桩）

```bash
git -C "$WT" apply "$MAIN/baseline/harness/catwalk_probe_instrumentation.patch"
cp "$MAIN/baseline/harness/catwalk_probe.cpp" "$WT/catwalk_probe.cpp"
cd "$WT" && g++ -std=c++17 -I include catwalk_probe.cpp \
    src/utils/*.cpp src/ground/*.cpp src/robot/*.cpp src/aStar/*.cpp src/csv/*.cpp \
    -lm -o /tmp/catwalk_probe
cd "$MAIN" && TRAPLA_SEED=3 /tmp/catwalk_probe 500 500
# 复原插桩（robot.cpp 必须回到只有 seed.patch 的状态）
git -C "$WT" checkout -- src/robot/robot.cpp
rm -f "$WT/catwalk_probe.cpp"
```

`catwalk_probe_instrumentation.patch` 给 `find_path` 加了一个 `g_trapla_foot_trace`（每个 path 点由哪只脚产生）
和一个 `g_trapla_will_cross_hits` 计数器；它**不是** `seed.patch` 的一部分，也不改算法。

---

## 2. 撤销

```bash
# 只撤销 C++ 改动（worktree 内）
git -C "$WT" checkout -- src/utils/index.cpp

# 或反向应用补丁
git -C "$WT" apply -R "$MAIN/baseline/seed.patch"
```

**忽略规则**：产物规则在 `baseline/.gitignore`，只排除 harness 编译产物；删掉该文件即可让产物也可见。
`.git/info/exclude` 不参与本目录的忽略（曾有一版本地屏蔽，已撤除）。

**完全拆除 worktree**（不影响 main 分支）：

```bash
git -C "$MAIN" worktree remove "$WT"
git -C "$MAIN" branch -D baseline-seeded
```

---

## 3. 目录清单

| 路径 | 内容 |
|---|---|
| `seed.patch` | 唯一的 C++ 改动；`git apply` 可回 main（已 `git apply --check` 通过） |
| `frozen/` | 3 seed × 2 输入 × 2 次运行的 `path_result.csv` + `sha256sum.txt` + `point_counts.txt`；`unpatched/` 是对照组；`crash_evidence.txt` 是确定性崩溃证据 |
| `golden/` | 函数级 输入→输出 CSV（见第 4 节）+ `golden_summary.txt`（每文件行数） |
| `harness/golden_dump.cpp` | golden 导出器源码（唯一事实源） |
| `harness/catwalk_probe.cpp` + `..._instrumentation.patch` | 分足探针源码与临时插桩 |
| `tools/catwalk_metrics.py` | 猫步指标脚本（指标定义写在 docstring 里） |
| `catwalk_metrics.json` | 6 条冻结路径的猫步指标基线 |
| `catwalk_parity_scan.txt` | 20 seed × 2 goal 的分足假设扫描（含崩溃与 `will_feet_cross` 命中数） |
| `catwalk_foot_probe.txt` | 两个代表运行的逐点原始输出 |
| `TEST_STATUS.md` | 6 个测试的退出码、原始输出尾部、失败 CSV 原文、`satisfy_spacing` 边界裁决 |
| `test_logs/` | 6 个测试的完整 stdout/stderr + 2 个失败 CSV |

---

## 4. golden 覆盖与最低行数（实测，见 `golden/golden_summary.txt`）

| 文件 | 行数/组数 | 覆盖的函数 |
|---|---|---|
| `grounds.csv` | 387 | 7 张显式地面（含全平/斜面/起伏/整列 inf/LCG 噪声/负值障碍/走廊） |
| `normal_angle.csv` | 45 | `CuPlane::normal_angle` |
| `up_fit.csv` | 45 | `up_fit`（同时导出被原地修改的 `three`） |
| `trip.csv` | 45 | `Ground::trip` |
| `cover.csv` | 40 组 / 1072 行 | `Foot::cover`（按 (x,y) 升序导出，Rust 按集合比较） |
| `corner.csv` | 40 | `Foot::corner` |
| `standable.csv` | 45 | `Foot::standable`（含 `covered_count`/`all_inside`/`normal_angle` 中间量） |
| `slide.csv` | 45 | `FootShape::slide`（输入区、结果枚举、输出区） |
| `satisfy_spacing.csv` | 55 | `Robot::satisfy_spacing`（前 11 组 = `constraints_test.cpp:14-26` 原样） |
| `satisfy_stride.csv` | 40 | `Robot::satisfy_stride`（含 distance 恰为 40 的严格小于边界） |
| `satisfy_turn.csv` | 42 | `Robot::satisfy_turn`（含 ±max_turn 与角度回绕） |
| `restore.csv` | 40 | `SqPlane::restore`（Intex 与 SqDot 两种 mode） |
| `restore_dot.csv` | 60 | `SqPlane::restore_dot` |
| `central_restore.csv` | 70 | `Intex::central_restore` / `SqDot::central_restore` |
| `index_scale.csv` | 220 | `index_scale(int)` / `index_scale(double)` / `scale(double)` / `Intex::sx,sy` |
| `scale_graph.csv` + `scale_graph_cells.csv` | 35 组 / 5217 行 | `SqPlane::scale_graph`（5 张有限地面 × 7 个 scale） |
| `steep_extend.csv` | 46 | `steep_extend`（含越界、inf 占比 ≥0.3、单点） |
| `geometric_decay.csv` | 76 | `geometric_decay(alpha,n)` / `geometric_decay(start,alpha,n)` |
| `direction_determine.csv` + `direction_guides.csv` | 40 组 / 110 行 | `direction_determine`（SqDot 与 Intex 两种 mode；guides 单独成文件） |

每行都带完整输入列；`Ground` 的每个格子在 `grounds.csv` 里逐格导出，Rust 侧读同一份，不要复现高度公式。

### golden 的已知边界与陷阱（写进 golden 的语义约束）

1. **`SqDot::scale` 有 const / 非 const 两个同名重载，语义不同**：const 版返回 `x*scale`（浮点），
   非 const 版返回 `ceil(x*scale)`（整数，`geometry.cpp:116-118` vs `238-240`）。`SqPlane::restore` 在同一函数里
   混用两者（`geometry.cpp:919-929`）。`restore.csv` 用 `mode` 列区分 `int`/`dot`，Rust 必须对齐到一致的取整语义。
2. **`Foot::cover` 的返回顺序未定义**（来自 `unordered_set` 迭代器，`foot.cpp:214`）。`cover.csv` 按 (x,y)
   升序导出；Rust 必须按集合比较，不能按序。
3. **inf / nan 行**：`steep_extend.csv` 有意包含 `infinity` 障碍格（它显式处理），`standable.csv`/`slide.csv`/`trip.csv`
   已排除 inf 地面，避免 `static_cast<int>(nan)`（UB）进入 golden。
4. **`index_scale` 两个重载实现完全相同**（都是 `ceil`，`scale.cpp:12-18`），保留两行是为了对齐签名。
5. 浮点全部以 `setprecision(17)` 输出，可比对到 1e-9；`nan`/`inf` 以字面量写出。

---

## 5. 冻结结果（实测）

同一 seed 连跑两次 sha256 相同（`frozen/sha256sum.txt` 原文）：

```
c48c8b853f5fb0677ac1322bb530a04b61390b255ff00ac1ac7f9c49af236fca  seed-3_alt_run1.csv
c48c8b853f5fb0677ac1322bb530a04b61390b255ff00ac1ac7f9c49af236fca  seed-3_alt_run2.csv
166e8ecd926bdb62b30ade15d94a832e761c14939601dfb3cf2ad536213f4782  seed-3_default_run1.csv
166e8ecd926bdb62b30ade15d94a832e761c14939601dfb3cf2ad536213f4782  seed-3_default_run2.csv
e0e0fa7e6ae879e324b7e187788806d855da5fb787efebd747e9185be81e5b4e  seed-6_alt_run1.csv
e0e0fa7e6ae879e324b7e187788806d855da5fb787efebd747e9185be81e5b4e  seed-6_alt_run2.csv
38749ea3b985bfa62a64bd943c723ac3c74cbbed4c2f5206ac7a0e3a82a37111  seed-6_default_run1.csv
38749ea3b985bfa62a64bd943c723ac3c74cbbed4c2f5206ac7a0e3a82a37111  seed-6_default_run2.csv
f37164164e297e66f976196e71c584c62bb585f6ebafaf513371291f338e347f  seed-7_alt_run1.csv
f37164164e297e66f976196e71c584c62bb585f6ebafaf513371291f338e347f  seed-7_alt_run2.csv
8afd40c63f726f358fc2f37878ac5444f60de7f0ab8c2dad94b3dbf5f510614e  seed-7_default_run1.csv
8afd40c63f726f358fc2f37878ac5444f60de7f0ab8c2dad94b3dbf5f510614e  seed-7_default_run2.csv
```

未打补丁的对照组（`frozen/unpatched/sha256sum.txt`）两次运行不同：

```
38749ea3b985bfa62a64bd943c723ac3c74cbbed4c2f5206ac7a0e3a82a37111  unpatched/run1_path_result.csv
1ca046740ec79b69bdd78e9b3d1459c0c67195e77ad9d5692c2bfd0803965dc4  unpatched/run2_path_result.csv
```

点数：未打补丁同一默认输入两次得到 **107 与 52**（`frozen/point_counts.txt`；早前一轮为 68 与 91）——
补丁后同一 (seed, 输入) 两次点数完全相同。

### 固定 seed 会暴露一个真实的越界崩溃

`frozen/crash_evidence.txt`：8/40 个 (seed, input) 组合以 **exit=134** 终止，断言为
`std::vector<SqDot>::operator[]: Assertion '__n < this->size()' failed`。
读码定位：`Robot::find_target` 在 `walk_candidates()` 返回空表时仍执行 `return candidates[0];`
（`src/robot/robot.cpp:321-331`，调用链 `walk_to -> get_target -> find_target`）。
随机源下这条路径只是"碰巧没踩到"；钉死 seed 后它变成可复现的失败——**Rust 侧必须显式处理空候选**。

## 6. 猫步指标与分足假设

- **分足规则（实测）**：`baseline/catwalk_parity_scan.txt` 的 32 个未崩溃运行里，`path` 的脚标签**严格交替**、
  `first_foot` 恒为 0 → **偶下标 = `feet[0]` = 左脚，奇下标 = `feet[1]` = 右脚**，`plot_path.py` 的
  `[::2]/[1::2]` 假设成立。另 8 个组合崩溃（见上）。
- `will_feet_cross` 在这些运行里命中 2~25 次（`catwalk_parity_scan.txt` 最后一列），说明"avoid 分支被走到"
  也不破坏奇偶分足——因此当前**不需要**改用 `will_feet_cross` 语义重定义分足。
- 指标定义写在 `tools/catwalk_metrics.py` 的 docstring（唯一事实源），结果在 `catwalk_metrics.json`。
  注意 `foot_separation_sequence` 是**足中心距**代理，与 C++ `satisfy_spacing` 的"四角到支撑脚中心线垂距减半宽"
  不是同一个量；两者的差值恰恰是 T4 要钉住的契约之一。
- 实测摘要（`seed / n / 总长 / 终点误差 / 间距 min-mean-max / 越界数 / 折线交点数`）：

```
seed=3 default n= 50 len=1090.290 goal_err= 20.982 sep=5.799/22.251/38.000 viol2_10=45 inter=4
seed=3 alt     n=130 len=2853.068 goal_err= 40.099 sep=6.093/22.117/40.002 viol2_10=126 inter=10
seed=6 default n=107 len=2067.675 goal_err=187.358 sep=9.543/19.506/40.000 viol2_10=104 inter=5
seed=6 alt     n=140 len=2897.402 goal_err=698.430 sep=6.089/20.845/40.000 viol2_10=129 inter=3
seed=7 default n= 58 len=1193.749 goal_err= 17.105 sep=7.874/20.943/40.000 viol2_10=55 inter=4
seed=7 alt     n=132 len=2906.293 goal_err= 27.280 sep=6.089/22.185/53.961 viol2_10=124 inter=10
```

终点误差按"最后一个点 vs goal"算；因为 `reach_target` 用 `active_distance=16` 作阈值（`robot.cpp:466-468`），
误差本就不保证为 0。

## 7. 如何确认 T0 的改动范围

```bash
cd /home/vesita/coding/my/trapla
git check-ignore -v baseline/README.md             # 期望：退出 1、无输出（证据不被 ignore）
git check-ignore -v baseline/harness/golden_dump   # 期望：命中 baseline/.gitignore 的产物规则
git status --porcelain | grep baseline             # 期望：?? baseline/
git status --porcelain                             # 其余条目属于并发的 T1
git worktree list                                  # 期望：main 与 trapla-baseline@baseline-seeded
```

T0 只写了三处：worktree（另立分支 `baseline-seeded`）、`baseline/`（证据）、以及 worktree 内的
`src/utils/index.cpp`（即 `seed.patch`，不回写 main）。main 工作区里 T0 **没有**改动任何被跟踪文件。
若 `git status` 出现 `Cargo.toml` / `Cargo.lock` / `rust-toolchain.toml` / `crates/` / `M .gitignore`，
那是并发的 T1 产物。
