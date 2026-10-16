# C++ 对照基线（冻结记录）

重写 Rust 之前，先在 C++ 原实现上冻结了一份可复现的对照基线：把随机源换成固定种子、导出函数级
golden 数据、记录测试现状与猫步指标。Rust 版据此逐条对齐。

**基线不在工作区，保存在 git 修订 `0038315`。** 读取任意文件：

```bash
git show 0038315:baseline/<路径>
git ls-tree -r --name-only 0038315 -- baseline     # 全部 59 个文件
```

C++ 的确定性化补丁在独立 git worktree `/home/vesita/coding/my/trapla-baseline`（分支 `baseline-seeded`）
里应用并构建，`main` 分支的 C++ 源码未被改动。

## 为什么基线需要固定种子

原实现 `scale_star` 每步用 `std::random_device` 洗牌邻居（`src/aStar/aStar.cpp:100`），
**同一输入两次运行会给出不同路径**（实测 68 点 vs 91 点；另一组 107 点 vs 52 点）。
不固定种子就无法做任何对照。确定性化只在 `Intex::get_neighbour_shuffled` 把随机源换成
固定种子的 `std::mt19937`，默认 seed=1、可用环境变量 `TRAPLA_SEED` 覆盖；算法、阈值、语义未动。

## 内容清单

| 路径 | 作用 |
|---|---|
| `baseline/seed.patch` | 唯一一处 C++ 改动（+16/−2），`git apply` 可回 main |
| `baseline/frozen/` | 3 seed × 2 输入 × 2 次的路径结果与 `sha256sum.txt`；`unpatched/` 是未打补丁的对照 |
| `baseline/frozen/crash_evidence.txt` | 40 组 (seed,输入) 里 8 组 `exit=134` 的断言原文 |
| `baseline/frozen/point_counts.txt` | 各组点数 |
| `baseline/golden/` | 19 类纯函数的输入→输出 CSV（每行带输入列），另有 `golden_summary.txt` |
| `baseline/harness/golden_dump.cpp` | 生成 golden 的 C++ harness |
| `baseline/harness/catwalk_probe.cpp` | 带出逐点脚身份的探针 |
| `baseline/tools/catwalk_metrics.py` | 猫步指标计算（口径的唯一事实源在其 docstring） |
| `baseline/catwalk_metrics.json` | 6 个 (seed, 目标) 用例的猫步指标基线与 `separation_lateral_sequence` |
| `baseline/catwalk_parity_scan.txt` | 奇偶分足假设的验证（20 seed × 2 目标） |
| `baseline/catwalk_foot_probe.txt` | 探针带出的逐点脚身份 |
| `baseline/TEST_STATUS.md` | 6 个测试的原始输出尾部与退出码 |
| `baseline/README.md` | 从零复现的完整命令 |

函数级 golden 夹具已迁到 **`crates/trapla-core/tests/golden/`**（22 个文件），作为测试夹具长期保留，
不通过 git 历史读取。

## 基线上的三条实测结论

1. **测试基线本就是红的**：`constraints_test` 与 `walk_test` 失败，其余 4 个通过。未打补丁的构建
   同样如此，两处失败不是补丁引入的。细节与失败 CSV 原文见 `TEST_STATUS.md`。
2. **固定种子暴露真实越界崩溃**：40 组里 8 组 `exit=134`，断言为
   `stl_vector.h:1253: std::vector<SqDot>::operator[]: Assertion '__n < this->size()' failed`。
   根因是 `Robot::find_target` 在候选表为空时仍取 `candidates[0]`（`src/robot/robot.cpp:321-331`）。
   随机源下只是没被踩到。Rust 版显式返回 `CoreError::NoPath`，登记为有意修正。
3. **`satisfy_spacing` 的实现偏离文档**：手算 `(50,55)` 的间距恰为 2.0，几何计算无误；实现用
   `spacing >= 2-1e-3 && spacing <= 10+1e-3`（`src/robot/robot.cpp:217-221`）判 `true`，
   而 `doc/project.md:50` 写的是「大于 2 且小于 10」。golden 实测边界：`2.000→true`、`1.999→true`、
   `1.9985→false`、`10.000→true`、`10.0005→true`、`10.001→false`。
   移植阶段按实现对齐，语义分歧留待单独裁决。

## 复现

```bash
# 1. 取回补丁与 harness
git show 0038315:baseline/seed.patch > /tmp/seed.patch
git worktree add /tmp/trapla-base 0038315
cd /tmp/trapla-base && git apply /tmp/seed.patch

# 2. 构建
cmake -S . -B build && cmake --build build -j

# 3. 固定 seed 跑两次，验证可复现（应输出相同 sha256）
TRAPLA_SEED=3 ./build/trapla   # 需要 exe 同级 ../data/csv/map.csv

# 4. 重新生成 golden（harness 与命令见 baseline/README.md）
```

`data/` 是原仓就跟踪的输入数据；重新生成地图用 `uv run trapla-data gen-map --seed <n> --out <路径>`
（见 `python/README.md`）。

## 打算怎么改基线的对照口径

基线记录的是 C++ 的行为，包含它自己的缺陷。Rust 版与基线的分歧分两类，处理方式不同：

- **有意的修正**（如空候选崩溃、`satisfy_spacing` 的区间语义）：逐条登记理由与影响面，
  在对照测试里显式列出、不靠放宽容差掩盖。
- **移植偏差**：属于缺陷，必须修到对齐。

登记表在 `doc/catwalk_diagnosis.md` 与各移植任务在共享任务板上留下的报告里。
