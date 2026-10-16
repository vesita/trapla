# T0 C++ 对照基线 — 测试现状（TEST_STATUS）

- 实测时间：2026-10-16 11:13:45 +0800
- 运行对象：`/home/vesita/coding/my/trapla-baseline`（worktree / 分支 `baseline-seeded`，已应用 `baseline/seed.patch`）
- 构建：`cmake --build build -j` → 退出码 **0**（11:13:4x 之前完成；worktree 内构建）
- 运行方式（与 `tests/run_tests.py:106` 一致）：`cd <worktree> && ./build/<name> <worktree>`
- 原始输出全文：`baseline/test_logs/<name>.out`

## 1. 退出码总表

| 测试 | 退出码 | 结论 |
|---|---|---|
| `main_test` | **0** | 通过（日志内 `main_test default 验证 通过 0 182.01`） |
| `constraints_test` | **1** | **基线即失败**（1/11 用例不符） |
| `aStar_test` | **0** | 3 个用例全通过 |
| `direction_test` | **0** | 4 个用例全通过 |
| `sequence_test` | **0** | 2 个用例全通过 |
| `walk_test` | **1** | **基线即失败** |

未打补丁的 main 工作区（`/home/vesita/coding/my/trapla/build/*_test`）在 10:55 实测得到同一组结果（退出码 0/1/0/0/0/1），因此这两个红不是 `seed.patch` 引入的。

## 2. 原始输出尾部（逐字）

### main_test
```
[2026-10-16 11:13:45.952] [INFO] main_test               default     验证     通过       0     182.01
[2026-10-16 11:13:45.952] [INFO] --------------------------------------------------------------------------------
[2026-10-16 11:13:45.952] [INFO] 总计: 验证式测试 1 个, 探索性测试 0 个
[2026-10-16 11:13:45.952] [INFO] 验证式测试失败用例数: 0
[2026-10-16 11:13:45.952] [INFO] 测试套件执行完成。所有验证式测试通过。
[2026-10-16 11:13:45.952] [INFO] === 测试完成 ===
```

### constraints_test
```
[2026-10-16 11:13:45.955] [INFO] spacing_constraint_test default     验证     失败       0       0.15
[2026-10-16 11:13:45.955] [INFO] --------------------------------------------------------------------------------
[2026-10-16 11:13:45.955] [INFO] 总计: 验证式测试 1 个, 探索性测试 0 个
[2026-10-16 11:13:45.955] [INFO] 验证式测试失败用例数: 0
[2026-10-16 11:13:45.955] [INFO] 测试套件执行完成。部分验证式测试失败。
[2026-10-16 11:13:45.955] [INFO] === 测试完成 ===
```
`log/spacing_constraint_failures.csv` 原文（已复制到 `baseline/test_logs/`）：
```
position_x,position_y,expected,actual
50,55,0,1
```

### walk_test
```
[2026-10-16 11:13:45.965] [INFO] walk_with_guide_simple_testdefault     验证     失败       0       0.23
[2026-10-16 11:13:45.965] [INFO] --------------------------------------------------------------------------------
[2026-10-16 11:13:45.965] [INFO] 总计: 验证式测试 1 个, 探索性测试 0 个
[2026-10-16 11:13:45.965] [INFO] 验证式测试失败用例数: 0
[2026-10-16 11:13:45.965] [INFO] 测试套件执行完成。部分验证式测试失败。
[2026-10-16 11:13:45.965] [INFO] === 测试完成 ===
```
`log/walk_with_guide_failures.csv` 原文：
```
test_case,error_type,expected,actual
0,2,0,2
```
同一次运行的日志正文给出实际返回值：`walk_with_guide_simple_test: 目标点(4.000000, 3.000000), 实际结果(1.000000, 1.000000)`，
即 `walk_to` 原样返回了摆动脚当前位置（`walk_test.cpp:53-55` 的 `error_type=2` = `!robot.position_check(result)`）。
`Robot::walk_to` 存在两处"静默返回当前位置"的出口：`bfs` 的 tick 用尽与目标越界（`src/robot/robot.cpp:433-434, 462-463`）。

### aStar_test / direction_test / sequence_test
三者的摘要尾部均为 `所有验证式测试通过。`，逐条状态见 `baseline/test_logs/<name>.out`（aStar: 3 条通过；direction: 4 条通过；sequence: 2 条通过）。

## 3. 必须先知道的两个测试框架陷阱（否则会误读上表）

1. **"失败数"列恒为 0，不代表没失败。** `addFailure/throwIfFailed` 用中文用例名（`constraints_test.cpp:30,53`），而 `runTests` 用宏名做 `clearFailures/failureCount`（`include/utils/test_framework.hpp:241,278`）。两者不是同一个 key，所以摘要里 `失败数=0` 却 `状态=失败`。
2. **"非验证式测试"能力目前是空转的。** `TestType::EXPLORATORY` 与 `setRunExploratoryTests` 都存在（`test_framework.hpp:35,170`），但没有一个测试注册成 EXPLORATORY（`tests/` 内零命中），默认 `runExploratoryTests=false`（`test_framework.hpp:323`），摘要恒为 `探索性测试 0 个`。`main_test` 用的是普通 `addDataRecord`，不会被跳过。

## 4. constraints_test 唯一失败用例的裁决（T4 依赖这条结论）

**问题**：`constraints_test.cpp:16` 的 `{SqDot(50,55), false}` 实测得到 `satisfy_spacing = true`。
**是测试期望错，还是实现错？** 判据是手算几何距离 + 文档区间语义。

### 4.1 手算（读码 + 复算，几何部分已由 golden 实测复核）

配置：`Robot(40, 75°, max_sep=10, min_sep=2, foot_len=5, foot_wid=3)`；`feet[0]=support=(50,50), rz=0`，`feet[1]=swing=(50,45), rz=0`，`now=Right`。
`new_pos=(50,55)`：

1. `Robot::next`：stride = |(50,45)-(50,55)| = 10 < `active_distance` = 40*0.4 = 16 → 脚**不转向**，`rz=0`（`robot.cpp:19,635-646`）。
2. `Foot::corner()`（`foot.cpp:217-236`，半长 2.5、半宽 1.5）：四角 = (52.5,56.5)、(52.5,53.5)、(47.5,53.5)、(47.5,56.5)。
3. 支撑脚"中心线"`SqLine(support.position, support.rz=0)` = 过 (50,50)、方向 (1,0) 的直线，即 `y = 50`（`geometry.cpp:373-374`）。
4. 四角到该线的最小垂距 = |53.5 - 50| = **3.5**。
5. `spacing = max(0, 3.5 - width/2) = 3.5 - 1.5 = 2.0`（`robot.cpp:201-214`）。
6. 支撑脚近侧长边（面向摆动脚那一侧）= `y = 51.5`；最近角点 (52.5,53.5) 到它的垂距 = **2.0**。→ 第 5 步的"中心线减半宽"与文档定义的"到近侧外形线的距离"在这一例中是**等价的**。

**结论：几何计算本身没错，`spacing` 确实精确等于 2.0。**

### 4.2 分歧只在边界比较符号

- 代码：`epsilon = 1e-3; min_allowed = 2 - 1e-3; max_allowed = 10 + 1e-3; return spacing >= min_allowed && spacing <= max_allowed;`（`robot.cpp:217-221`）→ **闭区间 + 1e-3 容差**，`spacing == 2.0` 判 **true**。
- 文档：`doc/project.md:50` "双足间距：**大于2**且小于10"；`doc/project.md:58`、`readme.md:25` 同口径 → **开区间 (2,10)**，`spacing == 2.0` 应判 **false**。
- `constraints_test.cpp:16` 的期望值跟文档一致。

**裁决（实测 + 手算）**：**不是测试期望错；是实现在下界处把开区间放宽成了带容差的闭区间。** 测试期望忠于文档，实现偏离文档。

### 4.3 golden 实测的容差边界（`baseline/golden/satisfy_spacing.csv`）

同一 (50,50)/(50,45) 配置下，`spacing` 与判定实测：

| case | newy | spacing | C++ 判定 | 说明 |
|---|---|---|---|---|
| 0 | 55.0 | 2.000000 | true | constraints_test 唯一失败用例 |
| 47 | 15.0 | 2.000000 | true | 另一配置的"恰好 2" |
| 49 | 54.999 | 1.999000 | **true** | 小于 2 却被接受 → 容差存在 |
| 50 | 54.9985 | 1.998500 | false | 容差外 |
| 51 | 54.998 | 1.998000 | false | |
| 48 | 23.0 | 10.000000 | true | 恰好 10 |
| 52 | 64.0005 | 10.000500 | true | 大于 10 却被接受 |
| 53 | 64.001 | 10.001000 | **false** | 期望上界内，浮点误差使其落到界外 |
| 54 | 64.0011 | 10.001100 | false | |

即实现实际接受区间是 `[2-1e-3, 10+1e-3]`（含端点，且上界因浮点表示并不精确），文档要求 `(2, 10)`。

### 4.4 对 T4 验收口径的影响（不由本任务决定，但判据在此）

- 若按文档采纳**开区间 (2,10)**：`constraints_test` 的 11 组期望值全部可作为 Rust 回归测试；C++ 基线的这一处判定是**已知红**，Rust 侧应判 false，需在 T4 显式登记为"与 C++ 基线的有意分歧"。
- 若选择保留容差的闭区间（数值健壮性）：必须改文档，并把 `{SqDot(50,55), false}` 改成 `true`——但那是改期望去迁就实现，需明确记录理由。
- 两个方向都必须显式决定；**不要默认站在测试这边，也不要默认站在实现这边**。判据是上面的手算与 golden 实测。

## 5. 与 seed.patch 的关系

`constraints_test` / `walk_test` 的失败与 `seed.patch` 无关：
- `constraints_test` 不调用 `scale_star`，只用 `satisfy_spacing`；
- `walk_test` 只用 `Ground(5,7)` + `walk_to`，不读 `map.csv`；
- 未打补丁的 main 构建在 10:55 给出同样 0/1/0/0/0/1。

`seed.patch` 只影响 `scale_star`（`aStar.cpp:100` → `Intex::get_neighbour_shuffled(int,int)`），它影响的是 `main_test`/`trapla` 的路径结果，而这两个在补丁前后都退出 0。
