# NoPath 诊断（T6d，只读）：51 个与归一化无关的 `exit=2` 是缺陷还是真不可达

- 任务：`task-21`。**只读**，本文件是唯一写入物；未修改任何代码、未跑任何 `cargo` 命令、未 git commit。
- 冻结二进制：`/tmp/nopath/trapla`
  `sha256 = 41ddc8bf1408a9a882d64041e4ecfd555099f8d7157ae0f92d5936e084dff100`
  （与 task-19 的 `/tmp/t6b/trapla_fixed`、`trapla_fixed_final` 同 hash ⇒ 与 `doc/known_divergences.md` 的 `post` 列同一测量对象）
- **本文件所有 e2e 数字都来自这个 sha256 的二进制**，命令一律写全路径，不用 `cargo run`。
- 读码证据基于冻结源快照：`/tmp/t6b/robot_fixed.rs`。动手冻结（13:11）时它与当时的 `crates/trapla-core/src/robot.rs` 逐字节相同（`diff -q` 通过）；**冻结之后 `robot.rs` 已被 T6c 改动**，因此本报告的层定位与行号都锚在「冻结快照 + sha256 `41ddc8bf...` 的二进制」上，对新版 `robot.rs` 只按语义重新核对、不再按行号引用。

---

## 0. 扫描口径（从哪里来、怎么复现）

120 = 4 起点 × 10 目标 × 3 seed（seed 0/1/2）：

- 起点：`(0,0)`、`(100,100)`、`(500,500)`、`(700,700)`
- 目标：`(900,1200)`、`(800,800)`、`(1000,500)`、`(500,1000)`、`(1200,900)`、`(900,300)`、`(300,900)`、`(1500,1500)`、`(200,1500)`、`(1500,200)`

组合清单取自 task-19 的产物 `/tmp/t6b/paired/rows.txt`（120 行，列 = start goal seed exit_pre exit_post）。地图：`data/csv/map.csv`，**1000 行 × 3000 列**（`--start/--goal` 的 `x` 是行，`y` 是列），高度值域 `[27, 123]`，无 `inf`、无负值。

用冻结二进制重跑全部 120 组合（`/tmp/nopath/scan.csv`），与 `rows.txt` 的 `post` 列逐一比对：

```
$ awk '{c[$4"->"$6]++} END{for(k in c) print k, c[k]}' /tmp/nopath/scan.csv
0->0 69
2->2 51
```

**120/120 一致**，其中 **51 个 `exit=2`**（本报告的对象）、69 个 `exit=0`。

---

## 1. 失败分层（51 个全部实测，不是抽样）

### 1.1 结论表

| 层 | 出口（冻结快照行号） | 个数 | 判定方式 |
|---|---|---|---|
| L1 高层 A* 找不到：`scale_star` 返回 `None` | `robot.rs:414-424` 的 `.ok_or(NoPath)?` | **48** | 探针直接调用 `scale_star`，全部得到 `None` |
| L2 `find_target` 候选集为空 | `find_path:447/454` → `walk_to:362` → `get_target` → `find_target:268-275` | **3** | 探针：`scale_star = Some`，逐步复刻循环后在 `avoid_feet_cross` 内命中空候选；L1 已被排除 ⇒ 只能是 L2 |
| `walk_to` 的 bfs 兜底 | `bfs:294-351` 返回 `Dot`，**从不返回 `Err`** | **0** | 读码：`bfs` 在目标越界（`306-309`）或 tick 耗尽（`319-321`）时静默返回摆动脚原位置，不产生 `NoPath` |
| tick 耗尽 | `find_path:428,438,470` | **0** | 3 个 L2 在第 15/16 步就死了（tick 预算 192）；48 个 L1 根本没进循环 |
| `find_path` 返回 `Ok` 但 `path` 为空 | `trapla-cli/src/plan.rs:64-67` | **0** | `scale_star` 返回的 `path` 至少 2 点；3 个 L2 的 `pos` 距目标 949 单位，不可能一开始就 `reach_target` |
| 其它 | — | **0** | — |

合计 48 + 3 = 51。

### 1.2 判定方法（可复现）

CLI 只打印 `错误: 规划失败：无路径 (seed N)`，**不区分层**，`--metrics` 在失败时也不落盘 ⇒ 单靠 e2e 无法归因。于是做了一个**只读探针**：

- 文件：`/tmp/nopath/probe.rs`（不在仓内，不改任何仓内文件）
- 编译：`rustc --edition 2024 -O -L dependency=<target/debug/deps> --extern trapla_core=/tmp/nopath/libtrapla_core-30f42f7f42cbed76.rlib probe.rs -o probe`
  （rlib 是在动手时**复制到 /tmp 的冻结副本**，`sha256=565372e43e0a...`；`-L dependency` 只读，不写 target）
- 探针做两件事：① 直接调用 `crate::astar::scale_star` 取 `Some/None`；② 逐行复刻 `robot_fixed.rs:426-478` 的 `find_path` 主循环与 `avoid_feet_cross:436-459`，每步入队前打印 `walk_candidates().len()`，并做一次交叉验证（另起 `Robot::find_path` 对拍）。

对全部 120 组合跑「只要 `scale_star`」模式：

```
$ awk ... /tmp/nopath/scaleonly.txt
frozen0 & scale_star=Some: 69
frozen2 & scale_star=None: 48
frozen2 & scale_star=Some(=>L2): 3
其他: (无)
```

对 51 个失败组合跑完整探针：

```
$ grep -c "L1" /tmp/nopath/probe_all.txt   → 48
$ grep -c "L2" /tmp/nopath/probe_all.txt   → 3
$ grep -v "L1\|L2" /tmp/nopath/probe_all.txt → (空)
```

⇒ **48 个 L1、3 个 L2，无其它层**。

---

## 2. 独立于被测实现的可达性判定

### 2.1 口径（只吃 `map.csv` + `steep_extend` 判死规则）

脚本 `/tmp/nopath/reach.py`，不读被测实现的任何内部状态：

1. 高度表直接从 `data/csv/map.csv` 读入（1000×3000）。
2. `steep_extend(fi,se)` 按本仓语义独立重写（含 `aStar.cpp:212` 的有意修正 `min(first.x, second.x)`）：`obstacle(+inf) 占比 >= 0.3`、`heights` 空、单点、`0.4*stddev+0.3*diff+0.3*slope > 10` 四条判死。
3. 缩放口径与 `robot.rs:414-419` 一致：`scale = 1/40 = 0.025`，`sr = ceil(1000*0.025) = 25`、`sc = ceil(3000*0.025) = 75`；
   `restore_block((bx,by)) = (clamp(ceil(bx*0.025)), clamp(ceil(by*0.025))) → (clamp(ceil((bx+1)*0.025)), clamp(ceil((by+1)*0.025)))`（同 `grid.rs::restore_block`）。
4. 缩放节点 `next` **可进入 ⟺ `steep_extend(restore_block(next)) != None`**；起点 `ss` 无条件入队；四邻域 BFS；`sg ∈ BFS` ⟺ 判定「高层可达」。

为什么 BFS 可达 ⟺ `scale_star` 返回 `Some`：`scale_star` 的 A* 在非负代价下完备，`rng.shuffle` 只改探索顺序、不改可达性。

**口径验证（实测）**：该独立模型与探针实际调用 `scale_star` 的 `Some/None` 在 **120/120** 上一致：

```
独立缩放 BFS 可达 ⟺ 探针 scale_star=Some 一致: 120 / 120  不一致: []
```

### 2.2 对照结果

| 规划器（冻结二进制） | 独立缩放 BFS | 个数 | 解释 |
|---|---|---|---|
| `exit=0` | 可达 | 69 | 一致 |
| `exit=2` | 不可达 | 48 | 一致：`sg` 根本不在缩放网格内 |
| `exit=2` | **可达** | **3** | **说不通 ⇒ 缺陷候选** |

3 个缺陷候选全是 `(0,0) → (900,300)`（seed 0/1/2）：`sg` 缩放后 `(23,8)` 在 `[0,25)×[0,75)` 内，且该图 **1875/1875 个缩放节点的 `steep_extend` 全部通过**（见 §5.2），所以高层必然返回 `Some`——探针证实 `Some(path_len=32, expanded=533)`。

**必须写明的边界**：上面判定的是「在规划器自己的**粗粒度抽象**下可达」。它**不证明**在完整足步约束（`satisfy_spacing/stride/turn` + `standable`）下一定存在可行落足序列——那需要一套独立的足步状态空间搜索，本任务没做。按 task-21 的定义（「规划器说不可达且独立判定也说不通」）这 3 个记为**缺陷候选**；「是否物理上真的可达」列为本报告**未判定**项（§7）。

---

## 3. 按目标分桶

（分桶基于 51 个失败组合；「越界」= `x >= 1000` 或 `y >= 3000`）

| 目标 | 失败/总 | 越界 | 层 |
|---|---|---|---|
| `(1500,1500)` | 12/12 | x=1500 越界 | L1 |
| `(1500,200)` | 12/12 | x=1500 越界 | L1 |
| `(1200,900)` | 12/12 | x=1200 越界 | L1 |
| `(1000,500)` | 12/12 | x=1000 越界 | L1 |
| `(900,300)` | 3/12 | 否 | L2（仅起点 `(0,0)`） |
| `(900,1200)` | 0/12 | 否 | — |
| `(800,800)` | 0/12 | 否 | — |
| `(500,1000)` | 0/12 | 否 | — |
| `(300,900)` | 0/12 | 否 | — |
| `(200,1500)` | 0/12 | 否 | — |

**失败与目标位置高度相关**：48/51 与「目标 `x` 越出地图」一一对应；剩下 3 个集中在一个 in-map 目标。

---

## 4. 按起点分桶

| 起点 | 失败/总 | 其中越界目标 | 其中 in-map |
|---|---|---|---|
| `(0,0)` | 15/30 | 12 | 3（`(900,300)`） |
| `(100,100)` | 12/30 | 12 | 0 |
| `(500,500)` | 12/30 | 12 | 0 |
| `(700,700)` | 12/30 | 12 | 0 |

**48 个 L1 只与目标有关，与起点无关**；`(900,300)` 那 3 个只出现在起点 `(0,0)`，与起点强相关。

补充的空间刻画（frozen 二进制、seed 0、起点 `(0,0)`、目标 x 200/400/600/800 × y 0/100/200/300/400/600/900/1200；`0` 成功、`2` 失败）：

```
 y\x    200    400    600    800
   0      0      2      2      2
 100      0      2      2      2
 200      0      2      2      2
 300      0      0      2      2
 400      0      0      2      2
 600      0      0      0      2
 900      0      0      0      0
1200      0      0      0      0
```

失败区是从起点朝 +x 展开的一个**楔形**：目标越远、越贴近 `y=0` 越容易失败。而 `Robot::new` 把两只脚放在 `(x, y±sep/2)`、`sep = 0.6*min_sep + 0.4*max_sep = 5.2`（冻结快照 `robot.rs:42-50`，即 ±2.6），起点 `(0,0)` ⇒ **左脚在 `(0,-2.6)`，在地图外（y<0）**。实测支持这一因果：

```
$ /tmp/nopath/trapla plan ... --start 0,0  --goal 900,300 → exit=2
$ /tmp/nopath/trapla plan ... --start 0,3  --goal 900,300 → exit=0   # 左脚 (0,0.4) 在图内
$ /tmp/nopath/trapla plan ... --start 0,6  --goal 900,300 → exit=0
$ /tmp/nopath/trapla plan ... --start 3,0  --goal 900,300 → exit=2   # 左脚 (3,-2.6) 仍在图外
$ /tmp/nopath/trapla plan ... --start 10,0 --goal 900,300 → exit=2
$ /tmp/nopath/trapla plan ... --start 0,10 --goal 900,300 → exit=0
```

---

## 5. 最小复现

### 5.1 `(0,0) → (900,300)`（L2，缺陷候选）

```
$ /tmp/nopath/trapla plan --map /home/vesita/coding/my/trapla/data/csv/map.csv \
      --start 0,0 --goal 900,300 --out /tmp/nopath/out.csv --seed 0
读取地图: /home/vesita/coding/my/trapla/data/csv/map.csv
地图已读入: 1000 行 x 3000 列
[1/1] 规划中 (seed 0) ...
错误: 规划失败：无路径 (seed 0)          ← exit=2
```

探针（`/tmp/nopath/probe 0 0 900 300 0`）尾部：

```
scale_star: start_idx=Idx { x: 0, y: 0 } goal_idx=Idx { x: 900, y: 300 } scale=0.025 -> Some(path_len=32, expanded=533)
step 13: pos=(63.383,4.000) swing=(0.000,-2.600) rz=0.0000 guides_left=31 candidates=21 tick=178
step 14: pos=(71.110,1.929) swing=(142.221,6.459) rz=-0.2618 guides_left=31 candidates=31 tick=177
step 15: pos=(71.110,1.929) swing=(0.000,-2.600) rz=0.0000 guides_left=31 candidates=5 tick=176
  [cross] swing=(0.000,-2.600) p1=(24.870,24.270) candidates_now=5
  [cross] p0=(142.221,6.459) candidates_now=0
结论: L2（avoid_feet_cross 第 2 步 walk_to Err(NoPath { from: Dot(DVec2(142.2208461224263, 6.45889527835962)),
      to: Dot(DVec2(142.2208461224263, 6.45889527835962)) }) ⇒ find_path:454）
[交叉验证] Robot::find_path -> Err(NoPath { from: Dot(DVec2(0.0, 0.0)), to: Dot(DVec2(900.0, 300.0)) })
```

三个 seed 的状态轨迹逐行相同（`probe_all.txt`），失败点一致。

卡在哪一步：高层 A* 没问题；卡在 **`walk_to`→`find_target` 候选集为空**，且是在**避让双脚交叉**（`will_feet_cross` 为真）时调用的第 2 次 `walk_to`——此时摆动脚刚被 `walk_update` 挪到 `p0=(142.221,6.459)`，`walk_candidates()` 归零。`avoid_feet_cross` 用 `?` 把它升级成整条 `find_path` 的 `NoPath`，没有回退。

### 5.2 `(0,0) → (1000,500)`（L1，确实不可达）

```
$ /tmp/nopath/trapla plan ... --start 0,0 --goal 1000,500 --out /tmp/nopath/out.csv --seed 0
错误: 规划失败：无路径 (seed 0)          ← exit=2
$ /tmp/nopath/probe 0 0 1000 500 0
scale_star: start_idx=Idx { x: 0, y: 0 } goal_idx=Idx { x: 1000, y: 500 } scale=0.025 -> None
结论: L1（scale_star 返回 None ⇒ find_path:421 NoPath）
```

`goal_idx.x = 1000 = 行数`，`ceil(1000*0.025) = 25 = sr` ⇒ `sg` 不在 `[0,25)`，`scale_star` 在入场检查（本仓 `astar.rs::scale_star` 的 `!in_scaled(sg)`）直接返回 `None`。另 47 个同类组合探针结论完全相同。

---

## 6. 结论（51 个）

| 分类 | 个数 | 占比 | 依据 |
|---|---|---|---|
| **实现缺陷候选** | **3** | **5.9%** | `(0,0)→(900,300)` × seed 0/1/2；独立判定高层可达（且 69 个成功组合的模型一致率 120/120），但规划器 `find_target` 空候选经 `avoid_feet_cross` 冒泡成 `NoPath` |
| **给定约束下确实不可达** | **48** | **94.1%** | 目标 `x ∈ {1000,1200,1500}`，越出 1000 行的地图；任何约束下都不可达 |
| **无法判定** | **0** | 0% | — |

两点补充：

1. 那 3 个「缺陷候选」的**最终责任层在 `robot.rs`（T4/T6c 范畴）**，不在 `astar.rs`/`terrain.rs`：`scale_star` 在该组合返回 `Some(32 点)`。触发条件是起点 `(0,0)` 让左脚初始落在图外 `(0,-2.6)`，以及 `avoid_feet_cross` 命中空候选时没有恢复路径。
2. 48 个虽然「确实不可达」，但它们是**参数越界**（`x ≥ 行数`），CLI 现在报 `exit=2`（规划失败）而不是 `exit=1`（参数错）。归到「规划失败」不算错，但作为产品语义值得单独裁决——本报告只登记，不修。

---

## 7. 额外发现（不计入 51，但同源）

### 7.1 地图远边的 in-map 死区（另一起缺陷候选）

`scale_star` 用 `index_scale(v, scale) = ceil(v*scale)` 把坐标映射到 `[0, sr)`；`sr = ceil(rows*scale)`。于是**地图内**、但靠近远边的点会被 `ceil` 推出缩放网格：

```
$ /tmp/nopath/trapla plan ... --start 100,100 --goal <x>,300 --seed 0
goal 960,300 → exit=0
goal 961,300 → exit=2      ← x=961 仍在地图内（rows=1000）
goal 980,300 → exit=2
goal 999,300 → exit=2

$ /tmp/nopath/trapla plan ... --start 100,100 --goal 500,<y> --seed 0
goal 500,2960 → exit=0
goal 500,2961 → exit=2     ← y=2961 仍在地图内（cols=3000）
goal 500,2999 → exit=2
```

起点同理（`--start 961,300 --goal 100,100 → exit=2`）。原因：`ceil(960*0.025)=24 < 25`，`ceil(961*0.025)=25 ≥ 25`。也就是 `x ∈ [961, 999]`、`y ∈ [2961, 2999]` 这一条 ≈ 一个 `max_stride` 宽的边带在高层图里**没有任何节点**。C++ 原实现在这里会用 `came_from[sg.x*sc+sg.y]` 越界（UB），Rust 的 `None` 是对 UB 的确定化，但「地图内目标被判 NoPath」这件事本身仍是缺陷候选。**不在 51 内**（51 的目标都在 x≥1000 的图外），属于扫描口径之外、值得另开任务。

### 7.2 `steep_extend` 门在 `scale_star` 里近乎空转

`scale_star` 用 `restore_block(next, scale=0.025)` 把缩放区块还原到原图。实测该映射在全部 1875 个缩放节点上的取值域是：

```
near_x ∈ {0,1}, far_x = {1}, near_y ∈ {0,1,2}, far_y ∈ {1,2}
1875 个缩放节点中 steep_extend 判死 0 个
```

即：**每个缩放节点看到的地形窗口都是地图左上角那一小块（约 2×3 格），与节点自身位置无关**（`block.scale(scale)` 是乘 `scale` 而不是除）。所以在本图上这层「陡峭度门」既不拦人、也不反映沿途地形。这与原 C++ 语义一致（`SqPlane::restore` 同样乘 `scale`），是移植保真，不是本轮引入；但它意味着 `scale_star` 的高层**没有**地形陡峭度约束。同样登记，不在 51 内。

### 7.3 `bfs` 兜底 / tick 耗尽不产生 `NoPath`

读码（`robot.rs:294-351`、`428-478`）+ 51 个探针轨迹：这两个缺陷只会让机器人**停滞或以部分路径成功返回（`exit=0`）**，不会变成 `exit=2`。task-21 列的这两个候选层在本次 51 个里各有 **0** 例。若将来要把「走不动了却报成功」也纳入诊断，需要另立判据（如路径长度/`goal_error` 阈值）。

---

## 8. 未判定项与缺什么

1. **3 个缺陷候选在完整足步约束下是否物理可达**：未判定。独立口径只覆盖高层缩放图。要判定需要一套独立的足步状态空间搜索（状态 = 双脚位置+朝向+which foot，动作 = `walk_candidates` 的合法集），或用一个不受 `avoid_feet_cross` 空候选阻塞的规划器变体做对照。
2. **48 个越界目标是否应改为 `exit=1`**：产品语义问题，未判定（本任务只读）。
3. **§7.1 的边带死区与 §7.2 的陡峭门空转**是否算缺陷：需要设计裁决；本报告只给实测现象与出处。
4. 探针复刻的是冻结快照的 `find_path` 循环；若 `robot.rs` 后续被 T6c 改动，层定位需在同一冻结二进制/快照上重跑（本报告所有数字都锚在 sha256 `41ddc8...`）。

---

## 9. 置信度

- **实测**（命令 + 原始输出）：
  - 冻结二进制 sha256；120/120 组合重跑与 `rows.txt post` 一致；
  - 48 L1 / 3 L2 / 0 其它（探针，全部 51 个）；
  - 独立缩放 BFS 与 `scale_star` 在 120/120 上一致；
  - 起点 `(0,0)` 楔形失败区、`--start 0,3` 复测成功；
  - x=960/961、y=2960/2961 的边界翻转；
  - 1875/1875 缩放节点 `steep_extend` 不判死、`restore_block` 取值域。
- **读码推断**（给 `文件:行`）：层出口清单（`robot.rs:414-424`、`447`、`454`、`268-275`、`294-351`、`428-478`）；`bfs` 与 tick 不产生 `NoPath`（由读码 + 探针轨迹共同支持）。
- **未判定**：§8 的 1–3。
