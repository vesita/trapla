# 约束式双足落足点规划：算法选型调研报告

> 范围：为 trapla（Rust，双足在线落足点规划，灰度高度图 → 满足物理约束的落足点序列）做算法选型。
> 本文只调研与选型，不含任何代码改动。调研日期 2026-10-19。
> 结论先行：**把「贪心逐足延伸」换成「在 (x, y, θ, foot) 状态图上的有界次优 anytime 搜索」**，
> 是唯一能同时满足「三类硬约束 + 确定性复现 + 在线重规划」三件事的路线；MIQP 与采样类都不该是第一步。

---

## 0. 结论与推荐（TL;DR）

### 0.1 推荐选型表

| 方案 | 核心机制 | 对本项目硬约束的处理 | 落地成本 | 主要风险 |
|---|---|---|---|---|
| **A（推荐）状态增广的 anytime 足步 A\*** | 状态 `(x, y, θ, foot)`；动作 = 现有极坐标候选的离散化档位；A\* 搜索，先 ε>1 出可行解再降 ε（ARA\* 语义），地图变化时增量修复（AD\* 语义） | 单步约束（步长/转角/法线）在**动作生成时过滤**；跨步耦合约束（双足间距 2–10cm、同足转角）在**动作过滤 + 状态里带对侧脚实际位置**；路径级指标（proper_crossings、步数预算）靠**代价权重 + 换边硬禁止**，不靠后处理 | 中：新增一个足步搜索模块（约 400–700 行 + 单测），不动 terrain/geom/foot/metrics；引导层 A\* 保留 | 状态数随 θ 离散与地图尺寸增长；θ 分档会改变步数；必须固定 tie-break 才能保住逐字节复现 |
| **B（先导增量）双足状态记忆 + 硬过滤** | 不换搜索框架，在现有 `Robot::find_path` 贪心里加「支撑脚/对侧脚/上一步足别」三件状态，候选先过硬过滤再按加权指向选 | 把 satisfy_spacing 从「对支撑脚」改成「对对侧脚实际位置」；同足转角与换边加硬约束 | 小：改动集中在 `robot.rs:471` 一带，1–2 天可跑出 6 用例对照 | 仍是贪心，proper_crossings 只能「大概率降 0」，不保证；它的价值是**证伪归因**（见 §6） |
| **C（不作为主搜索）MIQP 足步** | 环境分解成凸无碰区域，二进制变量分配落足区域，连续量解落足位姿，分支定界求全局最优（Deits & Tedrake 2014） | 约束最干净：可达域用**凸内近似**，转角用 sin/cos 的**分段线性内近似**（保证不低估可行性），障碍用凸区域 + big-M | 高：需要 MIP 求解器（Rust 生态无成熟商用级）、凸区域分解、且原文实测长序列很慢 | 10–30 步要「数十秒到分钟」；求解器分支定界路径不保证逐字节复现；适合做**离线精修层**而非在线主搜索 |

**不建议作主搜索**：采样类（RRT\*/BIT\*）。概率完备但非确定性，且本项目的可行落足域极窄
（双足间距只有 2–10cm、步长 <40cm），采样接受率会很低；可作为 MIQP/轨迹优化的初值来源。

### 0.2 第一个可落地增量（见 §6 的完整判据）

在 `Robot::find_path`（`crates/trapla-core/src/robot.rs:471`）内做**方案 B**：
给候选加三道硬过滤（步长 / 转角 / **对侧脚**间距）+ 同足转角与换边两项约束，
容器一律用 `BTreeMap`/显式排序键。判据：6 用例 `proper_crossings == 0`、
`lateral_separation.min ≥ 3.0`、`same_foot_turn` 的 |Δψ| 中位 < 8°、`steps ≤ baseline × 1.10`、
同 seed 两次运行逐字节一致。若这一步后 crossings 仍 >0，则**归因坐实为候选集本身允许交叉**，
下一步才上方案 A 换搜索（而不是继续调启发式）。

---

## 1. 调研方法与证据分级

### 1.1 环境实测

- 本机 `web_fetch` 打不开公网（`arxiv.org` 报 `resolves to a non-public IP address`），
  但 **`bash curl` 可直连**。因此本文采用三通道核对：
  1. **Crossref REST API**（`api.crossref.org/works/<DOI>`）：标题 / 作者 / 年份 / 出处 / 页码；
  2. **doi.org 内容协商**（`Accept: application/vnd.citationstyles.csl+json`）：DOI 解析出的登记记录；
  3. **arXiv API**（`export.arxiv.org/api/query`）：id / 标题 / 作者 / 完整摘要；
     以及作者主页 PDF 下载 + `pdftotext` 抽正文。
- 检索层只用于**发现候选**，不用于确认事实。凡是检索摘要里出现的 DOI / 作者，本文都回源核对过。

### 1.2 证据等级（全文统一）

| 等级 | 含义 | 判据 |
|---|---|---|
| **A 实读** | 下载到 PDF 或拿到 arXiv 完整摘要，读过摘要/正文原句 | 本文会写出读到的机制原句要点 |
| **B 元数据核对** | Crossref 记录 + doi.org 解析两条通道的标题/作者/年份一致；**正文未读** | 机制描述来自二手，已标注 |
| **C 二手转述** | 只有检索片段，未回源 | 本文尽量不用；用到的都显式标注 |

### 1.3 纠错记录（这是逐条回源的直接产出）

检索综合结果里有 4 处错误，全部按回源结果改正：

| 检索摘要的说法 | 回源结果 |
|---|---|
| Kuffner 2001 的 DOI 是 `10.1109/IROS.2001.973404` | 该 DOI 解析到 *Reduced inertial effect in damping-based posture control of mobile manipulator*；正确 DOI 是 **`10.1109/IROS.2001.973406`** |
| Deits & Tedrake 2014 的 DOI 是 `...7041375` | 该 DOI 是 *Whole-body motion planning with centroidal dynamics and full kinematics*；正确是 **`...7041373`** |
| Perrin 2011「Weakly collision-free paths」作者含 Young J. Kim | Crossref 记录作者为 **Perrin / Stasse / Lamiraux / Yoshida**（无 Kim） |
| 开源仓 `leggedrobotics/convex_plane_decomposition` 存在 | `git ls-remote` 失败（该路径不存在），本文不引用 |

另外：Hornung 2012 **没有 arXiv 预印本**（检索结果自己也说明了这一点），只能用 DOI。

### 1.4 兜底抽查（换措辞独立复核）

按要求挑「最新、最小众」的两条做换措辞抽查：

- **TAMOLS**（arXiv 2206.14049）：用「Jenelten Grandia Farshidian Hutter TAMOLS 摘要」换措辞检索，
  与 arXiv API 直接按 id 取回的摘要逐句一致 → 通过。
- **Griffin 2019**（arXiv 1907.08673）：用「Griffin Wiedebach McCrory planr region A\* 摘要」换措辞检索，
  与 arXiv API 直接按 id 取回的摘要一致 → 通过。
- 两条的 DOI/出处再用 Crossref 独立核对（`10.1109/tro.2022.3186804`、`10.1109/humanoids43949.2019.9035046`）也一致。

---

## 2. 把 trapla 的问题映射到文献坐标系

### 2.1 现状（一句话）

分层：1/40 降采样图上 A\*（曼哈顿启发 + 陡峭度代价）得引导折线 → `Robot::find_path`
沿引导按极坐标候选（半径 `active..max_stride`、角度相对支撑脚朝向 ±75° 步进 15°）
逐步贪心选一个（加权指向 `direction_determine`）。候选为空 → `NoPath`。

### 2.2 三类硬约束，处理手段完全不同

| 约束 | 数值 | 数学形态 | 现状由谁保证 | 合适的手段 |
|---|---|---|---|---|
| 步长 | < 40cm | 单步、只依赖支撑脚 | `Robot::satisfy_stride`（`robot.rs:191`） | **动作过滤**（生成候选时就拒绝） |
| 转向角 | 归一化到 (-π, π] 后 < ±75° | 单步、相对支撑脚朝向 | `Robot::satisfy_turn`（`robot.rs:208`） | 动作过滤 |
| 足平面法线 vs 重力 | < ±20° | 单步、只依赖落点邻域地形 | `Robot::position_check`（`robot.rs:242`） | 动作过滤（落点可行性） |
| **双足间距** | **2–10cm** | **跨步耦合：依赖「另一只脚的实际位置」，不是支撑脚朝向** | `Robot::satisfy_spacing`（`robot.rs:219`） | 动作过滤，但**必须把对侧脚位置放进状态** |
| 同足相邻步转角 | 指标口径 | 跨步耦合（隔一步） | 无 | 动作过滤或代价项 |
| `proper_crossings == 0` | 目标 0 | **路径级**（整条折线） | 无 | 搜索剪枝 / 代价项（后处理不可靠，见下） |
| 步数预算 | ≤ baseline × 1.10 | 全局 | 无 | 代价权重（步数项） |

**关键判断**：双足间距是「跨步耦合」约束 —— 它约束的是「第 k 步与第 k-1 步中**另一只脚**的关系」，
而现状的贪心每步只对「支撑脚」做局部最优。这就是猫步的机制来源：
没有 foot 状态，就没有任何东西阻止左右脚反复换边；`proper_crossings=4`、
`lateral_separation.min=0.82cm` 是这一缺失的直接表现，而不是启发式不够好。

**另一个必须记住的实测教训**：把双足轨迹压成共线可以让 `proper_crossings` 变 0，
但同时 `lateral_separation` 也变 0（`doc/catwalk_diagnosis.md` 的合成对照已证实）。
所以「用罚项把指标压下去」是假通过，路径级硬约束必须由**状态空间 + 硬过滤**保证，
罚项只能用于 tie-break。

### 2.3 文献坐标系的四个要素

任何足步规划器都可以用四个要素定位（本文所有条目都按这四栏写）：

1. **状态**：`(x, y, θ, foot)` 是否齐全？是否带地形/法线？
2. **动作**：离散原语集合，还是连续采样，还是优化变量？
3. **硬约束**：动作过滤（生成即拒绝）/ 投影（投到可行流形）/ 内近似（凸包保证保守）/
   罚项（软）/ 可行性证书（解出来再验）。
4. **搜索/优化**：A\*、anytime、增量、采样、分支定界、NLP。

---

## 3. 候选算法条目

> 每条都写全七项：核心机制 / 状态动作空间 / 硬约束处理 / 完备性最优性复杂度 /
> 在线重规划与确定性 / 开源实现 / 来源（含证据等级）。
> 「对本项目的可判实验」写的是**能在 trapla 上跑出对错的判据**，不是泛泛的展望。

### 3.1 足步状态空间 A\*（Kuffner 2001/2003；Chestnutt 2005）

- **核心机制（一句话）**：把「连续的全身运动规划」降维成「离散落足点序列的图搜索」，
  状态 = 支撑脚位姿 + 足别，动作 = 有限个运动学可行的相对步，A\* 在隐式图上搜。
- **状态 / 动作空间**：状态 `(x, y, θ, foot)`；动作是**依赖状态的有限集合**
  （前/后/侧/转向步，每步交替换脚）。Chestnutt 原文（A 级实读摘要）：
  "Using a finite set of state-dependent actions, we use an A\* search to compute optimal
  sequences of footstep locations up to a time-limited planning horizon."
- **硬约束怎么处理**：**动作过滤**。原语集合在生成时就把运动学可达性、自碰、
  足底碰撞筛掉；落足点再对障碍图做一次可行性检查。Kuffner 2003 的标题就是
  *Online footstep planning*，把 A\* 放进在线闭环（感知 → 重规划）。
- **完备性 / 最优性 / 复杂度**：给定原语集合与离散分辨率是**分辨率完备**；
  A\* 在启发可采纳时最优；复杂度随状态数指数增长，实际靠启发剪枝。
- **在线重规划与确定性**：A\* 本身确定（固定原语、固定 tie-break 即逐字节可复现）；
  重规划 = 从当前状态重跑（Kuffner 2003 的做法）。缺 anytime，时限压力下只能截断。
- **开源实现**：ROS `humanoid_navigation` 的 `footstep_planner`（C++，SBPL 后端，
  [ahornung/humanoid_navigation](https://github.com/ahornung/humanoid_navigation)，实测存在）。
- **对本项目的可判实验**：把 `find_path` 的候选集合当成原语集合，状态补上 foot，
  用 A\* 替换贪心；判据 = 6 用例 `proper_crossings` 是否降 0、`steps` 是否 ≤ baseline×1.10。
- **来源**：
  - Kuffner, Nishiwaki, Kagami, Inaba, Inoue, *Footstep planning among obstacles for biped robots*,
    IROS 2001, pp. 500–505, [DOI](https://doi.org/10.1109/IROS.2001.973406) —— **B（元数据双通道核对；正文未读）**
  - Kuffner, Kagami, Nishiwaki, Inaba, Inoue, *Online footstep planning for humanoid robots*,
    ICRA 2003, pp. 932–937, [DOI](https://doi.org/10.1109/ROBOT.2003.1241712) —— **B**
  - Chestnutt, Lau, Cheung, Kuffner, Hodgins, Kanade, *Footstep Planning for the Honda ASIMO Humanoid*,
    ICRA 2005, pp. 629–634, [DOI](https://doi.org/10.1109/ROBOT.2005.1570188) —— **A（实读 PDF 摘要，原文 PDF 已下载）**

### 3.2 Anytime 有界次优搜索：ARA\* / R\* 与足步规划（Hornung 2012）

- **核心机制（一句话）**：先用一个放大 ε 的启发式快速拿到「保证不超过 ε 倍最优」的解，
  再逐步降 ε 复用已有搜索树（INCONS 列表）逼近最优 —— 走起来比等最优解更重要。
- **状态 / 动作空间**：与 3.1 同构：`(x, y, θ, foot)` + 离散步原语（前进/侧移/后退/转向）。
  另提供 **R\***（随机化 A\*）：随机采中间状态把全局问题切成若干小子问题，
  缓解启发式导致的局部极小（U 形障碍、迷宫）。
- **硬约束怎么处理**：**动作过滤**，与原语集合绑定（同 3.1）。
  另有一个工程要点：启发式可以**不可采纳**（2D Dijkstra 场，忽略朝向与运动学），
  换来更快收敛，代价是解不再最优 —— 原文实测 ARA\* + 2D Dijkstra 启发最快，
  但「potential inadmissibility results in non-optimal paths for some scenarios」。
- **完备性 / 最优性 / 复杂度**：任意时刻都有 ε-次优保证；ARA\* 每次迭代内
  **每个状态至多扩展一次**（A 级实读原文："guarantees to examine each state at most once
  during its first search"），单次迭代复杂度 O(N log N)；ε → 1 时收敛到最优。
- **在线重规划与确定性**：天然支持 anytime（时间预算内持续改进）；
  但**确定性取决于 ε 调度**：若用 wall-clock 决定何时降 ε，输出随机器负载变化，
  逐字节复现失效。必须改成「按扩展次数/迭代次数」的固定调度。
- **开源实现**：SBPL（[sbpl/sbpl](https://github.com/sbpl/sbpl)，C++，实测存在）实现 ARA\*/AD\*/R\*；
  `humanoid_navigation` 的 `footstep_planner` 直接建立在 SBPL 上。
- **对本项目的可判实验**：同一张图、同一 seed，分别用 ε=2.0 固定次数调度与 wall-clock 调度，
  比较两次运行的落足点序列是否逐字节一致（预期 wall-clock 版不一致 → 这条就是确定性红线）。
- **来源**：Hornung, Dornbush, Likhachev, Bennewitz, *Anytime Search-Based Footstep Planning
  with Suboptimality Bounds*, Humanoids 2012, pp. 674–679,
  [DOI](https://doi.org/10.1109/HUMANOIDS.2012.6651592) —— **A（实读 PDF 摘要/正文，原文 PDF 已下载）**
- 配套理论：Likhachev, Gordon, Thrun, *ARA\*: Anytime A\* with Provable Bounds on Sub-Optimality*,
  NIPS 2003 —— **A（实读 PDF）**，见 §3.8。

### 3.3 State lattice 与运动原语体系化（Pivtoraiko 2009）

- **核心机制（一句话）**：把「离散状态 + 精确到达邻居的可行运动」做成**格**，
  于是任意格点上的动作集合都相同，重活（原语生成、碰撞扫掠体）可以离线预计算。
- **状态 / 动作空间**：状态是规则采样的位姿（含曲率等高阶量）；边是求解两点边值问题得到、
  **精确终止于邻居格点**的运动原语（A 级实读 Crossref 摘要："We compute a set of elementary
  motions that connects each discrete state value to a set of its reachable neighbors via
  feasible motions. Thus, this set of motions induces a connected search graph."）。
- **硬约束怎么处理**：**构造即可行**（feasibility by construction）——
  原语在生成阶段就满足微分/运动学约束；碰撞用预计算的扫掠体查表。
  这是「硬约束处理」里最干净的一类：约束进入动作集合，而不是进入代价。
- **完备性 / 最优性 / 复杂度**：分辨率完备；A\*/ARA\*/D\* Lite 都可直接在格上跑；
  在线只需普通图搜索（毫秒级），因为计算被挪到离线。
- **在线重规划与确定性**：完全确定（规则格 + 固定原语 + 固定 tie-break，无采样）。
  重规划直接换 ARA\*/AD\* 当后端。
- **开源实现**：SBPL 的 lattice 规划器；Autoware / Apollo 的 lattice 规划同源思路
  （后者为二手，未在本文核对具体模块）。
- **对本项目的可判实验**：用 `step < 40cm` 与 `±75°` 生成一个 5×7×N 的原语表，
  统计「这张高度图上被原语覆盖的落足格比例」；若比例太低（细窄通道进不去），
  说明纯格离散不够，需要 3.4 的连续化。
- **来源**：Pivtoraiko, Knepper, Kelly, *Differentially constrained mobile robot motion planning
  in state lattices*, Journal of Field Robotics 26(3):308–333, 2009,
  [DOI](https://doi.org/10.1002/rob.20285) —— **A（实读 Crossref 摘要，正文未读）**

### 3.4 Hybrid A\*（Dolgov 2008/2010）

- **核心机制（一句话）**：**连续状态挂到离散桶上**做 A\*（同桶只留最优），
  再用解析扩展（Reeds-Shepp 曲线）直连目标 + 数值平滑做第二阶段。
- **状态 / 动作空间**：状态是连续 `(x, y, θ)`（车辆模型），但按离散格桶做去重剪枝；
  动作 = 有界转向 + 固定弧长的前/后向运动原语。启发 = `max(不考虑障碍的非完整启发,
  不考虑运动学的有障碍 2D 启发)`。
- **硬约束怎么处理**：**动作级过滤 + 桶剪枝**（运动学可行由前向积分保证）；
  第二阶段非线性优化负责平滑与避障余量 —— 注意这会让「硬约束」在第二阶段变软。
- **完备性 / 最优性 / 复杂度**：对给定离散化是分辨率完备；**非最优**（桶剪枝会丢解）；
  A 级实读原文：全程 50–300ms 的重规划周期（DARPA Urban Challenge）。
- **在线重规划与确定性**：支持实时重规划（原文「typical full-cycle replanning times of
  50–300ms」）；但第二阶段的共轭梯度平滑是浮点迭代，跨平台逐字节复现风险高。
- **开源实现**：ROS 2 Nav2 的 Smac Planner（C++）、Apollo；本项目只值得借「连续状态 + 桶剪枝 +
  双启发取 max」这三招，不值得整套引入。
- **对本项目的可判实验**：把 θ 连续化、位置按 1cm 桶剪枝，测「同一条引导折线下
  proper_crossings 与 steps 的变化」；若剪枝后 crossings 明显下降，说明离散化粒度是主因。
- **来源**：Dolgov, Thrun, Montemerlo, Diebel, *Practical Search Techniques in Path Planning for
  Autonomous Driving*, 2008（STAIR 技术报告，[PDF](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)）
  —— **A（实读 PDF 摘要）**；期刊版 *Path Planning for Autonomous Vehicles in Unknown
  Semi-structured Environments*, IJRR 30(4):485–501, 2010,
  [DOI](https://doi.org/10.1177/0278364909359210) —— **B（Crossref 摘要）**

### 3.5 采样类：RRT\* / BIT\* 与连续足步 RRT

- **核心机制（一句话）**：用随机采样 + 局部连接在连续空间里生长树/图，
  RRT\* 通过「选最优父 + 重连」把概率完备升级成**渐近最优**；BIT\* 用一批批采样
  构造隐式随机几何图，再用 A\* 式启发排序只对「有希望」的边做碰撞检查。
- **状态 / 动作空间**：状态是连续位姿（足步版本是相对步 `Δx, Δy, Δθ` 的连续空间）；
  动作 = 采样 + steer / 局部连接。足步版本（Perrin 等）用扫掠体近似做碰撞检查，
  并在连续半步空间里扩展 —— 这是「离散原语」之外的另一条路。
- **硬约束怎么处理**：**边走边验**（可行性证书）：每条候选边都做运动学可达 + 碰撞 + 平衡校验；
  不满足的边直接丢弃。约束不进入状态空间，所以不需要离散化，但**没有可行性保证直到找到解**。
- **完备性 / 最优性 / 复杂度**：概率完备（RRT\*、BIT\* 渐近最优）；RRT 本身渐近次优
  （A 级实读 arXiv 摘要：现有算法的解「converges almost surely to a non-optimal value」）。
  窄可行域（本项目双足间距 2–10cm）下接受率会很低。
- **在线重规划与确定性**：可 anytime；**确定性最弱的一族** —— 即使固定 seed，
  跨平台浮点与近邻搜索实现差异都可能改变结果，与 trapla 的逐字节复现契约冲突。
- **开源实现**：OMPL（[ompl/ompl](https://github.com/ompl/ompl)，C++，含 RRT\*/BIT\*，实测存在）；
  足步连续版本无成熟通用库（Perrin 的实现在 LAAS 的 HPP 生态里，未逐仓核实）。
- **对本项目的可判实验**：固定 seed 跑两次同一场景，比较落足点序列是否逐字节一致
  （预期不一致或极难保证）；再统计「随机采样落在 satisfy_spacing 可行域内的比例」，
  若 < 5% 就直接判死这条路作为主搜索。
- **来源**：
  - Karaman, Frazzoli, *Sampling-based Algorithms for Optimal Motion Planning*,
    IJRR 30(7):846–894, 2011, [DOI](https://doi.org/10.1177/0278364911406761)，
    [arXiv:1105.1186](https://arxiv.org/abs/1105.1186) —— **A（实读 arXiv 完整摘要）**
  - Gammell, Srinivasa, Barfoot, *Batch Informed Trees (BIT\*)*, ICRA 2015,
    [arXiv:1405.5848](https://arxiv.org/abs/1405.5848) —— **A（实读 arXiv 完整摘要）**
  - Perrin, Stasse, Baudouin, Lamiraux, Yoshida, *Fast Humanoid Robot Collision-Free Footstep
    Planning Using Swept Volume Approximations*, T-RO 28(2):427–439, 2012,
    [DOI](https://doi.org/10.1109/TRO.2011.2172152) —— **B（仅元数据；机制描述为二手）**
  - Perrin, Stasse, Lamiraux, Yoshida, *Weakly collision-free paths for continuous humanoid
    footstep planning*, IROS 2011, pp. 4408–4413,
    [DOI](https://doi.org/10.1109/IROS.2011.6094511) —— **B（仅元数据；注意检索摘要把作者写错，已纠正）**
  - Hauser, Bretl, Latombe, Harada, Wilcox, *Motion Planning for Legged Robots on Varied Terrain*,
    IJRR 27(11–12):1325–1349, 2008, [DOI](https://doi.org/10.1177/0278364908098447) —— **B**
  - Xia, Xiong, Chen, *A deterministic sampling-based approach to global footstep planning for
    humanoid robots*, Humanoids 2009, pp. 142–147,
    [DOI](https://doi.org/10.1109/ICHR.2009.5379512) —— **B（「确定性采样」这个组合值得单独读，正文未读）**

### 3.6 优化类：MIQP 足步 与 轨迹优化

- **核心机制（一句话）**：MIQP 把「踩哪块区域」做成二进制变量、「踩在哪」做成连续变量，
  在凸分解后的环境里一次解到全局最优；轨迹优化则把接触时序/落足点作为连续决策变量，
  用 NLP 联合优化重心轨迹与落足点。
- **状态 / 动作空间**：没有搜索状态 —— 决策变量是整段 horizon 上每只脚的位姿（MIQP）
  或基座位姿 + 落足点 + 相位时长（TOWR/TAMOLS）。
- **硬约束怎么处理**：**内近似 + 硬约束进规划问题**。Deits & Tedrake 原文（A 级实读摘要）：
  可达性用「convex inner approximation of the reachable space」；转角用
  「piecewise linear approximation of sine and cosine, designed to ensure that the
  approximation **never overestimates** the robot's reachability」；障碍用凸区域分解 + big-M。
  这是本报告里**唯一能给出「不可行即证明不可行」证书**的路线。
- **完备性 / 最优性 / 复杂度**：MIQP 在给定凸分解下全局最优（或证明不可行）；
  但整数变量数随步数与区域数增长，是 NP-hard 家族的实践版 —— 原文实测：
  「短序列（几步）不到 1 秒；10–30 步要数十秒到分钟」（A 级实读原句）。
  轨迹优化（TAMOLS）走直接配点，原文称在线可「less than ten milliseconds」（A 级 arXiv 摘要）。
- **在线重规划与确定性**：MIQP 热启动困难，长序列不适合在线；分支定界路径依赖求解器内部
  启发与浮点，**不保证逐字节复现**（这是与 trapla 契约的硬冲突）。
  轨迹优化的固定迭代次数版本可以复现，但依赖 IPOPT 版本与浮点环境。
- **开源实现**：Drake（[RobotLocomotion/drake](https://github.com/RobotLocomotion/drake)，C++，
  含 MixedIntegerConvexOptimization 足步示例，实测存在）；TOWR
  （[ethz-adrl/towr](https://github.com/ethz-adrl/towr)，C++，实测存在）；
  OCS2（[leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)，C++，实测存在）。
- **对本项目的可判实验**：用单张 1000×3000 图取 30 步窗口，估一次 MIQP 的整数变量规模
  （步数 × 凸区域数），若 > 1e4 就基本可以判死在线使用；同时记录求解器版本，
  验证两次运行结果是否逐字节一致（预期否）。
- **来源**：
  - Deits, Tedrake, *Footstep Planning on Uneven Terrain with Mixed-Integer Convex Optimization*,
    Humanoids 2014, pp. 279–286, [DOI](https://doi.org/10.1109/HUMANOIDS.2014.7041373)
    —— **A（实读 PDF 摘要与性能原句）**
  - Winkler, Bellicoso, Hutter, Buchli, *Gait and Trajectory Optimization for Legged Systems
    Through Phase-Based End-Effector Parameterization*, RA-L 3(3):1560–1567, 2018,
    [DOI](https://doi.org/10.1109/LRA.2018.2798285) —— **B（元数据核对；机制为二手）**
  - Jenelten, Grandia, Farshidian, Hutter, *TAMOLS: Terrain-Aware Motion Optimization for
    Legged Systems*, T-RO 38(5):3395–3413, 2022, [DOI](https://doi.org/10.1109/TRO.2022.3186804)，
    [arXiv:2206.14049](https://arxiv.org/abs/2206.14049) —— **A（实读 arXiv 完整摘要）**
  - Tonneau, Del Prete, Pettre, Park, Manocha, Mansard, *An Efficient Acyclic Contact Planner for
    Multiped Robots*, T-RO 34(3):586–601, 2018,
    [DOI](https://doi.org/10.1109/TRO.2018.2819658) —— **B（仅元数据）**

### 3.7 腿式/四足 foothold planning（可通行性地图、Raibert 启发式、ANYmal）

- **核心机制（一句话）**：先用高程图算出**可通行性/落脚代价层**，再在名义落足点邻域里
  做一个小的连续优化或局部搜索，把「踩边缘、踩斜面、够不着」的落足点剔掉。
- **状态 / 动作空间**：四足通常不做全局足步图搜索 —— 步态给时序，落足点由
  名义启发式（Raibert 型：`p_hip + 半个支撑期 × 速度 + 速度误差修正`）给初值，
  再在邻域内优化。这是「局部连续、全局由步态/导航层负责」的架构。
- **硬约束怎么处理**：**代价 + 过滤混合**。斜面法线、粗糙度、边缘距离进代价；
  运动学可达与腿-地形碰撞做硬过滤。Raibert 启发式本身不是约束机制，
  它是「动力学上合理的初值」，约束靠后面的优化/过滤兜。
- **完备性 / 最优性 / 复杂度**：无完备性保证（局部优化）；计算量小
  （TAMOLS 原文：在线 < 10ms），适合高频重规划。
- **在线重规划与确定性**：在线性强；但卡尔曼式高程图与局部 NLP 都不是逐字节可复现的，
  适合「实时」不适合「可复现基准」。
- **开源实现**：[ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping)（C++）、
  [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map)（多图层 2.5D 栅格）、
  [leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy)（Python/CuPy）、
  [leggedrobotics/traversability_estimation](https://github.com/leggedrobotics/traversability_estimation)（C++）
  —— 四个仓均实测存在。
- **对本项目的可判实验**：把 `terrain.normal_angle` 的 20° 门槛从「动作过滤」改成
  「代价项」，测 `proper_crossings` 与 `steps` 是否被软化（预期 crossings 反弹）——
  用来证明**硬约束不能降级为罚项**。
- **来源**：
  - Raibert, *Legged Robots That Balance*, MIT Press, 1986
    （[OpenLibrary 记录](https://openlibrary.org/search.json?title=Legged+Robots+That+Balance&author=Raibert)）
    —— **B（书名/作者/年份核对；内容为公认二手，正文未读）**
  - Fankhauser, Bloesch, Gehring, Hutter, Siegwart, *Robot-Centric Elevation Mapping with
    Uncertainty Estimates*, CLAWAR 2014（Mobile Service Robotics）, pp. 433–440,
    [DOI](https://doi.org/10.1142/9789814623353_0051) —— **B（元数据；机制为二手）**
  - Jenelten, Miki, Vijayan, Bjelonic, Hutter, *Perceptive Locomotion in Rough Terrain –
    Online Foothold Optimization*, RA-L 5(4):5370–5376, 2020,
    [DOI](https://doi.org/10.1109/LRA.2020.3007427) —— **B（元数据；机制为二手）**

### 3.8 在线重规划：D\* Lite / AD\* / anytime 足步

- **核心机制（一句话）**：不从头重算，而是**增量修复**上一次的搜索树：
  只把受影响的边代价变化传播出去（D\* Lite 用 LPA\* 的 g/rhs 两个值），
  再把 anytime（ε 调度）叠上去就是 AD\*。
- **状态 / 动作空间**：与宿主搜索同构（本项目即 3.1/3.2 的 `(x, y, θ, foot)` 图）；
  D\*/AD\* 是**搜索算法层**，不改变状态与动作定义。
- **硬约束怎么处理**：不变 —— 约束仍在动作过滤/原语层；增量修复只处理**边代价变化**
  （例如某格突然变成障碍、法线变化导致该步不再可行）。注意：**可行性变化必须体现为
  「边被删除/代价置无穷」**，而不是改罚项，否则修复逻辑会给出「看起来更新过」的旧解。
- **完备性 / 最优性 / 复杂度**：D\* Lite 与 Focussed D\* 行为相同（A 级实读原文），
  实现更简单；AD\* 每步都复用之前的搜索，任意时刻给出 ε-次优解。
  复杂度取决于变化的边数与启发质量，最坏退回一次全量搜索。
- **在线重规划与确定性**：这一族是「在线」的正解。确定性可控：只要
  「变更集合的顺序」与「优先队列 tie-break」都固定（用有序容器 + 显式比较键），
  同一输入序列可逐字节复现。**禁止**用 wall-clock 决定 ε 降档或修复轮次。
- **开源实现**：SBPL（C++，含 AD\*）；`humanoid_navigation` 的 `footstep_planner` 支持
  ARA\*/AD\*/R\* 三种后端（ROS wiki 与仓库描述）。
- **对本项目的可判实验**：固定一张图跑一次，然后人为把引导带上一格改成障碍，
  再跑一次增量修复；判据 = 修复后路径仍满足全部硬约束，且 `proper_crossings` 不变差；
  同时记录「修复扩展状态数 / 全量重跑扩展状态数」的比值（预期 < 0.3）。
- **来源**：
  - Koenig, Likhachev, *D\* Lite*, AAAI 2002,
    [PDF](https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) —— **A（实读 PDF 摘要）**
  - Likhachev, Ferguson, Gordon, Stentz, Thrun, *Anytime Dynamic A\*: An Anytime, Replanning
    Algorithm*, ICAPS 2005,
    [PDF](https://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf) —— **A（实读 PDF 摘要）**
  - Likhachev, Gordon, Thrun, *ARA\*: Anytime A\* with Provable Bounds on Sub-Optimality*,
    NIPS 2003，[PDF](https://www.cs.cmu.edu/~maxim/files/ara_nips03.pdf) —— **A（实读 PDF 摘要）**
  - 足步 + anytime 的结合见 §3.2（Hornung 2012）—— **A**

### 3.9 当代对照（说明方向没走偏，但不作为选型）

- **粗糙地形上的 A\* 足步（Griffin 2019）**：平面区域表示 + **允许部分落足（partial footholds）**
  + 在格离散**之间**做后处理找更好的解。核心机制一句话：格搜索保证可行，
  后处理在格点之间找更优位姿。这印证了「A\* + 连续精修」是工业界主流组合。
  来源：Humanoids 2019, pp. 9–16, [DOI](https://doi.org/10.1109/HUMANOIDS43949.2019.9035046)，
  [arXiv:1907.08673](https://arxiv.org/abs/1907.08673) —— **A（实读 arXiv 完整摘要）**。
- **感知 + MIQP 双足（Acosta & Posa 2025）**：把感知不确定性与混合整数足步控制结合的最新方向，
  说明 MIQP 路线仍在演进。来源：T-RO 2025, [DOI](https://doi.org/10.1109/TRO.2025.3587998)
  —— **B（仅元数据，正文未读）**。
- **多接触/无环接触规划（Tonneau 2018）**：接触序列 + 优化，适用于攀爬类非平坦地形，
  与 trapla 的「平面行走 + 局部高差」场景不完全对应。**B**。

### 3.10 横向对比表

| 算法族 | 状态空间 | 硬约束处理 | 完备性 / 最优性 | 在线重规划 | 逐字节可复现 | 开源实现 | 对本项目适配度 |
|---|---|---|---|---|---|---|---|
| 足步 A\*（3.1） | `(x,y,θ,foot)` + 原语 | 动作过滤 | 分辨率完备 / 最优 | 重跑 | 是（固定 tie-break） | SBPL、humanoid_navigation | **高** |
| ARA\*/R\*（3.2） | 同上 | 动作过滤 | ε-次优保证 / 收敛到最优 | **原生 anytime** | 是（ε 按计数调度） | SBPL | **最高** |
| State lattice（3.3） | 规则格 + 精确原语 | 构造即可行 | 分辨率完备 / 最优 | 换后端 | 是 | SBPL lattice | 高（离线原语表） |
| Hybrid A\*（3.4） | 连续状态 + 桶剪枝 | 过滤 + 第二阶段放松 | 分辨率完备 / 非最优 | 强（50–300ms） | 中（平滑层浮点） | Nav2 Smac | 中（借三招） |
| RRT\*/BIT\*（3.5） | 连续 | 边验证 | 概率完备 / 渐近最优 | 可 anytime | **弱** | OMPL | 低 |
| MIQP（3.6） | 决策变量 | 内近似 + 硬约束 | 全局最优 / 可证不可行 | 弱（热启动难） | **弱**（求解器路径） | Drake | 低（离线精修可） |
| 轨迹优化（3.6） | 决策变量 | 硬约束 + 配点 | 局部最优 | 强（<10ms，TAMOLS） | 弱 | TOWR、OCS2 | 低（不含连续控制层） |
| 四足 foothold（3.7） | 无全局状态 | 代价 + 过滤 | 无保证 | 强 | 弱 | elevation_mapping、grid_map | 中（借高程图思路） |
| D\* Lite / AD\*（3.8） | 宿主搜索图 | 同宿主 | 同宿主 + 增量修复 | **最强** | 是（有序容器） | SBPL | **高（第二阶段必上）** |

---

## 4. 选型建议

### 方案 A（推荐）：状态增广的 anytime 足步 A\*

**机制**
- 状态 `s = (x, y, θ, foot)`，位置按 1–2cm、朝向按 15°（或更细）离散；
  另在节点里冗余存「对侧脚实际位置」——因为双足间距约束依赖它，不能只靠朝向推。
- 动作 = 现有极坐标候选分档：半径取 `[active, max_stride)` 内若干档 × 角度 ±75° 步进 15°
  （沿用现有候选几何，现有 x/y 单位是 cm，无需换算）。
- 成本 `g`：步数项（权重最大，直接对应 `steps` 预算）+ 路径长度 + 换边惩罚（tie-break 级）。
- 启发 `h`：复用现有 1/40 A\* 的距离场（2D Dijkstra，Hornung 2012 的做法），
  允许不可采纳以换速度，但要在报告里注明「不保证最优」。
- 搜索：先 ε=2.0 出可行解 → 固定次数降档 → 地图更新时走 AD\* 增量修复。

**硬约束处理（逐条对号）**
| 约束 | 手段 |
|---|---|
| 步长 < 40cm | 动作生成即拒（复用 `satisfy_stride`） |
| 转角 < ±75° | 动作生成即拒（复用 `satisfy_turn`） |
| 法线 < ±20° | 落点可行性过滤（复用 `position_check`） |
| 双足间距 2–10cm | 动作生成时用**对侧脚实际位置**校验（`satisfy_spacing` 的本意），违规候选不生成 |
| 同足转角 | 状态里带上一步足别与朝向，动作过滤 |
| proper_crossings = 0 | 换边在动作层**硬禁止**（同侧连续两步的横向偏移方向锁定）+ 交叉计数的代价项 |
| steps ≤ +10% | 步数代价权重（主要旋钮），用 6 用例回归调 |

**落地成本**：中。新增 `crates/trapla-core/src/footstep_search.rs`（约 400–700 行含单测），
改动 `robot.rs:471` 的 `find_path` 接口（保留 `NoPath` 契约）。不改 terrain/geom/foot/metrics。

**主要风险**
1. θ 离散档位会直接改变步数 —— 需要把 θ 档位纳入「既有 golden 语义」，否则 steps 红线会被离散化吃掉。
2. 状态数：名义上 3e6 格 × 48 档 × 2 足 ≈ 2.9e8（**推断，非实测**），必须靠「引导带内剪枝 +
   强启发 + ε」压到可接受扩展数；实际扩展数要用 6 用例实测，不能拍脑袋。
3. tie-break 不固定 → 逐字节复现失效（rng.rs 已把确定性写成硬要求）。

### 方案 B（先导增量）：双足状态记忆 + 硬过滤（保持贪心）

**机制**：不换搜索框架。在 `find_path` 里维护「当前足别 / 对侧脚位置 / 上一步足别与朝向」，
候选先过 `satisfy_spacing`（对侧脚）、`satisfy_stride`、`satisfy_turn`，再按加权指向选。
容器用 `BTreeMap`/显式排序。

**取舍**：成本 1–2 天，几乎零架构风险；但它**不保证** crossings=0 —— 贪心在局部仍可能换边。
它的真正价值是**证伪归因**：如果加了足别状态后 crossings 仍 > 0，说明根因在
「候选集合本身允许左右交叉」，那就必须上方案 A（换搜索），而不是继续调指向权重。

**代价**：若先做 B 再上 A，B 的部分代码（候选过滤）可复用（过滤逻辑与搜索层解耦），
浪费可控。**风险**：容易把 B 的「大致改善」当成终点，指标回落时反复调参 —— 必须一开始就写死
「B 的目标是证伪，不是达标」。

### 方案 C（不作为主搜索）：MIQP 足步

**机制**：见 §3.6。**取舍**：约束最干净、能证不可行；但原文实测长序列要数十秒到分钟，
且求解器路径不可逐字节复现、Rust 侧无成熟求解器生态。**定位**：只在「离线精修」
（把方案 A 的离散解当 warm start，在局部 5–10 步窗口里用 MIQP 微调）有意义，
且必须接受「产物不可复现」的代价，或干脆不进仓库。

### 为什么采样类落选（一句话版）

采样类的问题不是「不够好」，而是**与本项目的两个硬契约正面冲突**：
逐字节复现（RRT\*/BIT\* 依赖随机序列与浮点近邻搜索）+ 极窄可行域
（双足间距 2–10cm，随机落点命中率极低）。作为 MIQP/轨迹优化初值可以，作主搜索不行。

---

## 5. 对既有 golden 夹具与指标口径的影响

### 5.1 会碰到的夹具（按「改哪一层」分）

| 改动层 | 受影响的夹具 | 影响方式 |
|---|---|---|
| 只改 `find_path` 的候选过滤/选择（方案 B） | `robot_golden.rs` 的 `test_metrics_full`、`test_reproducibility` | 输出路径变了 → 期望坐标/步数要更新；**逐点谓词夹具不动** |
| 改候选集合的档位（半径/角度分档，方案 A） | `robot_golden.rs` 的 `test_empty_candidates_returns_no_path`、`test_metrics_full` | 空候选 → `NoPath` 契约必须保留；paths 改变 |
| 改引导层 A\*（启发式/代价/降采样比） | `terrain_astar_golden.rs` 的 `a_star_*` 与 `rng_frozen_seed_42_sequence` / `rng_frozen_seed_42_shuffle` | 引导折线变 → 下游全变；若改变 `shuffle` 调用次数，**RNG 冻结序列夹具直接失败** |
| 引入 MIQP / NLP / 采样 | `test_reproducibility` | 大概率失败（浮点/求解器路径），需要单独豁免或在 CI 里标注 |
| 新增足步搜索模块（方案 A） | `geom_grid_golden.rs`、`terrain_astar_golden.rs` 的几何/RNG 夹具 | **不受影响**（只要不动 geom/terrain/rng） |

**关键结论**：单点谓词夹具（`test_golden_corner/cover/standable/slide/satisfy_spacing/
satisfy_stride/satisfy_turn` + 各自的边界守卫）测的是 `Foot`/`Robot` 的**逐点谓词**，
不是路径。只要不修改 `satisfy_*` 与 `position_check` 的语义，它们不受影响。
真正需要更新期望值的是**路径级**夹具：`test_metrics_full`、`test_reproducibility`，
以及（若改引导层）`terrain_astar_golden.rs` 的 A\* 与 RNG 夹具。

### 5.2 指标口径（`crates/trapla-core/src/metrics.rs`，冻结定义）

| 指标 | 字段 | 目标 | 新算法下怎么保证 |
|---|---|---|---|
| `proper_crossings` | `proper_crossings: usize` | == 0（主判据） | 状态里带 foot + 换边硬禁止 → 结构性保证；**不能**靠后处理压 |
| `lateral_separation` min | `lateral_separation: Vec<f64>` | ≥ 3.0（辅） | `satisfy_spacing` 对**对侧脚**校验（不是对支撑脚） |
| `same_foot_turn` | `same_foot_turn: Vec<f64>` | \|Δψ\| 中位 < 8° | 状态里带上一步足别与朝向，动作过滤 + 代价项 |
| `steps` | `steps: usize` | ≤ baseline × 1.10 | 步数代价权重；θ 离散档位要盯住 |
| `goal_error` | `goal_error: f64` | 不退化 | 终点判据沿用现有口径 |

> 注意：现有 `metrics.rs` 的口径是冻结的（注释写明与 C++ 基线同源）。新算法**不得**
> 为了过指标去改 metrics；改的是搜索，不是尺子。

### 5.3 确定性契约（`crates/trapla-core/src/rng.rs` 已写死的硬要求）

1. seed 只能来自显式传入，禁止系统时间/`random_device`；
2. 同 seed 跨平台逐字节同序列（SplitMix64 + xorshift64\*，纯整数）；
3. 涉及顺序处用 `BTreeMap` 或显式排序，禁止依赖 `HashMap` 迭代顺序；
4. 本报告追加一条：**anytime 的 ε 调度与重规划轮次必须由「扩展次数/迭代次数」决定，
   禁止由 wall-clock 决定** —— 否则同一 seed 两次运行因机器负载不同而分叉。

---

## 6. 落地路线与第一个可落地增量

### 增量 1（方案 B，1–2 天）：足别状态 + 硬过滤，先做归因

1. 在 `Robot::find_path`（`robot.rs:471`）里维护 `(which_foot, other_foot_pos, last_foot_dir)`；
2. 候选先过 `satisfy_stride` / `satisfy_turn` / `satisfy_spacing`（对侧脚）；
3. 增加「同足转角」与「换边」硬限制（换边 = 同侧连续两步横向偏移符号相反）；
4. 全程 `BTreeMap` / 显式排序键。
5. **判据（6 用例）**：
   - `proper_crossings == 0`（主）
   - `lateral_separation.min ≥ 3.0`（辅 1）
   - `same_foot_turn` 的 |Δψ| 中位 < 8°（辅 2）
   - `steps ≤ baseline × 1.10`（回归红线）
   - 同 seed 两次运行输出逐字节一致
6. **预测与证伪**：若 crossings 仍 > 0 → 归因坐实为「候选集允许交叉」，直接进增量 2；
   若 crossings = 0 但 steps 超红线 → 说明换边禁止太硬，需要在方案 A 里改成代价项。

### 增量 2（方案 A，1–2 周）：把贪心换成 anytime 图搜索

- 新增 `footstep_search.rs`，状态/动作/代价/启发按 §4 方案 A；
- 先在 ε 固定为 1（纯 A\*）下跑通，再开 ε 调度；
- 判据同上 + 记录「扩展状态数 / 规划耗时」随 ε 的变化曲线（用于定在线预算）。

### 增量 3（在线重规划，1 周）：AD\* 增量修复 + 触发条件

重规划触发条件（全部由状态判定，不看时钟，保证可复现）：

| 触发 | 判据 | 动作 |
|---|---|---|
| T1 地图变化 | 引导带内出现新障碍格，或法线越过 20° 门槛 | 删除/置无穷相关边 → AD\* 增量修复 |
| T2 执行偏差 | 实际落足与计划落足距离 > 半个格 | 以实测位姿为新起点重规划 |
| T3 无解 | 搜索返回 `NoPath` | 提高 ε 或回退到引导折线 + 局部重试 |
| T4 预算富余 | 已花扩展数 < 预算 | 继续降 ε 精修（有界次数） |

判据：T1 触发后「修复扩展数 / 全量重跑扩展数」< 0.3，且修复后路径仍满足全部硬约束与
主指标；同输入序列两次运行逐字节一致。

---

## 7. 未验证清单（必须与上面的结论分开读）

1. **未逐条点开所有论文的正文**。A 级条目读了 PDF/arXiv 摘要（会写出读到的原句要点）；
   B 级条目只做了元数据双通道核对，机制描述来自二手转述。
2. **Kuffner 2001 / 2003 正文未读**：作者主页 `kuffner.org` 的论文列表不含这两篇 PDF
   （该页面实测可访问但不提供 2001/2003 全文），CMU 的 `~Joel/footstep/` 页面返回 HTTP 500。
   条目内容按二手 + 与 Chestnutt 2005 的一致性推断。
3. **Perrin 2011/2012、Hauser 2008、Tonneau 2018、Acosta & Posa 2025、Xia 2009、
   TOWR、Jenelten 2020、Fankhauser 2014 正文未读**（仅元数据/检索片段）。
4. **复杂度与状态数估算全是推断**：2.9e8 名义状态、扩展数、在线耗时都没有在
   trapla 上实测；本文没有跑任何 cargo 命令或测试。
5. **开源实现的「可借鉴模块」是文档级判断**：只核了仓库存在性、语言、活跃度，
   没有逐仓读源码（SBPL 的 ARA\*/AD\* 实现、humanoid_navigation 的 footstep_planner
   接口细节均未读）。
6. **检索级参考**：`leggedrobotics/convex_plane_decomposition` 在检索摘要里出现过，
   但 `git ls-remote` 失败，本文不引用；`AIS-Bonn/humanoid_path_planner` 同样不存在，
   用 `ahornung/humanoid_navigation` 代替。
7. **本文不含任何对 trapla 代码的修改**：本次调研只新建了本文件。
   核对时刻（2026-10-19 14:22）的 `git status` 里还有 `M Cargo.toml` 与 `?? crates/trapla-lab/`，
   它们是**同期另一个任务（task-1 无头测试台）的写入**（mtime 14:18/14:19，早于本文件 14:22），
   不属于本次调研，本文也未触碰。

---

## 8. 参考文献（可点）

**搜索类（足步 A\* / anytime）**
1. Kuffner, Nishiwaki, Kagami, Inaba, Inoue. *Footstep planning among obstacles for biped robots.* IROS 2001:500–505. https://doi.org/10.1109/IROS.2001.973406
2. Kuffner, Kagami, Nishiwaki, Inaba, Inoue. *Online footstep planning for humanoid robots.* ICRA 2003:932–937. https://doi.org/10.1109/ROBOT.2003.1241712
3. Chestnutt, Lau, Cheung, Kuffner, Hodgins, Kanade. *Footstep Planning for the Honda ASIMO Humanoid.* ICRA 2005:629–634. https://doi.org/10.1109/ROBOT.2005.1570188
4. Hornung, Dornbush, Likhachev, Bennewitz. *Anytime Search-Based Footstep Planning with Suboptimality Bounds.* Humanoids 2012:674–679. https://doi.org/10.1109/HUMANOIDS.2012.6651592
5. Likhachev, Gordon, Thrun. *ARA\*: Anytime A\* with Provable Bounds on Sub-Optimality.* NIPS 2003. https://www.cs.cmu.edu/~maxim/files/ara_nips03.pdf
6. Likhachev, Ferguson, Gordon, Stentz, Thrun. *Anytime Dynamic A\*: An Anytime, Replanning Algorithm.* ICAPS 2005. https://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf
7. Koenig, Likhachev. *D\* Lite.* AAAI 2002. https://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
8. Pivtoraiko, Knepper, Kelly. *Differentially constrained mobile robot motion planning in state lattices.* JFR 26(3):308–333, 2009. https://doi.org/10.1002/rob.20285
9. Dolgov, Thrun, Montemerlo, Diebel. *Practical Search Techniques in Path Planning for Autonomous Driving.* 2008. https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf
10. Dolgov, Thrun, Montemerlo, Diebel. *Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments.* IJRR 30(4):485–501, 2010. https://doi.org/10.1177/0278364909359210
11. Griffin, Wiedebach, McCrory, Bertrand, Lee, Pratt. *Footstep Planning for Autonomous Walking Over Rough Terrain.* Humanoids 2019:9–16. https://doi.org/10.1109/HUMANOIDS43949.2019.9035046 · https://arxiv.org/abs/1907.08673

**采样类**
12. Karaman, Frazzoli. *Sampling-based Algorithms for Optimal Motion Planning.* IJRR 30(7):846–894, 2011. https://doi.org/10.1177/0278364911406761 · https://arxiv.org/abs/1105.1186
13. Gammell, Srinivasa, Barfoot. *Batch Informed Trees (BIT\*).* ICRA 2015. https://arxiv.org/abs/1405.5848
14. Perrin, Stasse, Baudouin, Lamiraux, Yoshida. *Fast Humanoid Robot Collision-Free Footstep Planning Using Swept Volume Approximations.* T-RO 28(2):427–439, 2012. https://doi.org/10.1109/TRO.2011.2172152
15. Perrin, Stasse, Lamiraux, Yoshida. *Weakly collision-free paths for continuous humanoid footstep planning.* IROS 2011:4408–4413. https://doi.org/10.1109/IROS.2011.6094511
16. Hauser, Bretl, Latombe, Harada, Wilcox. *Motion Planning for Legged Robots on Varied Terrain.* IJRR 27(11–12):1325–1349, 2008. https://doi.org/10.1177/0278364908098447
17. Xia, Xiong, Chen. *A deterministic sampling-based approach to global footstep planning for humanoid robots.* Humanoids 2009:142–147. https://doi.org/10.1109/ICHR.2009.5379512

**优化类**
18. Deits, Tedrake. *Footstep Planning on Uneven Terrain with Mixed-Integer Convex Optimization.* Humanoids 2014:279–286. https://doi.org/10.1109/HUMANOIDS.2014.7041373
19. Winkler, Bellicoso, Hutter, Buchli. *Gait and Trajectory Optimization for Legged Systems Through Phase-Based End-Effector Parameterization.* RA-L 3(3):1560–1567, 2018. https://doi.org/10.1109/LRA.2018.2798285
20. Jenelten, Grandia, Farshidian, Hutter. *TAMOLS: Terrain-Aware Motion Optimization for Legged Systems.* T-RO 38(5):3395–3413, 2022. https://doi.org/10.1109/TRO.2022.3186804 · https://arxiv.org/abs/2206.14049
21. Tonneau, Del Prete, Pettre, Park, Manocha, Mansard. *An Efficient Acyclic Contact Planner for Multiped Robots.* T-RO 34(3):586–601, 2018. https://doi.org/10.1109/TRO.2018.2819658
22. Acosta, Posa. *Perceptive Mixed-Integer Footstep Control for Underactuated Bipedal Walking on Rough Terrain.* T-RO 2025. https://doi.org/10.1109/TRO.2025.3587998

**腿式 foothold / 地形**
23. Raibert. *Legged Robots That Balance.* MIT Press, 1986. https://openlibrary.org/search.json?title=Legged+Robots+That+Balance&author=Raibert
24. Fankhauser, Bloesch, Gehring, Hutter, Siegwart. *Robot-Centric Elevation Mapping with Uncertainty Estimates.* CLAWAR 2014:433–440. https://doi.org/10.1142/9789814623353_0051
25. Jenelten, Miki, Vijayan, Bjelonic, Hutter. *Perceptive Locomotion in Rough Terrain – Online Foothold Optimization.* RA-L 5(4):5370–5376, 2020. https://doi.org/10.1109/LRA.2020.3007427

**开源实现（仓库存在性用 GitHub API / git ls-remote 实测）**

| 仓库 | 语言 | 对应条目 |
|---|---|---|
| [sbpl/sbpl](https://github.com/sbpl/sbpl) | C++ | 3.1/3.2/3.3/3.8（ARA\*/AD\*/R\*/lattice） |
| [ompl/ompl](https://github.com/ompl/ompl) | C++ | 3.5（RRT\*/BIT\*） |
| [ahornung/humanoid_navigation](https://github.com/ahornung/humanoid_navigation) | C++ | 3.1/3.2（ROS 足步规划，SBPL 后端） |
| [RobotLocomotion/drake](https://github.com/RobotLocomotion/drake) | C++ | 3.6（MIQP 足步） |
| [ethz-adrl/towr](https://github.com/ethz-adrl/towr) | C++ | 3.6（轨迹优化） |
| [leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2) | C++ | 3.6（最优控制） |
| [ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping) | C++ | 3.7（高程图） |
| [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) | C++ | 3.7（2.5D 多图层栅格） |
| [leggedrobotics/elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | Python | 3.7（GPU 高程图） |
| [leggedrobotics/traversability_estimation](https://github.com/leggedrobotics/traversability_estimation) | C++ | 3.7（可通行性） |

---

## 9. 一页纸决策摘要

| 问题 | 结论 |
|---|---|
| 现状为什么猫步 | 状态里没有 `foot`，跨步耦合约束（双足间距）与路径级约束（交叉）无处安放；贪心每步只对支撑脚局部最优 |
| 换什么 | 换**状态空间**，不是换启发式：`(x, y, θ, foot)` + 硬过滤 + 有界次优 anytime 搜索 |
| 为什么不 MIQP | 约束最干净但慢（10–30 步数十秒到分钟）+ 求解器路径不可逐字节复现 |
| 为什么不采样 | 概率完备但确定性弱，且 2–10cm 的窄可行域采样接受率低 |
| 在线怎么落 | 第二阶段叠 AD\* 增量修复；触发条件全部由状态判定，不看时钟 |
| 第一个增量 | 方案 B（足别状态 + 硬过滤），目标是**证伪归因**，不是达标 |
| 最先会坏的夹具 | `test_metrics_full`、`test_reproducibility`；引导层若动还会坏 `terrain_astar_golden.rs` 的 RNG 冻结序列 |
