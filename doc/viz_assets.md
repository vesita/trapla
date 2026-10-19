# trapla-viz 世界盒：开源模型/资产 + 成熟可视化方案清单

面向 `crates/trapla-viz` 从「加载 `map.csv` + `path.csv` 回放」扩成**世界盒**（app 内跑算法、步进回放、
约束可视化、算法对比）时可直接借用/下载的开源资产与方案。

- 本文件只做调研，**不含代码改动**。
- 三项收尾判据贯穿全文：**许可证在来源处核实**、**体积/多边形量实测**、**是否破坏
  `--headless-selftest` / `--screenshot` 的确定性（给出依据）**。
- 标记约定：**【实测】** = 本机跑过命令或解析过文件；**【来源处核实】** = 从原始 URL 读到许可原文；
  **【推断】** = 由源码/规格推得但未直接验证；**未核实** = 没做到，一律不进推荐。

## 0. 核验方法（读者复现这份清单要先知道的两件事）

| 事项 | 事实 | 证据 |
|---|---|---|
| 取网 | 本机 `web_fetch` 工具被 SSRF 策略挡（`URL hostname resolves to a non-public IP address`），但 **`curl` 可通公网**（`raw.githubusercontent.com` / `github.com` / `crates.io` / `api.polyhaven.com` / `ambientcg.com` 均返回 200） | 本会话实测 |
| 许可证核法 | 全部走 `curl` 读**来源处**原文：仓库 LICENSE 文件、资产目录 README、官方 license 页面、crates.io API；GitHub REST API 匿名额度已耗尽（403 rate limit），所以文件清单用 GitHub 的 `tree-commit-info` JSON、体积用 raw 的 `HEAD Content-Length` | 本会话实测 |
| Bevy 0.20 可用性 | **按本机 `~/.cargo/registry/src/index.crates.io-*/bevy*-0.20.0/` 源码判定**，不是按记忆或文档 | 下文的 feature 展开与源码行号 |

`trapla-viz` 的默认 feature 集（`bevy = { version = "0.20.0", features = ["system_font_discovery"] }`
即 `default` + 该项）：`default = [2d, 3d, ui, audio]` → `3d_bevy_render` → `3d_api`
（`common_api, bevy_light, bevy_mikktspace, ktx2, morph_animation, morph, smaa_luts, tonemapping_luts, zstd_rust`）
+ `bevy_gltf` + `gltf_animation` + `bevy_pbr` + `bevy_post_process` + `bevy_anti_alias`；
`common_api` 内含 **`hdr` 与 `png`**。依据：`bevy-0.20.0/Cargo.toml` 的 `default` / `3d` / `3d_bevy_render` /
`3d_api` / `common_api` 定义。【实测】

**默认集之外**（用到就得改 `Cargo.toml`，本会话不许改，故下列项只作为「要付的代价」列出）：
`jpeg`、`meshlet`、`pbr_clustered_decals`（簇式贴花 decal）、`basis-universal`、`bluenoise_texture`、
`dfg_lut`、`bevy_dev_tools`（`dev`）。

---

## 1. 排序建议（先看这张表）

### 第一档：一两天内明显提升观感，且不破坏两种非交互模式

| # | 做什么 | 成本 | 立刻可做 | 破坏自检/截图确定性？ |
|---|---|---|---|---|
| A1 | 落足点从**球**换成**定向 3×5 足矩形**（左右分色、步序刻度、足间连线） | 极低（约 60 行，纯 gizmo） | 是 | 否，依据见 §4.1 |
| A2 | 地形 **hillshade + 高度色带**（改 `height_color` 为「色带 + 朗伯光照」） | 低（约 40 行，无 shader） | 是 | 否，依据见 §4.2 |
| A3 | **约束可视化**：越界步高亮 + 双足间距走廊带 + 足平面法向角着色 | 低到中（每条 30–80 行） | 是 | 否，依据见 §4.3 |
| A4 | **网格线**：`Gizmos::grid_3d` 叠加地面网格（或按 `--grid` 开关） | 极低（约 15 行） | 是 | 否，依据见 §4.4 |
| A5 | **后处理**：`DistanceFog` + `ContactShadows` + `SSAO` + 固定 `Tonemapping` | 低（各 1 个组件，SSAO 需 1 个插件） | 是 | 否，依据见 §4.5 |
| A6 | **HUD 扩容**：步索引/越界计数/法向角读数进现有 Bevy UI 面板 | 低（改 `viz.rs::update_status`） | 是 | 否（纯 UI） |

### 第二档：好看，但体积大 / 风险高 / 要外部工序

| # | 做什么 | 代价 | 风险 |
|---|---|---|---|
| B1 | 双足机器人 GLB（RobotExpressive，CC0，0.44 MiB，骨骼 + 14 段动画） | 需按 3×5 cm 足长标定缩放；Y 轴要与 `HEIGHT_SCALE = 0.05` 一致处理 | 机器人比例与「厘米级足」不匹配；动画与步进回放难对齐 |
| B2 | 程序化天空 `Atmosphere`（零资产）→ 或 HDRI 天空盒（Poly Haven CC0，2k `.hdr` 4.23 MiB） | HDRI 路线要**外部转 cubemap KTX2**（`Skybox` 要求 `TextureViewDimension::Cube`） | 多一道离线工序，且截图会多一条 skybox 管线 |
| B3 | 地形 PBR 材质（ambientCG CC0，1K-JPG zip 9.2 MiB / 1K-PNG zip 17.8 MiB） | 要给地形补 UV（`ATTRIBUTE_UV_0`）+ 三张贴图 | JPG 需非默认 `jpeg` feature；贴图与顶点色二选一或多层混合 |
| B4 | 机器人部件网格（MuJoCo Menagerie，Apache-2.0/BSD-3，STL） | **Bevy 0.20 无 STL/OBJ loader**（实测：`AssetLoader` 只有 gltf/image/hdr/exr/font/audio/shader/world/animation-graph/meshlet）→ 必须 Blender/assimp 转 GLB | 引入离线转换工序与资产入库 |
| B5 | 环境贴图（`EnvironmentMapLight` + Bevy 自带 `pisa_*.ktx2`，CC0 派生） | 需非默认 feature **`dfg_lut`**，否则 LUT 是 `Handle::default()` | 改 feature 会牵动默认依赖面 |
| B6 | egui 调试面板（`bevy_egui 0.43.1`，MIT，依赖 `bevy ^0.20.0`） | 新增依赖树；与现有 Bevy UI 面板职责重叠 | 与 task-7 的菜单改动重叠，需协调 |
| B7 | LOD：`VisibilityRange` 分块（本机核实存在于 `bevy_camera`）或 meshlet | meshlet 是非默认 + 实验性 feature | 对 10 万顶点级的当前网格没有必要 |
| B8 | TAA / AutoExposure / 动模糊 / 体积雾开 jitter | 时域累积或帧相关 | **会破坏截图确定性**，见 §4.6 |

---

## 2. A 档条目：落地方式与验证方式

### A1 落足点改为定向足矩形（本轮最值的单项）

**现状（实测）**：`crates/trapla-viz/src/viz.rs:94` `FOOT_RADIUS = 11.0`，落足点是**半径 11 世界单位
的球**（`viz.rs:404-405`），而 `doc/project.md:18` 写明足是 **3×5 cm 的矩形**；且 `doc/project.md:43`
说足位姿 = 中心点 `(x, y)` + 朝向 `rz`。世界单位与格都是 cm（`scene.rs:104-107`：`X = 列`、`Z = 行`）。
即当前标记形状与尺寸都比真实足大一个量级且丢掉了朝向。

**先例**：
- `ahornung/humanoid_navigation` 的 `footstep_planner`：RViz 里落足点用**定向矩形 + 有效/无效配色**表达。
  许可证 **GPL 3**（来源处核实：`footstep_planner/package.xml` 的 `<license>GPL 3</license>`，以及
  `FootstepPlannerNode.cpp` 头部的 GNU GPL 声明）→ **只借约定，不借任何代码**。
- `leggedrobotics/free_gait` 的 `free_gait_rviz_plugin`：README 原文 "visualize footholds, trajectories,
  support polygons etc."，且可沿时间轴 scrub。许可证 **BSD-3-Clause**（来源处核实：仓库 `LICENSE`，
  Copyright 2015-2017 Péter Fankhauser, ETH Zurich，三条款）。→ 时机轴/scrub 与「support polygon」两点可借。

**Bevy 0.20 落地（本机源码核实）**：`Gizmos::rect(isometry, size, color)`（`bevy_gizmos/src/gizmos.rs:610`）、
`linestrip`（`:497`）、`arrow`（`arrows.rs:129`）、`grid_3d`（`grid.rs:268`）都在；
`Isometry3d` 可用 `Isometry3d::new(translation, Quat::from_rotation_y(rz))` 表达朝向。
建议：矩形 3×5（cm）、填充色按左右脚（沿用现有绿色/橙色 `viz.rs:399-402`）、
未走到的用半透明、越界步用红色描边（见 A3）。

**坑（本项目已实测）**：点位 CSV 只有 `(x, y)`，`rz` 必须重建；`doc/lab.md` 记录只有 `smoothed`
（`Robot::next_committed` 语义）能复现 `Robot::feet()` 的真实末态（`raw` 偏差可达 5.5 rad）。
所以矩形朝向必须按 `smoothed` 重建，不要用步向直接当朝向。

**验证没坏**：① `--headless-selftest` 退出码 0 且三处数值（顶点/三角形/足点数）逐字不变；
② 截图连跑两次 `sha256sum` 相同；③ 人眼判据：任取一段路径，矩形长边方向与相邻同脚点位移方向一致
（可视化里可直接对照折线）。

### A2 地形 hillshade + 高度色带

**现状**：`scene.rs:143-148` 的 `height_color` 只是按高度线性插值绿→亮。
**先例**：GDAL `gdaldem hillshade`（含 Horn 公式与 ZevenbergenThorne 的对比说明）与 `gdal_contour`，
官方文档 <https://gdal.org/en/stable/programs/gdaldem.html>【来源处核实：页面正文含 hillshade/contour/Horn】。
Hillshade 本质就是「坡面法向与光照方向点积」，与 `scene.rs` 里已有 `mesh.compute_normals()` 天然配套。

**落地（两条路，都不改网格拓扑）**：
- 便宜路：在 `build_terrain_mesh` 里用相邻格差分算坡度，把 `height_color` 换成
  「色带查表 × (0.35 + 0.65·max(0, n·L))」。色带自己写 8 段线性插值常量表即可（**不要**引入 matplotlib/
  turbo 数据文件，避免多一份带许可证的资产）。
- 精细路：把片元 shader 挂在 `StandardMaterial`（`base_color_texture` 或自定义 `Material`）上算光照与色带；
  需要给网格补 `ATTRIBUTE_UV_0`（`bevy_mesh/src/mesh.rs:363`，存在）。

**验证没坏**：`--headless-selftest` 的 `Mesh::count_vertices()` 与三角形数**不会变**（加 UV/法向属性不改变
属性计数），`scene.rs:205` 的单测 `mesh_vertex_count_matches_formula`（断言 `count_vertices() == r*c`、
`triangle_count == (r-1)(c-1)2`）照旧通过；截图两次哈希一致。

### A3 约束可视化（本项目差异化重点）

四个子项按「先做贵的判断、再做好看的图」排：**可站立性贴图 → 双足间距走廊带 → 越界步高亮 → 法向角着色**
（后两者其实共用同一份 per-cell 标量场）。

**先例**：
- `ANYbotics/grid_map`：核心机制是「**多层标量场**（layer）叠加在栅格上，用色映射渲染，RViz 插件
  `grid_map_rviz_plugin` 切层查看」。许可证 **BSD-3-Clause**（来源处核实：仓库 `LICENSE`，Copyright 2019 ANYbotics AG，三条款）。→ 我们照抄「约束 = 一张可切换的标量层」这个信息结构。
- `ANYbotics/elevation_mapping`：把高程图发布成上面那种图层（如 `elevation_inpainted`），
  并在 RViz 里切层看；README 原文提到 "change the height layer to another layer"。许可证同为 **BSD-3-Clause**。
  → 「可站立性贴图」就是它的 traversability 层的同类物：**支撑足矩形覆盖格 + 拟合平面法向角 < 20° + 全在图内**，
  阈值唯一事实源是 `crates/trapla-core/src/config.rs`（`doc/lab.md` 口径表）。

**Bevy 0.20 落地**：
1. **可站立性贴图**：CPU 端遍历覆盖格算出 `u8` 标量场，`Image::new_fill`/`Image::from_data` 生成
   `R8Unorm`（或 `Rgba8`）纹理，用一张与地形同尺寸的贴片 mesh（把地形网格的 XZ 复制一份、Y 抬高
   一个 `ε`）贴上去，`StandardMaterial { base_color_texture, alpha_mode: Blend, unlit: true }`。
   依据：`StandardMaterial` 的 `base_color_texture`/`alpha_mode`/`uv_transform` 字段本机核实存在
   （`bevy_pbr/src/pbr_material.rs:58/685/790`）。
2. **走廊带**：`Gizmos::linestrip` 画支撑脚中心线两侧的边界线（最省事）；若要「带」的填充感就生成一条
   triangle strip mesh，用半透明材质。RViz 侧的对应物是 free_gait 的 support polygon。
3. **越界步高亮**：`trapla-lab validate` 已经能逐点判硬约束（`doc/lab.md`），把违规索引传给 viz，
   违规落足矩形换成红色描边 + 面板计数。
4. **法向角着色**：同一份 per-cell 场换成 `acos(n·Y)` 的角度直方图配色（20° 阈值处给明显分界线）。

**成本**：可站立性贴图 1 张 1000×3000 = 3M 格的遍历，纯 CPU 毫秒到几十毫秒级【推断：按 O(格数) 单遍
+ 3×5 卷积估】；其余三项各 30–80 行。

**验证没坏**：新增的是**独立 mesh/贴图/gizmo**，不进 `TerrainMesh`；`--headless-selftest` 只测
`build_terrain_mesh` 那一个网格（`main.rs:99/119-122`），因此它的打印值必须逐字不变 —— 这是这条的判据。
截图：新增不透明/半透明 mesh 会改变像素，但同一命令两次仍应逐字节一致。

### A4 地面网格线

**先例**：Bevy 自带 `examples/dev_tools/infinite_grid.rs`【来源处核实：`bevy` v0.20.0 tag 的
`examples/README.md` 列出 "Infinite grid … suitable as a ground plane for editors"】，但该示例需要
**非默认** `bevy_dev_tools`（`dev`）feature，故照抄思路而不要照抄依赖。
**落地**：`Gizmos::grid_3d(...)`（`bevy_gizmos/src/grid.rs:268`，本机核实存在）在基准高度画一层网格，
`--grid` 开关控制；gizmo 不参与光照/遮挡，天然不影响地形网格与自检数值。
**验证没坏**：同上三连（自检数值不变 / 截图两次同哈希）。

### A5 后处理四件套

**落地（全部本机核实存在于 0.20，且默认 feature 下可用）**：
`DistanceFog`（`bevy_pbr/src/fog.rs`，prelude 导出）、`ContactShadowsPlugin`（`bevy_pbr/src/contact_shadows.rs`）、
`ScreenSpaceAmbientOcclusionPlugin`（`bevy_pbr/src/ssao/mod.rs`，prelude 导出）、`Tonemapping`（`bevy_render/src/view/mod.rs`）。
**先例**：Bevy v0.20.0 自带示例 `examples/3d/{fog,ssao,tonemapping,contact_shadows,bloom_3d}.rs`
（来源处核实：`examples/README.md`）。
**成本**：各 1 个组件 + SSAO 1 个插件；SSAO 需要相机有 depth prepass（Bevy 会自动加），
`ScreenSpaceAmbientOcclusionPlugin` 在 GPU 不支持 5 张 storage texture 时会 warn 并跳过。
**注意**：`Bloom` 带 `#[require(Hdr)]`（`bevy_post_process/src/bloom/settings.rs:32`）→ 加上它会把相机切到
HDR 目标格式；对截图确定性无影响，但会把「默认曝光」变成 HDR 路径，视觉需要重新调。

### A6 HUD 扩容

现有面板是 Bevy UI `Text`（`viz.rs:277-330`）。补步索引 / 越界步计数 / 当前步法向角即可。
**注意（实测）**：Bevy 0.20 **没有世界空间 3D 文字**（`bevy_text` 只有 `Text`（UI）与 `Text2d`）。
所以「把步序编号贴在足边」这件事没有零成本做法：要么走 UI 面板列表（最省），要么
`Camera::world_to_viewport` + UI 绝对定位自己贴（中），要么上 `bevy_egui`（B6）。

---

## 3. 资产台账（每条：URL / 许可证核实处 / 体积 / 多边形量 / 骨骼动画 / Bevy 0.20 判断）

> 体积一律是 HTTP `Content-Length` 实测；多边形/骨骼/动画是**解析 GLB 的 glTF JSON chunk** 得到的实测值。

### 3.1 直接可用（glTF 2.0，默认 feature 就能加载）

| 资产 | URL（下载） | 许可证（核实处） | 体积【实测】 | 多边形量/骨骼/动画【实测】 | Bevy 0.20 判断 |
|---|---|---|---|---|---|
| **RobotExpressive** 低多边形双足机器人 | <https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/models/gltf/RobotExpressive/RobotExpressive.glb> | **CC0 1.0**（核实处：同目录 `README.md` 原文 "Model by Tomás Laulhé … CC0 1.0"） | 463,988 B = 0.44 MiB | 3,237 三角形 / 7,214 顶点 / 74 节点 / 14 mesh / 2 skin / 86 关节 / **14 段动画**（Walking, Running, Idle, Jump, Dance, Punch, Wave, Sitting, Standing, ThumbsUp, No, Yes, Death, WalkJump）/ 无贴图（3 材质纯色） | **可用**：`bevy_gltf` + `gltf_animation` 都在默认集里（`3d_bevy_render`）→ 直接 `SceneRoot` 加载、`AnimationPlayer` 播动画。**要预处理的是尺度**：X/Z 是格(cm)、Y 被 `HEIGHT_SCALE = 0.05` 压了 20 倍（`scene.rs:14`），模型 Y 必须同倍处理，否则机器人相对地形起伏会离谱地高 |
| **CesiumMan** 带皮肤的行走人形 | <https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/CesiumMan/glTF-Binary/CesiumMan.glb> | **CC-BY 4.0** + Cesium logo 商标条款（核实处：`Models/CesiumMan/README.md` 的 Legal 段） | 438,044 B = 0.42 MiB | 4,672 三角形 / 3,273 顶点 / 22 节点 / 1 skin / 19 关节 / 1 段动画（无名）/ 1 张贴图 | 可用（同上）。**必须署名**（CC-BY）且带 Cesium 商标素材，用于内部沙盒无碍，对外交付要注意 |
| **Fox** 带绑定与三段动画的四足 | <https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/main/Models/Fox/glTF-Binary/Fox.glb> | **混合**：模型 CC0 1.0（PixelMannen）+ **绑定/动画 CC-BY 4.0**（tomkranis）+ **转换 CC-BY 4.0**（AsoboStudio/scurest）（核实处：`Models/Fox/README.md` Legal 段） | 162,852 B = 0.16 MiB | 1,728 顶点 / 图元**无索引** → 576 三角形【推断：1728/3】/ 26 节点 / 1 skin / 24 关节 / 3 段动画（Survey, Walk, Run）/ 1 张贴图 | 技术上可用，但**四足且是混合许可**，不进推荐 |

### 3.2 机器人部件网格（MuJoCo Menagerie：许可干净，但要离线转换）

仓库：<https://github.com/google-deepmind/mujoco_menagerie>；**每个模型目录在仓库根 `LICENSE` 里有独立条款段**。

| 模型 | 许可证（核实处：根 `LICENSE` 的对应段） | 网格与体积【实测】 | Bevy 0.20 判断 |
|---|---|---|---|
| `booster_t1`（人形） | **Apache-2.0** | 24 个 `.stl`；**两只脚的网格各 979,684 B = 0.93 MiB**（`left_foot_link.stl` / `right_foot_link.stl`，HEAD 实测）；抽测 14 个网格合计 3.42 MiB（其余未逐个称重） | **不可直接加载**：Bevy 0.20 无 STL/OBJ loader（实测 `AssetLoader` 实现只有 gltf / image / hdr / exr / font / audio / shader / world / animation-graph / meshlet）→ 需 Blender 或 assimp 转 GLB 入库 |
| `iit_softfoot`（自适应软足，**与本项目足部主题最贴**） | **BSD-3-Clause**（核实处：`iit_softfoot/LICENSE` 逐字三条款 + `iit_softfoot/README.md` 写明 "provided under a BSD-3-Clause License"） | `meshes/` 下 10 个文件；实测 `tacco.stl` 0.27 MiB、`qb_move_advanced_for_mate.stl` 0.15 MiB（其余未逐个称重） | 同上报 STL 转换问题；无骨骼、无动画（刚体部件） |
| `agility_cassie`（双足） | **MIT**（核实处：根 `LICENSE` 对应段为 MIT 文本） | 未逐个称重（**未核实**） | 同上：STL → 需转换 |
| `unitree_g1` / `unitree_h1` / `berkeley_humanoid` / `anybotics_anymal_c` | **BSD 三条款样式**（核实处：根 `LICENSE` 对应段；逐字核到第 1、2 条，第 3 条未展开 → 置信度较高但非逐字全核） | 未逐个称重（**未核实**） | 同上：STL → 需转换 |

### 3.3 地形材质（CC0，来源处核实）

| 资产 | URL | 许可证（核实处） | 体积【实测，官方 API 返回】 |
|---|---|---|---|
| ambientCG `Rock030`（岩石） | <https://ambientcg.com/view?id=Rock030>；API：<https://ambientcg.com/api/v2/full_json?type=Material&id=Rock030&include=downloadData> | **CC0 1.0**（核实处：<https://docs.ambientcg.com/license/> 正文 "Creative Commons CC0 1.0 Universal License"，且资产页写 "CC0 license … free to use without attribution"） | 1K-JPG zip 9,677,446 B = 9.23 MiB；1K-PNG zip 18,666,311 B；2K-JPG 33,491,328 B；4K-JPG 125,103,042 B；8K-JPG 480,212,586 B |
| Poly Haven 材质/HDRI（通用来源） | <https://polyhaven.com/license>（许可页）；示例资产页 <https://polyhaven.com/a/kloppenheim_06_puresky> | **CC0**（核实处：<https://polyhaven.com/license> 正文多处 CC0 说明） | 见 3.4 |

**Bevy 侧注意（本机核实）**：默认集含 `png`，**不含 `jpeg`**。要直接用 ambientCG 的
`*_JPG.zip` 就得加非默认 `jpeg` feature；用 `*_PNG.zip` 则默认可用（代价是 zip 大约翻倍）。

### 3.4 天空 / 环境贴图

| 资产 | URL | 许可证（核实处） | 体积【实测】 | Bevy 0.20 判断 |
|---|---|---|---|---|
| Poly Haven `kloppenheim_06_puresky`（晴空 HDRI） | API：<https://api.polyhaven.com/files/kloppenheim_06_puresky> | **CC0**（核实处：<https://polyhaven.com/license>） | 2k `.hdr` 4,434,394 B = 4.23 MiB；4k `.hdr` 17,385,661 B；8k `.hdr` 66,959,416 B；16k 204,043,144 B；24k 433,225,600 B | **`.hdr` 能加载**（`hdr` 在 `common_api` 里），**但不能直接当天空盒**：`Skybox` 要求图像的 view dimension 是 `TextureViewDimension::Cube`（本机核实：`bevy_core_pipeline/src/skybox/mod.rs` 的 `sanity_check_skybox_image_and_warn`）→ 等距柱状 `.hdr` 必须离线转 cubemap KTX2 |
| Bevy 自带环境贴图 `pisa_diffuse/specular_rgb9e5_zstd.ktx2` | <https://raw.githubusercontent.com/bevyengine/bevy/v0.20.0/assets/environment_maps/pisa_specular_rgb9e5_zstd.ktx2>（specular 6,792,052 B = 6.48 MiB，HEAD 实测）；同目录还有 `sky_skybox.ktx2`、`info.txt` | **CC0 派生**（核实处：`bevy` v0.20.0 的 `CREDITS.md` 原文 "Environment maps (`.hdr` files) from HDRIHaven (CC0 license)"；`assets/environment_maps/info.txt` 给出从 Khronos `pisa.hdr` 转换的命令行） | specular 6.48 MiB（实测）；diffuse 未逐个称重（**未核实**） | **KTX2 + zstd 默认可用**（`ktx2`、`zstd_rust` 在 `3d_api`）；但用 `EnvironmentMapLight` 需要非默认 **`dfg_lut`** feature —— 缺它时 `bevy_pbr` 只塞一个 `Handle::default()`（本机核实：`bevy_pbr/src/lib.rs:348`）→ 镜面环境光无效 |
| 程序化天空 `Atmosphere`（零资产） | <https://github.com/bevyengine/bevy/blob/v0.20.0/examples/3d/atmosphere.rs> | Bevy 示例（MIT/Apache-2.0 双许可） | 0 B | **可用且零成本**：`Atmosphere` 在 `bevy_light`（`atmosphere.rs`），`bevy_pbr` 里无 feature 门（`pub use atmosphere::*`） |

### 3.5 足印 / 落足标记

**没有找到可直接用的开源标准资产**（未核实 = 我按关键词检索未命中来源处许可证可查的条目）。
推荐**程序化生成**：`Gizmos::rect` 画 3×5 定向矩形（A1），或代码里生成一个 1×1 的矩形 mesh + 逐足
`Transform`（要参与光照/遮挡时才需要 mesh）。先例见 A1（footstep_planner 约定 / free_gait 插件）。

---

## 4. 确定性判定（每条都给依据）

**两条判据分开说**：
- **D1（同一命令逐字节可复现）**：同一条 `--screenshot … --frames N` 命令重复跑，PNG 的 sha256 相同。
- **D2（与帧数/时间无关）**：改 `--frames`、改起点时刻都不改变同一视角的成像。

`--screenshot` 打开 `RenderPlugin::synchronous_pipeline_compilation`（本机核实字段存在：
`bevy_render/src/lib.rs:164`）保证管线在首帧前同步编译 —— 这条是既有的防「只截到清屏色」的保护，
**新增后处理不会绕过它**，代价只是同步编译时间变长。

### 4.1 A1 落足矩形：不破坏
纯 gizmo 绘制，不碰 `build_terrain_mesh`。自检只打印地形网格与足点数（`main.rs:99-122`），足点数不变。

### 4.2 A2 hillshade/色带：不破坏
只改 `ATTRIBUTE_COLOR` 的取值或新增 `ATTRIBUTE_UV_0`；`Mesh::count_vertices()`（每属性计数）与索引数不变
→ `scene.rs:205` 单测与自检打印值都不变。着色是纯确定性函数（无噪声、无时间）。

### 4.3 A3 约束可视化：不破坏
新增独立实体/贴图，不进 `TerrainMesh`；标量场计算是纯函数（输入地图+路径 → 输出场），
不含 RNG/时间。唯一要注意的是**不要在贴图生成里用 HashMap 遍历顺序决定像素**（用数组下标遍历即可）。

### 4.4 A4 网格线：不破坏
`Gizmos::grid_3d` 位置由常量与场景参数决定，无时间量。

### 4.5 A5 后处理：不破坏（逐条源码依据）
| 效果 | 判定 | 依据（本机源码） |
|---|---|---|
| `DistanceFog` | 不破坏（D1+D2） | `bevy_pbr/src/fog.rs`，无 `frame_count`/时间输入 |
| `ContactShadows` | 不破坏 | `bevy_pbr/src/contact_shadows/*` 无 `frame_count` |
| `SSAO` | **不破坏** | `ssao.wesl:43` 的 `index += 288u * (globals.frame_count % 64u)` 被 `@if(TEMPORAL_JITTER)` 包住，而 `TEMPORAL_JITTER` 来自视图上的 `TemporalJitter`（TAA 才插）→ 不开 TAA 时噪声与帧数无关 |
| `Tonemapping` | 不破坏 | `bevy_render/src/view/mod.rs`，纯 LUT |
| `Atmosphere` | 不破坏 | `bevy_pbr/src/atmosphere/*.wesl` 无 `frame_count`/`globals.time` |
| `Bloom` | 不破坏 | `bevy_post_process/src/bloom/` 全目录无 `frame_count`；`#[require(Hdr)]` 只改目标格式 |
| `FogVolume` | **默认不破坏** | `FogVolume.jitter` 默认 `0.0`（`bevy_light/src/volumetric.rs:68`）；一旦手动设 jitter>0，`volumetric_fog.wesl` 会用 `interleaved_gradient_noise(position.xy, globals.frame_count)` → 变帧相关 |

### 4.6 会破坏确定性的效果（写进排除项的理由）

| 效果 | 破坏原因（依据） |
|---|---|
| `TemporalAntiAliasing` | `#[require(TemporalJitter, MipBias, DepthPrepass, MotionVectorPrepass)]`（`bevy_anti_alias/src/taa/mod.rs:113`）+ 历史缓冲 → 收敛依赖帧数；且它插的 `TemporalJitter` 会把 SSAO 变成帧相关（`ssao.wesl:43`） |
| `MotionBlur` | `bevy_post_process/src/motion_blur/motion_blur.wesl` 用 `frame_count` |
| `AutoExposure` | 64 bin 直方图逐帧自适应曝光（`bevy_post_process/src/auto_exposure/settings.rs:17-22` 文档），需要 compute；【推断】曝光跨帧累积 → 帧数敏感 |
| `FogVolume` 开 jitter | 见 4.5 最后一行 |
| 改 `HEIGHT_SCALE` / `stride` / 顶点算式 | 不改变退出码，但会**改变 `--headless-selftest` 打印的顶点/三角形数与截图基线** → 若外部有基线比对，视为破坏 |

### 4.7 每个推荐项的验证配方（复制即可）

```bash
cd <trapla 检出根>
CARGO_TARGET_DIR=$HOME/.cache/trapla-viz-target cargo build -p trapla-viz
BIN=$HOME/.cache/trapla-viz-target/debug/trapla-viz
M=data/csv/map.csv; P=data/csv/path_result.csv

# 1) 无头自检：退出码 0，且「地形网格实测 / 路径点数 / 对照」三处数值与改动前逐字一致
$BIN --map $M --path $P --headless-selftest; echo "exit=$?"

# 2) 单测（含 scene.rs 的网格计数断言）
CARGO_TARGET_DIR=$HOME/.cache/trapla-viz-target cargo test -p trapla-viz

# 3) 截图 D1：同一条命令两次，哈希必须相同
$BIN --map $M --path $P --screenshot /tmp/v1.png --frames 60 && sha256sum /tmp/v1.png
$BIN --map $M --path $P --screenshot /tmp/v2.png --frames 60 && sha256sum /tmp/v2.png

# 4) 截图 D2：改帧数，若哈希变化则该效果帧相关（据此把 jitter/TAA/AutoExposure 排除）
$BIN --map $M --path $P --screenshot /tmp/v3.png --frames 30 && sha256sum /tmp/v3.png
```

---

## 5. 被排除项与理由

| 被排除项 | 理由（含核实处） |
|---|---|
| **BrainStem** 关节机器人 GLB（3,194,848 B = 3.05 MiB，带骨架） | 许可证是 **Poser EULA**（核实处：`Models/BrainStem/README.md` Legal 段 "Smith Micro Software, Inc. [Poser EULA]"）→ 非开源，**不得进推荐**，也不要拷进仓库 |
| `footstep_planner` / `humanoid_navigation` 的**代码** | **GPL 3**（核实处：`footstep_planner/package.xml` 的 `<license>GPL 3</license>`；`FootstepPlannerNode.cpp` 头部 GNU GPL 声明）→ 只借「定向矩形 + 有效/无效配色」的约定，不引入任何代码 |
| `bevy_panorbit_camera` 0.35.1 | 依赖 **`bevy ^0.19`**（核实处：crates.io `/crates/bevy_panorbit_camera/0.35.1/dependencies`）→ 与本仓 bevy 0.20 不兼容；且本项目已有自研 `orbit_camera.rs`，无必要 |
| `Fox` | **许可混合**：模型 CC0 但绑定/动画与转换是 CC-BY 4.0（核实处：`Models/Fox/README.md`）→ 需署名；且四足，与本项目双足足印主题不符 |
| Meshlet（`examples/3d/meshlet.rs`） | feature `meshlet` 非默认且官方标为实验性；对当前 10 万顶点级网格无收益 |
| `TAA` / `AutoExposure` / `MotionBlur` / `FogVolume.jitter>0` | 会破坏截图确定性，依据见 §4.6 |
| 簇式贴花 decal 当「足印贴花」 | 需要非默认 feature `pbr_clustered_decals`；且足印用 gizmo/mesh 就够了 |
| 24k/16k HDRI（433 MB / 204 MB .hdr） | 体积过大，且 Skybox 路线本身还要转 cubemap |
| 用 Bevy 示例的 `infinite_grid` 直接抄 | 需要非默认 feature `bevy_dev_tools`（`dev`）→ 抄 `grid_3d` 那几行即可 |
| `breeze/assets/character/*.glb`（3 个角色 GLB） | **授权不明**（breeze 仓内未见这些模型的许可证声明，本会话按硬约束未改动该仓、也未逐一核实其来源）→ 按「核不到就不进推荐」处理 |
| `bgolus` 的网格着色器文章 / Shadertoy 等第三方技法页 | 取网返回 **403（Cloudflare）**，**未核实** → 不给可点结论；网格/等高线抗锯齿改用 GDAL 文档与自写 fwidth 方案表述 |
| SBPL 的代码 | 许可证其实是 **BSD-3-Clause**（核实处：`src/include/sbpl/utils/mdp.h` 头部 "Copyright (c) 2008, Maxim Likhachev … Neither the name of the Carnegie Mellon University …"），**许可没问题**；排除理由是它是 C++ 规划库、与 Bevy 世界盒无交集（其价值已被本项目 `doc/algorithm_survey.md` 覆盖） |

---

## 6. 未验证清单（诚实交代）

1. **未逐条 `cargo build`**：本会话受硬约束限制（不 build、不装依赖），所有「Bevy 0.20 可用性」都是
   读本机 registry 源码 + feature 展开判定的结果，**没有编译验证**。
2. **模型体积未全部称重**：Menagerie 的若干模型（`agility_cassie`、`unitree_g1/h1`、`berkeley_humanoid`、
   `anybotics_anymal_c`）只登记了许可，网格体积标「未核实」；`booster_t1` 只称了 14/24 个网格。
3. **`pisa_diffuse_rgb9e5_zstd.ktx2` 未称重**（只称了 specular 6.48 MiB）。
4. **`AutoExposure` 的帧相关性是【推断】**（读的是组件文档与实现结构，没有跑图验证）。
5. **CC-BY 资产的署名义务**（CesiumMan 的 Cesium logo 商标条款）**未做法律解读**，只是把来源处原文列出。
6. **未做任何 HTTP 逐条「点开页面看渲染效果」**：所有条目都只到「URL 可达 + 内容原文可读」这一级；
   资产好不好看、动画是否合适，需要你把 GLB 拖进 Blender/查看器确认。
7. **Poly Haven / ambientCG 的资产级下载没有真的下载**（体积来自官方 API 返回值，不是本地 `stat`）。
