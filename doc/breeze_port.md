# breeze → trapla 可移植组件清单与移植方案

目标：把 breeze（Bevy **0.19**，第三人称 3D 沙盒）里能用的部分搬到 trapla-viz（Bevy **0.20**），
让 trapla 的算法测试环境获得「世界盒」式交互沙盒能力：选场景 / 放起终点 / 选算法 / 步进回放 /
约束可视化 / 算法对比。

本文只回答「搬什么、怎么改、多大代价、怎么验证没坏」。**不含**逐行改法（那是实施阶段的事）。

- 证据强度标注：**[实读]** = 本轮读过源码并给出 `文件:行`；**[推断]** = 由实读的代码链推出，未跑构建；
  **[未核实]** = 本机拿不到证据，不做断言。
- 硬约束：两个仓都只读，未跑 `cargo build`，未改任何 breeze 文件。

路径约定：`breeze/…` 与 `trapla/…` 指两个**同级检出**（本文按 `breeze/` 与 `trapla/` 的工作区布局写相对路径）；
cargo registry 证据按 `$CARGO_HOME/registry/src/<registry>/<crate>-<ver>/…` 写相对位置，版本号随文给出。

---

## 0 结论摘要

| # | 组件 | 结论 | 工作量粗估 |
|---|---|---|---|
| A | vendored `smooth-bevy-cameras` crate | **搬**（要改 Cargo.toml + 核实 0.20 漂移） | 0.5–1 人日（大半是编译验证） |
| A2 | `breeze/src/managers/camera.rs`（多相机注册表/切换/过渡） | **不搬**（trapla-viz 单相机） | — |
| B | 四层 managers 的**骨架**（`mod.rs` 分层 + 每层一个 Plugin） | **搬骨架，不搬层** | 0.5 人日 |
| C | `sandbox/plugin.rs` 的 HUD + 调试热键模式 | **改后搬**（内容重写，rapier 线框那条丢弃） | 0.3–0.5 人日 |
| D | `ui.rs` 的 `GamePhase` 状态机 | **搬** | 0.3 人日 |
| D2 | `ui.rs` 的 egui 菜单 | **现在不搬**（bevy_egui 版本卡在 `^0.19`） | 阻塞 |
| E | `state.rs` 存档快照 | **只搬模式，不搬代码** | 0.5 人日（可选） |
| F | `fonts.rs` + 思源宋体资产 | **搬（改）**，但**不解决 ICU4X 告警** | 0.5 人日 |
| G | `mechanics/interact.rs` 交互原语 | **概念搬，实现重写**（无 rapier） | 1–2 人日 |
| H | `tools/` 网格纹理生成器（`grid.png`） | **搬生成器，不搬 PNG** | 0.2 人日 |
| I | `docs/pitfalls/` 13 篇踩坑笔记 | **迁移期参考，不落进 trapla 文档体系** | — |

最先有可见效果的是 **§4 步骤 3（选场景）**；最快见效（一行改动）的是 **步骤 0**。

---

## 1 逐组件盘点

### A. 相机：vendored `smooth-bevy-cameras`（breeze）

**职责** [实读]
- `LookTransform`（eye/target/up）与 `Smoother`（指数平滑）两层，同步系统把前者写进场景 `Transform`：
  `breeze/crates/smooth-bevy-cameras/src/look_transform.rs:12-18`（插件）、`:156-167`（同步+平滑）。
- 四个控制器 fps / orbit / unreal / third_person，统一为「`default_input_map`（原始输入→消息）+
  `control_system`（消息→姿态）」：`src/controllers/orbit.rs:37-48`、`src/controllers/orbit.rs:181-228`；
  第三人称见 `src/controllers/third_person.rs:53-96`（控制器字段）、`:98-118`（bundle）、`:224-267`（控制）。
- 公共鼠标锁：`src/cursor.rs:55-71`（`CursorLock` 组件，`keys: Vec<KeyCode>`）、`:77-93`（插件）、
  `:97-108`（启动锁定）、`:111-147`（Trigger/Flip 切换）。X11 不支持 `Locked` 会降级 `Confined`，
  Wayland 下跳过光标复位 —— 见 `breeze/docs/pitfalls/cursor-platform-grab.md:1-10`。
- 角度工具 `LookAngles`：`src/look_angles.rs:16-71`。
- 模块导出面：`src/lib.rs:120-127`。

**依赖 breeze 的哪些业务** [实读]：**零**。该 crate 只依赖 bevy（`+ approx`，可选 `serde`），
见 `breeze/crates/smooth-bevy-cameras/Cargo.toml:12-19`；第三人称控制器只认自己的
`ThirdPersonTarget` 标记组件（`third_person.rs:41-44`），不认识 breeze 的 player。

breeze 里的业务胶水在 `breeze/src/managers/camera.rs`：多相机注册表 `CameraRegistry`
（`:45-53`）、出生两台相机（`:129-185`）、把 `InputActions` 转成 `ControlMessage`（`:190-229`）、
自定义控制循环（`:240-291`）、数字键切换 + 过渡（`:300-345`、`:349-376`）。

**移植到 trapla 的映射**
- crate：**能直接搬**，但要改 **6 处**：
  1. `Cargo.toml:12-19` 用 `bevy.workspace = true` / `approx.workspace = true` / `serde.workspace = true`，
     trapla 工作区没有这三个 workspace 依赖（`trapla/Cargo.toml:18-23` 只有 trapla-core/io/clap/serde/serde_json）
     → 必须改成显式版本，bevy 改 `0.20`。
  2. `dev-dependencies` 同步改。
  3. 0.19→0.20 的 API 漂移核实（见下）。
  4. 与 trapla-viz 现有 `orbit_camera.rs` 的**类型重名**（`LookTransform` / `Smoother`）→ 必须二选一，
     不能并存。
  5. 该 crate 的 `LookTransformPlugin` 与 trapla-viz 自己的 `look_transform_system` 是同一职责，
     搬运后要删掉后者。
  6. MIT 许可文件要一起带（见 §3 许可证）。
- `managers/camera.rs`：**不建议搬**（单相机场景用不上注册表/过渡/多目标）。

**工作量**：0.5–1 人日，其中大半是编译核实。

**风险**
- **API 版本漂移**：搬运面用到的 API 我逐条在 bevy 0.20 源码里核对过，**都存在** [实读]：
  `Bundle` derive（`bevy_ecs_macros-0.20.0/src/lib.rs:58`）、`Message`/`Event` derive（同文件 `:586`/`:557`）、
  `MessageReader`/`MessageWriter`（`bevy_ecs-0.20.0/src/message/message_reader.rs:34`、`message_writer.rs:62`）、
  `App::add_message`（`bevy_app-0.20.0/src/app.rs:427`）、`Single`（`bevy_ecs-0.20.0/src/system/query.rs:2995`）、
  `CursorOptions`（`bevy_window-0.20.0/src/window.rs:752`）、`MouseScrollUnit`（`bevy_input-0.20.0/src/mouse.rs:135`）、
  `Reflect` derive（`bevy_reflect_derive-0.20.0/src/lib.rs:416`）、`App`/`PreUpdate` 等 prelude 路径。
  **然而「符号存在」≠「签名兼容」**：本轮不跑构建，**未编译核实** [未核实]。留一条最低成本的验证命令见 §4。
- 许可：MIT（见 §3）。
- 编译时间：crate 本身很小，不新增依赖树；但 `.otf` 等资产不参与。
- 功能净损失：vendored `OrbitCameraController` **没有 `home`/`reset`**（实读全文件无 `home|reset`），
  也没有 trapla-viz 的键盘旋转/俯仰键位；pitch 夹取范围从 trapla 的 `0.05..1.5`
  （`trapla/crates/trapla-viz/src/orbit_camera.rs:126-127`）变成 `±(π/2−0.01)`
  （`src/look_angles.rs:47-51`）。

### A2. `managers/camera.rs`（breeze）— 不搬

见上；trapla-viz 单相机 + 固定 home，搬它只会多出注册表/过渡两套没人用的状态。

### B. 四层 managers 的骨架（breeze `src/managers/`）

**职责** [实读]：`mod.rs:38-53` 的 `GameManagerPlugin` 按依赖顺序注册
Ui → User → World → Character → Camera → Snapshot → Player → Mechanics → Sandbox；
`mod.rs:1-18` 用一张图声明「上层可依赖下层、反之不可」。
各层：`user.rs:19-45`（`InputActions` 语义动作资源）、`:80-86`（插件）、`:89-134`（唯一读原始输入的地方）；
`world.rs:15-19`（`WorldState` 追踪）、`:23-36`（插件）、`:39-49`（静态场景）、`:83-108`（地面）、
`:111-136`（光照）、`:139-201`（道具）；`character.rs:44-55`（插件）、`:60-103`（出生）、
`:106-121`（切换模型）、`:127-149`（重建模型子节点）、`:152-164`（复位）。

**依赖 breeze 业务** [实读]：`character.rs` 依赖 rapier（`RigidBody::KinematicPositionBased` +
`Collider::capsule_y` + `KinematicCharacterController`，`:89-92`）、`player::{PlayerMarker,PlayerMovement}`、
`CharacterAnimator`、GLB 资产（`:38-42`）；`world.rs` 依赖 rapier `Collider`（`:104`、`:157`、`:183`）
与 `state::{NetworkId,PropKind,Replicated}`（`:11`）。

**移植映射**：**只搬「分层 + 每层一个 Plugin + 上层依赖下层」这一条骨架与 `InputActions` 式的输入抽象**；
四个具体层都不搬。trapla-viz 的对应物是 `scene.rs`（几何/网格）+ `viz.rs:226-275`（`setup_scene` 建场景）。

**工作量**：0.5 人日。

**风险**：低。注意 `GlobalAmbientLight` 资源在两边都有（breeze `world.rs:25-29`；
bevy 0.20 在 `bevy_light-0.20.0/src/ambient_light.rs:62`），而 trapla-viz 现在把 `AmbientLight`
挂在相机上（`trapla-viz/src/viz.rs:263-267`，注释见 `:252`）——**别同时上两套**。

### C. sandbox HUD + 调试热键（breeze `src/sandbox/plugin.rs`）

**职责** [实读]：`:44-60` 插件（HUD 只在 `Playing` 出生、`run_if` 门控）、`:63-104` 左上 FPS/坐标/相机 HUD +
底部 HelpBar、`:110-174` 每 0.5s 节流刷新（`:134-137` 注释明写这是为了绕开 ICU4X 噪音）、
`:177-184` 反引号切换物理线框。

**依赖 breeze 业务** [实读]：`bevy_rapier3d::render::DebugRenderContext`（`:20`、`:177-184`）、
`CameraRegistry` + `ThirdPersonController`（`:23`、`:117`、`:151-157`）、`PlayerMarker`（`:115`）、`GameFonts`（`:22`）。

**移植映射**：**模式可搬，HUD 内容要重写**。保留「一个 `HudState` 资源 + 一个 `Hud` 标记组件 +
节流刷新 + 热键派发」的结构；文本换成「回放进度 / 地形形状(行×列·stride·顶点数) / 指标值」。
rapier 线框那条**不搬**（trapla 无 rapier）。

**工作量**：0.3–0.5 人日。

**风险**：低。`Text` / `TextFont` / `FontSize::Px` / `FontSource` 在 bevy 0.20 都存在
（`bevy_text-0.20.0/src/text.rs:783`、`:284`）[实读]。节流那条**不要**照搬到 trapla-viz：它治的是日志噪音不是性能，
真正的解见 §2 问题 ①。

### D. `ui.rs` 状态机 + egui 菜单（breeze）

**职责** [实读]：`:24-32` `GamePhase { MainMenu, Playing }`（`derive(States)`）；
`:36-46` `MenuAction` / `MenuSelection { NewGame, Continue }`（菜单写选择，`Playing` 的 `OnEnter` 消费，
消费点在 `state.rs:121-127`）；`:51-63` `UiPlugin`（系统放 `EguiPrimaryContextPass`，注释解释放 `Update` 会首帧 panic）；
`:71-105` 菜单绘制；`:107-132` 把思源宋体注册进 egui。
`:68-70` 记了一条 egui 0.34 `CentralPanel` 与 bevy_egui 0.40 帧模型不兼容的坑，改用 `Area`。

**依赖 breeze 业务** [实读]：`crate::fonts::UI_FONT_PATH`（`:15`）、`crate::state::save_file_exists`（`:16`、`:81`）；
依赖 `bevy_egui 0.41.1`（`breeze/Cargo.toml:26`、`breeze/Cargo.lock:772-775`，其 egui 为 0.35，
`Cargo.lock:2517-2520`）。

**移植映射**
- **状态机直接搬（几乎零改动）**：`States` / `init_state` / `in_state` / `NextState` / `OnEnter`
  在 bevy 0.20 都在（`bevy_state-0.20.0/src/app.rs:40`、`condition.rs:103`、
  `state/resources.rs:181`、`state/transitions.rs:19`）[实读]。
- **egui 菜单现在不要搬**：本机 crates.io 索引缓存里 `bevy_egui` 最新为 `0.41.1`，其 bevy 依赖是 `^0.19`
  （`0.40.0`–`0.41.1` 均如此）[实读，索引缓存]；trapla-viz 是 bevy 0.20 → 直接搬会版本冲突。
  是否存在 0.20 兼容版本 [未核实]（本机无法访问 crates.io 核对上游）。
  替代：用 Bevy UI 复刻菜单（trapla-viz 已在用 `Node`/`Text`，`viz.rs:277-326`）。

**工作量**：状态机 0.3 人日；egui 路线 = 阻塞。

**风险**：状态机本身无风险；egui 的 `FontData::from_owned(...).into()` 是 egui 0.35 的 API（版本敏感）；
`ui.rs:20-22`、`:118` 从 **当前工作目录相对的 `assets/`** 读字体，与 trapla-viz「路径一律显式给出、不靠 CWD 猜」
（`trapla-viz/README.md:22`、`cli.rs:1-3`）冲突。

### E. `state.rs` 存档快照（breeze）

**职责** [实读]：`:28-47` `NetworkId`（稳定实体标识）/`Replicated`/`PropKind` 与 `SAVE_VERSION`；
`:49-79` `GameState { version, player, entities }`；`:82-95` 存档路径（XDG 回落 HOME）；
`:101-118` 插件；`:130-175` 存（遍历权威状态→postcard→写盘）；`:178-276` 读
（按 `NetworkId` 匹配：覆盖/缺失 spawn/多余 despawn，`:244-273`）；`:278-321` 往返单测；
`:323-360` 缺失实体按模板重建。
`:1-12` 的论证「存档与联机是同一个问题」是这份设计最值钱的部分。

**依赖 breeze 业务** [实读]：rapier `Velocity`（`:135`、`:253-256`、`:344-358`）、
`player::{CharacterAnimator,PlayerMovement}`、`managers::character::{apply_character_model,…}`、`postcard`。

**移植映射**：**只搬模式，不搬代码**。可复用的是三条：
① 稳定 id（`NetworkId`）作为跨会话主键；② 版本号 + 不匹配就拒绝；
③ 「更新 / 补齐 spawn / 删除多余」三路收敛。trapla-viz 需要的会话快照是
「场景 id + 算法 id + 起终点 + 回放位置」，与 rapier/角色无关。
**不建议**为此给 trapla-viz 加 `postcard`：该 crate 现在只依赖 bevy/clap/serde_json
（`trapla-viz/README.md:9-11`），先用 CLI 参数 + `serde_json` 就够。

**工作量**：0.5 人日（可选，世界盒要记住上次选择时才做）。

**风险**：低。主要风险是**打破 trapla-viz 的「不依赖 core/io」约束**——加依赖前先问一句是否真需要持久化。

### F. `fonts.rs` + 字体资产（breeze）

**职责** [实读]：`:10` 资产路径常量 `fonts/SourceHanSerifSC-Regular.otf`；
`:13-26` `GameFonts { ui: Handle<Font> }`（`FromWorld` 里 `asset_server.load`）；`:28-34` 插件。
egui 侧字节路径在 `ui.rs:107-132`（`std::fs::read("assets/…")` + `FontDefinitions::default()`）。

**依赖 breeze 业务**：**零**（纯 AssetServer）[实读]。

**移植映射**：**值得搬，但要改**。改 4 处：
1. trapla 目前**没有 `assets/` 目录**（实测无 `.gitattributes`、无 `assets/`）→ 新建资产目录，
   并注意 Bevy `AssetPlugin` 默认按 **CWD 相对** 解析 `assets/`；trapla-viz 的原则是显式路径，
   需要显式设置或改用 `include_bytes!` / `embedded_asset!`（bevy 0.20 有该宏：
   `bevy_asset-0.20.0/src/io/embedded/mod.rs:352`）[实读]。
2. 用它替换 trapla-viz 现在的 `FontSource::Family("Noto Sans CJK SC")`
   （`trapla-viz/src/viz.rs:102-103`、`:279`）→ 换成 `FontSource::Handle(handle)`；
   换来的是**不再依赖目标机器装了 Noto Sans CJK SC**，截图产物跨机器一致。
3. 许可文件要随字体一起带（见 §3）。
4. 体积：24,543,332 B（≈23.4 MiB），breeze 用 git-lfs 跟踪（`breeze/assets/.gitattributes`），
   trapla 现在**没有 LFS**。
- **它不解决 ICU4X 告警**，理由与证据见 §2 问题 ①。

**工作量**：0.5 人日（含截图目视验证）。若做字体子集化可显著减小体积，但需要新增工具链，本轮**未评估** [未核实]。

**风险**：① 许可（§3）；② 24 MB 进仓库与 LFS 决策；③ CWD 相对资产路径；
④ 若走 egui 路线还要额外做 `ui.rs:107-132` 那份字节加载。

### G. `mechanics/interact.rs`（+ `kick.rs`）

**职责** [实读]：`:36-45` `Interactable` + `InteractMessage`；`:47-58` 最大交互距离资源；
`:67-83` 插件；`:86-112` 屏幕底部提示 UI；`:154-225` 每帧从**活跃相机**发射屏幕中心射线 →
命中可交互体 → 高亮（改该实体**独有**材质的 emissive，`:129-151`）→ 提示文字 → E 键发消息。
`mechanics/mod.rs:1-15` 写明「加新机制」的模板；`kick.rs:1-50` 是第一个机制演示。

**依赖 breeze 业务** [实读]：`bevy_rapier3d::ReadRapierContext` + `QueryFilter`（`:26-27`、`:157`、`:182-190`）、
`CameraRegistry`（`:158-165`）、`PlayerMarker`（`:174`）、`GameFonts`（`:30`、`:86`）。

**移植映射**：**概念搬，实现重写**。「世界盒」的「放起终点」本质就是这条射线拾取。
trapla-viz 没有 rapier → 用 bevy 自带的射线/`MeshRayCast`，或更省事的**解析反解**：
地形是世界坐标下的规则网格（`trapla-viz/src/scene.rs:19-37` 的降采样 + `:74` 的 `world_from_map`），
屏幕射线对高度场步进求交即可，无需物理后端。

**工作量**：1–2 人日。

**风险**：rapier 是硬阻断（不过 trapla-viz 本就不该引物理引擎）；高亮约定要求「材质独有」
（`interact.rs:20-23`），trapla-viz 的地形用单一共享材质（`viz.rs:234-241`），高亮策略要重新设计。

### H. 网格纹理生成器（breeze `tools/`）

**职责** [实读]：`tools/src/breeze_tools/__init__.py:19-25`（调色板 + 输出路径按
`Path(__file__).resolve().parents[3]` 定位仓库根）、`:28-34`（细/粗线像素规则）、
`:37-63`（zlib+struct 手写 PNG）、`:66-75`（`--size/--cell/--major` CLI）；
入口注册在 `tools/pyproject.toml:12-14`（`generate-grid-texture = "breeze_tools:main"`）；
用法 `tools/README.md:12-14`。产物 `breeze/assets/textures/grid.png`（62,623 B）由
`breeze/.gitignore`（`assets/textures/`）排除，不入库；Rust 侧唯一耦合是
`breeze/src/managers/world.rs:89-91` 的 `asset_server.load("textures/grid.png")`。

**移植映射**：**搬生成器，不搬 PNG**（PNG 可现生成）。改 2 处：输出路径、
（可选）改个 CLI 名。对世界盒的价值：地面参考网格给出空间尺度，比现在的纯色平面更容易判断「点放哪了」
（与 lead 反馈的「地形是单色平面、起伏看不出」是同一类问题）。

**工作量**：0.2 人日。**风险**：极低；许可见 §3（breeze 无仓库级 LICENSE）。

### I. `docs/pitfalls/` 13 篇

**移植映射**：**只作迁移期参考**，其中 6 篇对 trapla 直接有用：
`icu4x-ja-segmentation.md`、`bevy-019-api-changes.md`、`camera-rotation-mapping.md`、
`cursor-lock-key.md`、`cursor-platform-grab.md`、`toolchain-and-logging.md`。
**不要**整份复制进 trapla 文档体系（同一事实两个家，见 `doc-discipline`）；需要的结论已内联进本文。

---

## 2 三个特别问题

### ① trapla-viz 的「debug 构建下中文 Text 刷 ICU4X data error」——换 breeze 的字体解决不了

**结论：不能。** 这条输出是**文本分段模型**缺失，与用哪一档字体文件毫无关系；
breeze 的 `fonts.rs` + 思源宋体解决的是**另一个问题**（字形覆盖 / 不依赖系统字体）。

**证据链（实读，trapla 实际依赖树）**
1. parley **0.11.1**（bevy 0.20 的文本链，`trapla/Cargo.lock:4415-4417`）
   `src/analysis/mod.rs:46-55`：未开 `complex-scripts` feature 时，`word_segmenter()` 用
   `WordSegmenter::new_for_non_complex_scripts`，**不加载中日词典**；
   `:408` 对整段文本调 `segment_str`。
2. icu_segmenter **2.3.0**（`trapla/Cargo.lock:3240-3242`）
   `src/complex/mod.rs:183-209`：`select(ChineseOrJapanese)` 在 `ja` 槽为空时
   `ERR.with_display_context("Chinese/Japanese")` → 正是 trapla-viz README 记的那条；
   `src/word.rs:860-878` 显示只有 `load_dictionary()` 才会 `with_japanese_dictionary()`
   载入 `cjdict`（数据已在 `icu_segmenter_data-2.3.0/data/segmenter_dictionary_auto_v1.rs.data`，5.2 MB）。
3. icu_provider **2.3.1**（`trapla/Cargo.lock:3223-3225`）
   `src/lib.rs:195-215`：`logging` feature 关闭且 `debug_assertions` 打开时，
   `crate::log` 被别名为 `std::eprintln!`；**关闭后是空宏**。
   这解释了为什么它绕过 Bevy tracing / `RUST_LOG`、且只在 debug 出现。

**breeze 侧的交叉印证** [实读]：breeze 自己**加载了思源宋体**，仍在为这条噪音做规避——
`sandbox/plugin.rs:134-137` 用 0.5s 节流压刷新频率、`mechanics/interact.rs:196-197` 只在目标变化时写 `Text`；
其 `docs/pitfalls/icu4x-ja-segmentation.md:16-19` 的结论也是「推荐 release 跑」。

**怎么解（trapla-viz 侧，按优先级）**
1. **首选：给 bevy 打开 `complex_script_segmentation` feature。** 实读证据链：
   `bevy-0.20.0/Cargo.toml:2859` `complex_script_segmentation = ["bevy_internal/complex_script_segmentation"]`
   → `bevy_internal-0.20.0/Cargo.toml:221` → `bevy_text-0.20.0/Cargo.toml:38`
   `complex_script_segmentation = ["parley/complex-scripts"]`；开了之后 parley 走
   `WordSegmenter::new_dictionary`（`analysis/mod.rs:47-50`）→ 载入 cjdict → 不再产生该 `DataError`。
   该 feature **不在 bevy 默认 features**（`bevy-0.20.0/Cargo.toml:2867-2872` 只有 2d/3d/ui/audio）。
   代价：把中日词典编进二进制（编译体积/时间增加，具体数值 [未核实]）。
   **未跑构建实测** [推断] —— 源码链完整，但没有本轮的编译/运行验证。
2. 仍可用 **release** 构建规避（0.19 时代 breeze 的做法）：`debug_assertions` 关闭后 shim 变空宏。
   代价是编译与迭代变慢。
3. **`--screenshot` 的验收不受这条噪音影响**：判据是退出码 + PNG 魔数 + 字节数
   （`trapla-viz/src/main.rs:151-160`），stderr 有噪音不改变判据。

**fonts.rs 是否值得搬**：**值得，但理由要写对**。
- 它**不解决**告警（本节结论）。
- 它解决的是：截图里的中文**不再依赖目标机器装了哪个 CJK 字体**（现在靠
  `FontSource::Family("Noto Sans CJK SC")` + `system_font_discovery` feature，
  `trapla-viz/Cargo.toml:13`、`viz.rs:102-103`）；缺字体时会退回通用 sans-serif 并可能出豆腐块。
  这是「截图产物跨机器一致」的诉求，与告警是两件事。
- 若只想要告警消失，**一行 Cargo feature 比搬 23 MB 字体划算得多**。

### ② `GamePhase` + egui 菜单骨架能否承载「选场景 / 选算法 / 步进回放 / 对比」

**结论：状态机能承载（直接搬）；egui 菜单现在承载不了（版本卡住）；而且四项需求里有三分之二
breeze 根本没有对应物。**

**能直接搬的部分（几乎零改动）**
- `GamePhase`（`ui.rs:24-32`）作为 `States` 枚举 + `app.init_state`（`managers/mod.rs:43`）+
  `run_if(in_state(…))` 门控 + `OnEnter` 出生系统。
- `MenuAction` 的「菜单写选择 → 进状态时消费」模式（`ui.rs:36-46` + `state.rs:121-127`）。
- bevy 0.20 的对应 API 全在 [实读]：`init_state` / `in_state` / `NextState` / `OnEnter`
  （`bevy_state-0.20.0/src/app.rs:40`、`condition.rs:103`、`state/resources.rs:181`、`state/transitions.rs:19`）。

**必须改的几处**
1. **egui 被版本卡住**：breeze 用 `bevy_egui 0.41.1`（bevy `^0.19`），trapla-viz 是 bevy 0.20。
   本机索引缓存里 `0.40.0`–`0.41.1` 全部要求 `^0.19`；是否有 0.20 版 [未核实]（本机 crates.io 不可达）。
   → 用 Bevy UI 复刻（trapla-viz 已有 `Node`/`Text` 菜单化基础），或等 bevy_egui。
   注意 breeze 记的 `CentralPanel` 坑（`ui.rs:68-70`）是 bevy_egui 0.40/egui 0.34 的问题，换版本后要重判。
2. **选项面不同**：`MenuSelection` 只有 `NewGame`/`Continue`（`ui.rs:40-46`）；
   目标要的是「场景 id / 算法 id / 起终点 / 步进 / 对比」，要换成配置型资源
   （如 `SandboxConfig { scenario_id, planner, start, goal }`）。
3. **菜单里每帧读盘**：`ui.rs:81` 每次绘制都 `save_file_exists()`。换成场景/算法清单后要缓存成资源，
   不要在 UI 闭包里做 IO。
4. **输入加载时机要挪进 App**：trapla-viz 现在在 `App::new` **之前**就把 map/path/metrics 读完
   （`main.rs:31-91`），而「选场景」发生在菜单里 → 加载要挪进 `OnEnter(Running)`。
   `--headless-selftest` 不建 App（`main.rs:94-148`），不受影响。
5. **「步进回放」和「对比」breeze 没有对应物**：breeze 只有 F5/F9 存读档（`state.rs`），没有回放。
   trapla-viz 的 `Playback`（`viz.rs:59-73`、`:366-380`）+ 单步键（`:343-350`）已覆盖「步进回放」；
   「算法对比」要新写（N 条路径并存 + 叠加/并排 + 指标对照），这是**新增功能**，不是移植。

**判据**：搬状态机后，`--headless-selftest` 的输出应当逐行不变（它不进 App）。

### ③ 用 `smooth-bevy-cameras` 替换 `orbit_camera.rs`：两条非交互验收会不会坏

**结论**
- `--headless-selftest`：**不会坏**。它根本不建 `App`（`trapla-viz/src/main.rs:94-148` 只用
  `scene::build_terrain_mesh` / `data::*` / `metrics::*`），相机模块只参与**编译**、不参与运行。
  唯一可能打坏它的是**编译失败**（vendored crate 是 0.19 时代的 Cargo.toml）。
- `--screenshot`：**运行路径上不该坏，但有 4 处必须显式处理，漏一个就会坏**。

**必须处理的 4 处**
1. **必须注册 `LookTransformPlugin`**（`look_transform.rs:12-18`）——把 `LookTransform` 写进
   `Transform` 的同步系统在它里面。trapla-viz 现在是自己的 `look_transform_system`
   （`orbit_camera.rs:241-247`）在 `(orbit_input, drive_look_transform, look_transform_system).chain()`
   里做的（`:184-191`）。漏注册 → 相机 `Transform` 永远停在 spawn 值，截图是错视角。
2. **初始位姿要等价构造**：vendored `OrbitCameraBundle::new(controller, eye, target, up)`
   （`orbit.rs:57-71`）会同时设 `Transform`，所以第 N 帧取景可以保持；
   但要沿用现在的算法（`viz.rs:253-274` 由 `scene.home = (yaw,pitch,dist,centroid)` 算 eye）。
   角度约定两边**一致**：vendored `LookAngles::unit_vector` 展开为 `(cos p·sin y, sin p, cos p·cos y)`
   （`look_angles.rs:103-108`），与 `orbit_camera.rs:147-153` 相同；trapla-viz 的 README 也声明
   分层与角度约定照该 crate（`trapla-viz/README.md:59-61`、`orbit_camera.rs:1-17`）。
3. **`assert_not_looking_up()` 会 panic**：vendored `control_system` 每帧从视线反解角度并断言
   视线不与 `Vec3::Y` 平行（`orbit.rs:223`、`look_angles.rs:62-70`）。
   当前 home pitch = 0.62 rad（`viz.rs:122`）安全；但 trapla-viz 的俯仰夹取是 `0.05..1.5`
   （`orbit_camera.rs:126-127`），vendored 是 `±(π/2−0.01)`（`look_angles.rs:47-51`）——
   **交互**模式下键盘能推到的极值不同，截图模式无输入不受影响。
4. **键位与文档三处同步**：vendored orbit 默认是 `Ctrl`+左键旋转 / 右键平移 / 滚轮缩放
   （`orbit.rs:123-179`），trapla-viz 现在是左键拖动 + `Q/E` + `Z/X` + `F` 复位
   （`orbit_camera.rs:193-229`）。改了就要同步 `cli.rs:12-27` 的 `CONTROLS_HELP`、
   `viz.rs:319-325` 的底部提示、`trapla-viz/README.md:38-39`——三处漏一处就是文档撒谎。

**净损失**：trapla-viz 的 7 个相机单测（`orbit_camera.rs:249-331`，含「正 pitch = 相机在目标上方」
「鼠标右移右视」两条钉住约定、防回归的用例）会随模块一起删；vendored crate 只有 4 个测试
（都在 `look_angles.rs:110-160`），**不覆盖** trapla-viz 那两条约定。
再加上该 crate 在 breeze 里已被声明「不再跟随上游」并改造过（`lib.rs:1-11`），
搬过来等于把维护面从一个有测试的本地模块换成一个无测试的 vendored 分支。
→ **不推荐现在替换**；若确实要换（例如想要它的 four-controller 与 CursorLock），
把 §4 步骤 5 排在最后，并先补上那两条约定的移植测试。

---

## 3 许可证逐项

| 对象 | 许可 | 落地要求 |
|---|---|---|
| `breeze/assets/fonts/SourceHanSerifSC-Regular.otf`（思源宋体 SC） | **SIL OFL 1.1**，Copyright 2017-2022 Adobe，保留字体名 `Source`（`breeze/assets/fonts/LICENSE.txt:1-9`） | 随软件捆绑/嵌入/再分发均允许。条件：① 字体本身不得单独出售；② 每份拷贝需附版权声明与许可全文（**`LICENSE.txt` 必须随 `.otf` 一起搬**）；③ 衍生版本不得使用保留名 `Source`（**做子集/改名后不得再叫 Source…**）；④ 不得用作者名做推广背书；⑤ 字体及衍生版本必须整体以 OFL 分发，不得换别的许可。用字体生成的文档不受此限。 |
| `breeze/crates/smooth-bevy-cameras/` | **MIT**，Copyright (c) 2021 bonsairobo（`Cargo.toml:8` + `LICENSE` 全文） | 搬运/改造后再分发需保留版权声明与许可文本（breeze 的改造版未改许可声明）。 |
| `breeze/crates/bevy_mod_inverse_kinematics/` | **MIT**，Copyright (c) 2022 Bram Buurlage（`LICENSE-MIT`） | 本次**不搬**，列出仅供排除。 |
| breeze 自有代码（`src/managers`、`ui.rs`、`sandbox/`、`state.rs`、`mechanics/`、`tools/`） | **未声明** —— breeze 仓根目录**没有 LICENSE / COPYING 文件**（实测） | 同作者内部移植无碍；但一旦要对外分发 trapla，缺许可声明就是问题。**建议 lead 在实施前决定**：给 trapla 加一份仓库级 LICENSE，或明确这批代码的许可。 |
| 生成的 `grid.png` | 生成物，与 breeze 代码同许可状态；本方案建议**不搬 PNG**、只搬生成器 | 同上一条。 |
| trapla 现有代码 | 仓根同样**无 LICENSE 文件**（实测） | 与上一条一并处理。 |

---

## 4 建议移植顺序

每一步都用 trapla-viz 已有的两条非交互验收之一判定「没坏」：

- **V1 = `--headless-selftest`**：退出码 0 + 打印块（`main.rs:102-146`）与改动前逐行一致
  （地图行×列、stride、顶点算式、`Mesh::count_vertices()`、路径点数与左右脚对照、目标点、指标）。
- **V2 = `--screenshot`**：退出码 0 + `verify_png` 通过（存在、≥8 字节、PNG 魔数，`main.rs:151-160`），
  并比对 PNG 字节数与改动前的基线（尺寸取窗口物理像素，`README.md:79`）。
  命令形如 `$BIN --map data/csv/map.csv --path data/csv/path_result.csv --screenshot /tmp/viz_N.png --frames 60`。

| 步 | 内容 | 怎么验证没坏 | 估时 |
|---|---|---|---|
| 0 | **`complex_script_segmentation` feature**（`trapla-viz/Cargo.toml:13` 加一个 feature 名） | 构建通过；V1 逐行不变；V2 出 PNG 且大小与基线一致；debug 跑 V2 时 stderr 不再出现 `No segmentation model` | 0.1 人日 |
| 1 | **字体资产化**：搬 `fonts.rs` 模式 + 思源宋体，`FontSource::Family` → `FontSource::Handle` | V1 不变（不建 App，与字体无关）；V2 目视中文仍清晰、非豆腐块；再在一台**没装 Noto Sans CJK SC** 的环境复跑 V2 对照 | 0.5 人日 |
| 2 | **`GamePhase` 状态机 + Bevy UI 菜单骨架**（不引 egui）；加载逻辑挪进 `OnEnter`；截图路径加开关跳过菜单 | V1 逐行不变（这是最关键的一条）；V2 在「跳过菜单」下 PNG 与基线一致；交互模式能进/出菜单 | 0.5–1 人日 |
| 3 | **选场景**：接 `trapla-lab` 的场景库（`crates/trapla-lab/src/scenario.rs:208` `builtin` / `:318` `select` / `:338` `write_map_csv`） | V1 打印的「地图 行×列 / stride / 顶点数」与所选场景 `write_map_csv` 产物一致（同一判据已有）；V2 看出地形换了 | 1 人日 |
| 4 | **选算法 + 对比**：读 `trapla-lab` 的 `Planner` 清单（`planner.rs:99-108`、`:237` `builtin`），展示 N 条路径 | V1 打印的路径点数/左右脚计数与所选 `path_result.csv` 一致；V2 出 PNG；对比面板数字与 `--metrics` JSON 对得上 | 2–3 人日 |
| 5 | **约束可视化 / 放起终点**：`mechanics/interact.rs` 概念重写（无 rapier，解析反解高度场）+ 越界点标红 | V2 目视越界点标红；数字与 `trapla-lab` 独立校验器输出一致 | 2 人日 |
| 6 | （可选、最后）**相机替换与 HUD 改造**：见 §2 问题 ③ 的 4 条前置 | V1 不变；V2 PNG 与基线一致（同取景）；补两条约定测试后才允许替换 | 1–2 人日 |

**最先看到效果的一步 = 步骤 3**（换场景 → 地形一眼不同，直接把「单色平面」变成可辨认的地形）。
步骤 0 是「最快见效」的一行改动（终端立刻干净），但它不产生世界盒能力。
步骤 1 与 2 都是**前置白工**，价值在于让后面每一步的截图判据稳定、可跨机器复现。

**验证纪律**
- 每步动手前先存一份基线（V1 输出 + V2 的 PNG 与其字节数），改完按上表比对；不要用「看着差不多」代替。
- 步骤 6 若替换相机，必须**先**把 `orbit_camera.rs:249-331` 里那两条约定（正 pitch 在上、鼠标右移右视）
  改成对新控制器的测试，再删旧模块——否则那两条约定就没有强制点了。

---

## 5 未核实清单

| 项 | 状态 |
|---|---|
| vendored crate 在 bevy 0.20 下**实际编译通过** | 未核实（本轮不跑 `cargo build`）。符号已逐条在 0.20 源码核对存在，签名兼容性未验。最低成本验证：单独建一个只依赖该 crate + bevy 0.20 的最小 workspace 跑 `cargo check`。 |
| `complex_script_segmentation` 打开后告警**实际消失**、以及编译体积增量 | 未核实（源码链完整：`bevy_text` → `parley/complex-scripts` → `WordSegmenter::new_dictionary` → 载入 `cjdict`；无编译/运行验证）。 |
| bevy_egui 是否存在与 bevy 0.20 兼容的版本 | 未核实（本机 crates.io 不可达；索引缓存最新为 0.41.1，要求 `^0.19`）。 |
| 字体子集化后的体积与工具链 | 未评估。 |
| 各步骤工作量估算 | 推断（按「改动面 + 验证面」粗估，非实测工时）。 |
| breeze `tools/` 生成器在 Python 3.14 + `uv` 下的可运行性 | 未跑（只读了源码与入口注册）。 |
