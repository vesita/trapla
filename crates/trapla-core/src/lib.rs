//! trapla-core：双足机器人落足点规划的核心库。
//!
//! 本文件是 T1 冻结的模块边界，方法签名一律不许改；实现见 T2/T3/T4。
//!
//! - [`config`]：全部阈值的**唯一事实源**（原 C++ 把 40/2/10/75°/20° 散在多处）。
//! - [`constraints`]：硬约束的**唯一实现点**；姿态（候选 `next` vs 提交 `next_committed`）
//!   由调用方显式给出，模块自己从不推导。
//! - [`geom`]：点 / 索引 / 线段 / 平面与平面拟合（原 `geometry.cpp` 的两个同名
//!   不同语义 `scale()` 在本重写里被消灭：缩放只有 [`geom::index_scale`] 与 [`geom::Idx::central_restore`]）。
//! - [`grid`]：栅格数据、邻域、代价与降采样。
//! - [`terrain`]：栅格之上的地形查询（法向角、拟合平面、陡峭度）。
//! - [`foot`]：足部几何、覆盖格、可站立判定与滑动。
//! - [`astar`]：A*（曼哈顿启发）与缩放版搜索。
//! - [`rng`]：**确定性**随机源（可 seed、可 clone、跨平台稳定；原 C++ 用了 8 处
//!   `std::random_device`，整条链路不可复现）。
//! - [`robot`]：规划器主循环。
//! - [`metrics`]：猫步指标的载体（serde 序列化）。

pub mod astar;
pub mod config;
pub mod constraints;
pub mod error;
pub mod foot;
pub mod geom;
pub mod grid;
pub mod metrics;
pub mod rng;
pub mod robot;
pub mod terrain;
