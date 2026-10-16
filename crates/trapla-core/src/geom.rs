//! 二维 / 三维几何基元，以及平面拟合。
//!
//! ## 底座：glam（task-8 修正案）
//!
//! - 本仓的数值全部来自 C++ 的 `double`（原仓没有一处 `float`），所以**只用 glam 的 f64 变体**：
//!   [`glam::DVec2`] / [`glam::DVec3`] / [`glam::DMat2`]。不带 `D` 前缀的同名变体是 f32，
//!   一律禁止 —— 精度掉到 1e-7 会让与 C++ 基线的 1e-9 golden 容差直接失效
//!   （机械判据：本文件里不得出现裸的 f32 变体名，即 f32 变体名必须带 `D` 前缀才算合法）。
//! - 选 glam 而不是 nalgebra：viz（Bevy）与 glam 同源，可视化侧可直接复用这些类型。
//!   但**禁止**在 `trapla-core` 里引入 `bevy` —— viz 是独立 crate。
//! - 库只提供**原语**（向量、点积、叉积、归一化、矩阵）；算法保持显式代码。
//!
//! ## 不许换库的算法（必须与 C++ 逐位对齐，见 task-8）
//!
//! 1. [`fit_plane_up`]（原 `Ground::trip`：`<3` 退化、`=3` 定面、`>3` 用前三点定面再逐个 `up_fit`）；
//! 2. [`Plane::normal_angle`]（原 `atan2(sqrt(a²+b²), |c|)`，实测 `src/utils/geometry.cpp:1160-1167`）；
//! 3. [`index_scale`]（`ceil(v * scale)`，原 `src/utils/scale.cpp:12`）；
//! 4. `Grid` 的均值池化缩放（`grid.rs`，原 `src/utils/geometry.cpp:657-688`）；
//! 5. `Terrain::steep_extend` 的加权式（原 `src/aStar/aStar.cpp:205`）。
//!
//! 这些必须保持与原实现**相同的运算顺序**，库只能在其中充当原语。另有三条禁令：不许用
//! `ndarray` / `nalgebra` 替换 `Grid`（它是私有 `Vec<f64>` + 访问器，换 array 库会引入新的
//! 行/列主序混淆面）；不许用 `petgraph` 替换 A*（`steep_extend` 剪枝、代价含目标格高度、
//! 邻居洗牌这些自定义语义它覆盖不了）；`parry2d` 不采纳，线段相交谓词按原实现显式写
//! （`segments_intersect`，`src/utils/geometry.cpp:1242-1259`）。
//!
//! ## 与原 C++ 的缩放语义
//!
//! 原 `SqDot` 有两个同名、语义不同的 `scale()`（`geometry.cpp:116` const 版返回浮点、
//! `geometry.cpp:238` 非 const 版返回 `ceil` 整数），而且在同一函数里混用（`geometry.cpp:919-929`）。
//! 本重写里缩放只有两个明确入口，名字不同、语义各一：
//!
//! - [`index_scale`]：`ceil(v * scale)` → 缩放索引（原 `scale.cpp:12` 与两个 `scale()` 的整数版）；
//! - [`Idx::central_restore`]：`v / scale + 0.5 / scale` → 区块中心的**浮点**原图坐标
//!   （原 `SqDot::central_restore`，`geometry.cpp:150-153`）。
//!
//! 邻域方向只有一份表：[`Dir4`]（原仓在 `geometry.cpp:264,442,449` 与 `index.cpp:54` 有 4 份拷贝，
//! 且方向含义只靠位置约定）。

use crate::error::{CoreError, Result};

/// 栅格点（单位：格）—— `glam::DVec2` 的 newtype，保留原语义命名与访问器。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Dot(pub glam::DVec2);

impl Dot {
    pub const fn new(x: f64, y: f64) -> Self {
        Self(glam::DVec2::new(x, y))
    }

    pub const fn x(self) -> f64 {
        self.0.x
    }

    pub const fn y(self) -> f64 {
        self.0.y
    }

    /// 与另一点的欧氏距离：`sqrt((dx)² + (dy)²)`（原 `SqDot::distance`，`geometry.cpp:336-338`）。
    pub fn distance(self, o: Dot) -> f64 {
        let dx = self.x() - o.x();
        let dy = self.y() - o.y();
        (dx * dx + dy * dy).sqrt()
    }

    /// 指向另一点的方位角（弧度）= `atan2(o.y - self.y, o.x - self.x)`
    /// （原 `SqDot::angle`，`geometry.cpp:340-342`；`SqDot::line_angle` 是同一式子，已合并到这里）。
    ///
    /// 取值域 `(-PI, PI]`：`-PI` 取不到（`atan2(-0.0, 负数)` 之外都在 `(-PI, PI]`）。
    /// 两点重合（含两侧都为 0 的退化向量）时 `atan2(0.0, 0.0) == 0.0`。
    pub fn angle(self, o: Dot) -> f64 {
        (o.y() - self.y()).atan2(o.x() - self.x())
    }

    /// 取整为栅格索引：**四舍五入**（`std::round`，半值远离零），不是截断、也不是 `ceil`。
    ///
    /// 原 `SqDot::x_index()/y_index()`（`geometry.cpp:97-108`）是
    /// `static_cast<int>(std::round(v))`，`SqDot::as_index()`（`geometry.cpp:354-356`）把两个分量
    /// 各取整后装进 `Intex`。注意：`:238` 的非 const `SqDot::scale(double)` 用的是 `ceil`，
    /// 两者在右边界格（如 0.5）与负坐标（如 -0.5）上结果不同 —— 本重写只保留这里这一个语义。
    pub fn as_idx(self) -> Idx {
        Idx::new(self.x().round() as i32, self.y().round() as i32)
    }
}

/// 整数栅格索引（保持普通 struct，不裹 glam）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Idx {
    pub x: i32,
    pub y: i32,
}

impl Idx {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// 索引 → 点（`as f64`，不缩放；原 `Intex::as_dot`，`index.cpp:13-15`）。
    pub fn as_dot(self) -> Dot {
        Dot::new(self.x as f64, self.y as f64)
    }

    /// 缩放图上第 `self` 个区块的中心在**原图**上的浮点坐标：
    /// `(x / scale + 0.5 / scale, y / scale + 0.5 / scale)`（即 `(x + 0.5) / scale`）。
    ///
    /// 公式取自 `Intex::central_restore`（`index.cpp:108-111`）与 `SqDot::central_restore`
    /// （`geometry.cpp:150-153`）**同一处**收敛而来，但**不截断**：原 `Intex` 版本因为返回 `Intex`
    /// 把结果截断向零（`central_restore.csv` 的 `int` 行），`SqDot` 版本保留浮点（`dot` 行）。
    /// 本重写取**无损失的浮点**语义，定义域是「任意索引、无栅格上下文」。
    ///
    /// 落回栅格（截断向零 + 钳位到本栅格边界）由 [`crate::grid::Grid::restore_dot`] 负责 ——
    /// 后者**委托**本函数，公式只有这一处。
    pub fn central_restore(self, scale: f64) -> Dot {
        let offset = 0.5 / scale;
        Dot::new(
            self.x as f64 / scale + offset,
            self.y as f64 / scale + offset,
        )
    }
}

/// 四邻域方向。索引含义在这里有名字，不再靠调用点的位置约定
/// （原仓 `geometry.cpp:264,442,449` 与 `index.cpp:54` 是 4 份只有位置约定的拷贝）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Dir4 {
    /// `dx = -1, dy = 0`
    Left = 0,
    /// `dx = 1, dy = 0`
    Right = 1,
    /// `dx = 0, dy = -1`
    Down = 2,
    /// `dx = 0, dy = 1`
    Up = 3,
}

impl Dir4 {
    /// 按原仓 `dx = {-1,1,0,0}` / `dy = {0,0,-1,1}` 的顺序（`geometry.cpp:264-266`）。
    pub const ALL: [Dir4; 4] = [Dir4::Left, Dir4::Right, Dir4::Down, Dir4::Up];

    /// 该方向的索引增量 —— **全仓唯一的四邻域偏移表**。
    pub const fn delta(self) -> (i32, i32) {
        match self {
            Dir4::Left => (-1, 0),
            Dir4::Right => (1, 0),
            Dir4::Down => (0, -1),
            Dir4::Up => (0, 1),
        }
    }
}

/// 线段。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Line {
    pub p0: Dot,
    pub p1: Dot,
}

impl Line {
    /// 过点 `p`、方向角 `rz` 的直线（取 `p0 = p`，`p1 = p + (cos rz, sin rz)`）。
    ///
    /// 与 `SqLine(const SqDot& point, double angle)`（`geometry.cpp:373-374`）同一条直线：
    /// 该构造函数取 `a = sin(rz)`、`b = -cos(rz)`、`c = cos(rz)*p.y - sin(rz)*p.x`，
    /// 正是两点式 `(p, p + (cos, sin))` 的系数乘 `-1`，距离相同。
    pub fn from_point_angle(p: Dot, rz: f64) -> Self {
        Self {
            p0: p,
            p1: Dot::new(p.x() + rz.cos(), p.y() + rz.sin()),
        }
    }

    /// 点到线的距离 —— 是**到无限长直线**的距离，不是到线段的距离
    /// （原 `SqLine::distance`，`geometry.cpp:375-377`：`|a*x + b*y + c| / sqrt(a² + b²)`，
    /// 系数取自两点式构造 `geometry.cpp:371`；线段外的点仍按所在直线算）。
    pub fn dist_to_point(self, p: Dot) -> f64 {
        let a = self.p0.y() - self.p1.y();
        let b = self.p1.x() - self.p0.x();
        let c = self.p0.x() * self.p1.y() - self.p1.x() * self.p0.y();
        (a * p.x() + b * p.y() + c).abs() / (a * a + b * b).sqrt()
    }

    /// 两线段是否相交 —— 原 `segments_intersect`（`geometry.cpp:1242-1259`）逐行移植。
    ///
    /// 边界语义（实测自源码，非推断）：先做包围盒快速排斥，再做跨立实验，判据是
    /// `cross1*cross2 <= 0 && cross3*cross4 <= 0`，`<= 0` 使**端点相触算相交**；
    /// **共线重叠**（包围盒相交、四个 cross 全为 0）也算相交；
    /// **共线但不重叠**被包围盒那一关拒掉。端点退化成点（`p0 == p1`）时，
    /// 一对 cross 恒等，结果恰好等价于「该点落在线段 2 上」。
    pub fn intersects(self, o: Line) -> bool {
        if self.p0.x().min(self.p1.x()) > o.p0.x().max(o.p1.x())
            || o.p0.x().min(o.p1.x()) > self.p0.x().max(self.p1.x())
            || self.p0.y().min(self.p1.y()) > o.p0.y().max(o.p1.y())
            || o.p0.y().min(o.p1.y()) > self.p0.y().max(self.p1.y())
        {
            return false;
        }

        let cross1 = (self.p1.x() - self.p0.x()) * (o.p0.y() - self.p0.y())
            - (o.p0.x() - self.p0.x()) * (self.p1.y() - self.p0.y());
        let cross2 = (self.p1.x() - self.p0.x()) * (o.p1.y() - self.p0.y())
            - (o.p1.x() - self.p0.x()) * (self.p1.y() - self.p0.y());
        let cross3 = (o.p1.x() - o.p0.x()) * (self.p0.y() - o.p0.y())
            - (self.p0.x() - o.p0.x()) * (o.p1.y() - o.p0.y());
        let cross4 = (o.p1.x() - o.p0.x()) * (self.p1.y() - o.p0.y())
            - (self.p1.x() - o.p0.x()) * (o.p1.y() - o.p0.y());

        cross1 * cross2 <= 0.0 && cross3 * cross4 <= 0.0
    }
}

/// 点在平面哪一侧（原 `CuPos`，`geometry.hpp:48`；判据 `geometry.cpp:1113-1125`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Above,
    Below,
    Inside,
}

/// 平面 `a*x + b*y + c*z + d = 0`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plane {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl Plane {
    fn value_at(self, p: Point3) -> f64 {
        self.a * p.x() + self.b * p.y() + self.c * p.z() + self.d
    }

    /// 原 `CuPlane::get_pos`（`geometry.cpp:1113-1125`）：`|value| <= 1e-9` 记 `Inside`。
    fn side(self, p: Point3) -> Side {
        let v = self.value_at(p);
        if v > 1e-9 {
            Side::Above
        } else if v < -1e-9 {
            Side::Below
        } else {
            Side::Inside
        }
    }

    /// 平面法向与竖直方向（+z）的夹角（弧度）。
    ///
    /// 实测（`src/utils/geometry.cpp:1160-1167`）：`normal_angle = atan2(sqrt(a²+b²), |c|)`；
    /// 法向模为 0 时原实现返回 `0.0`。与 `acos(|c| / ||n||)` 等价但数值更稳 —— **照原式子写**，
    /// 不改用库的归一化路径重写（运算顺序要对齐 golden）。
    pub fn normal_angle(self) -> f64 {
        let magnitude = (self.a * self.a + self.b * self.b + self.c * self.c).sqrt();
        if magnitude == 0.0 {
            return 0.0;
        }
        let bottom = (self.a * self.a + self.b * self.b).sqrt();
        bottom.atan2(self.c.abs())
    }

    /// 求平面在 `(x, y)` 处的高度 `z = -(a*x + b*y + d) / c`。
    ///
    /// **保留原行为**：`c == 0`（竖直平面）时除零，产生非有限值（±inf / NaN）。
    /// 调用方负责先判断 `c != 0`；本函数不许偷偷返回 0 或加兜底。
    pub fn z_at(self, x: f64, y: f64) -> f64 {
        -(self.a * x + self.b * y + self.d) / self.c
    }
}

/// 三维点 —— `glam::DVec3` 的 newtype（拟合平面用）。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point3(pub glam::DVec3);

impl Point3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(glam::DVec3::new(x, y, z))
    }

    pub const fn x(self) -> f64 {
        self.0.x
    }

    pub const fn y(self) -> f64 {
        self.0.y
    }

    pub const fn z(self) -> f64 {
        self.0.z
    }
}

/// 三点定面；三点共线 / 重合（法向全零）时返回 `None`（原 `CuPlane::define_plane` 返回 `false`
/// 的判据，`geometry.cpp:1081-1098`）。
fn plane_of(three: &[Point3; 3]) -> Option<Plane> {
    let v1 = three[1].0 - three[0].0;
    let v2 = three[2].0 - three[0].0;
    let n = v1.cross(v2);
    if n.x == 0.0 && n.y == 0.0 && n.z == 0.0 {
        return None;
    }
    let d = -(n.x * three[0].x() + n.y * three[0].y() + n.z * three[0].z());
    Some(Plane {
        a: n.x,
        b: n.y,
        c: n.z,
        d,
    })
}

/// 把浮点坐标按比例换成整数索引：`ceil(v * scale)`。
///
/// 与原 C++ `src/utils/scale.cpp:12` 的 `index_scale(double, double)` 逐字一致
/// （实测：`return static_cast<int>(std::ceil(index * scale));`）。
/// 这是本重写里**唯一**的「浮点 → 缩放索引」入口（原 `SqDot::scale` 双重载已消灭）：
/// 索引侧用本函数，浮点侧用 [`Idx::central_restore`]，两个名字两套语义，不再是同名重载。
///
/// 边界（`index_scale.csv` 实测）：`0 * scale == 0`；负数的 `ceil` 向零靠
/// （`(-1) * 0.025 = -0.025 → ceil = -0.0 → 0`，`(-1) * 1.0 = -1`）；
/// 乘积恰为整数时结果就是该整数（`40 * 0.025 → 1`），不会再多加一。
pub fn index_scale(v: f64, scale: f64) -> i32 {
    (v * scale).ceil() as i32
}

/// 单步 `up_fit`（原 `up_fit`，`geometry.cpp:1169-1199`）：保证 `dot` 不落在返回的平面**上方**。
///
/// 迭代细节（照源码）：
/// 1. 用当前三点定面；若 `dot` 不在面上方（`Above`），直接返回该面，且**不改动**三点；
/// 2. 否则依次尝试把第 `i` 个点（`i = 0,1,2`）换成 `dot`，替换后能定面且 `dot` 不在新面上方，
///    就返回新面并**保留**这次替换；
/// 3. 三次替换都不成 —— 用 `dot` 覆盖第 0 个点后定面返回。
///
/// 相对 C++ 的**确定化**：C++ 在候选三点退化时仍会对未初始化的 `A/B/C/D` 求值（UB），
/// 这里把「退化候选」当作不可用而跳过；当前三点本身退化时视为「无约束」（任何点都算在上方），
/// 于是循环会尝试用后续点换出一个能定面的三点组。最终仍无法定面时返回 `None`。
fn up_fit_one(dot: Point3, three: &mut [Point3; 3]) -> Option<Plane> {
    match plane_of(three) {
        Some(plane) if plane.side(dot) != Side::Above => return Some(plane),
        _ => {}
    }

    for i in 0..3 {
        let saved = three[i];
        three[i] = dot;
        let candidate = plane_of(three);
        if candidate.is_some_and(|plane| plane.side(dot) != Side::Above) {
            return candidate;
        }
        three[i] = saved;
    }

    three[0] = dot;
    plane_of(three)
}

/// 拟合一个「不高于所有样本点」的平面（原 `Ground::trip`，`src/ground/ground.cpp:61-90`）。
///
/// 分支（与原实现对齐）：
/// - `points.len() < 3` ⇒ `Err(DegeneratePlane)`；
/// - `== 3` ⇒ 三点定面；三点共线 / 重合 ⇒ `Err(DegeneratePlane)`；
/// - `> 3` ⇒ 前三点定初始面（共线时进入「无约束」状态），再对**每个**点做 [`up_fit_one`] 迭代；
///   迭代后仍无法定面（例如所有点共线）⇒ `Err(DegeneratePlane)`。
///
/// 与 C++ 的三处**确定化**（都发生在原实现的 UB 处）：
/// 1. `== 3` 且三点共线：C++ 的 `CuPlane(const std::array<CuDot,3>&)` 在 `define_plane` 返回
///    `false` 时留下未初始化的 `A/B/C/D`（`geometry.cpp:1060-1062,1088-1090`），读它们是 UB ——
///    `normal_angle.csv` 的 case 3/4/5 就是这份 UB 的产物（三个分量恒为 `3,3,3,-5`，且不构成
///    任何过这三点的平面）。这里明确返回 `Err(DegeneratePlane)`（`CoreError::DegeneratePlane`
///    的文档本就写着「三点共线」）。
/// 2. `> 3` 且初始三点共线：C++ 带着 UB 面继续迭代，输出取决于栈上残留值；这里用
///    「无约束 + 跳过退化候选」，结果仍满足 up-fit 后置条件（没有任何点在面上方），
///    `trip.csv` 的 case 1/21 因此与原值一致，case 26 原值是 UB 垃圾。
/// 3. 候选三点退化：跳过，不再对偶然的栈残留做判定。
///
/// 定义域边界：本函数只接受已经采样好的三维点。原 `Ground::trip` 在采样点越出栅格时返回
/// 默认 `CuPlane`（`ground.cpp:64-66`），对应的越界检查属于 [`crate::terrain::Terrain::plane_at`]
/// 的采样环节（应返回 `CoreError::OutOfBounds`），不在本函数的定义域内。
pub fn fit_plane_up(points: &[Point3]) -> Result<Plane> {
    if points.len() < 3 {
        return Err(CoreError::DegeneratePlane);
    }

    let mut three = [points[0], points[1], points[2]];
    if points.len() == 3 {
        return plane_of(&three).ok_or(CoreError::DegeneratePlane);
    }

    // 原实现在循环前构造的初始面只被第一次 `up_fit` 覆盖，没有别的用处；这里直接进循环。
    let mut fitted = None;
    for &point in points {
        fitted = up_fit_one(point, &mut three);
    }
    fitted.ok_or(CoreError::DegeneratePlane)
}
