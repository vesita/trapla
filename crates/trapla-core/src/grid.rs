//! 栅格数据：单元格存取、邻域、代价与降采样。
//!
//! 障碍用 [`OBSTACLE`]（`f64::INFINITY`）表示，与原 C++ `SqPlane` 的
//! `std::numeric_limits<double>::infinity()` 一致（`src/utils/geometry.cpp:663,812`）。
//! 注意原仓**两种障碍约定并存**：`SqPlane` 用 `+inf`，`Ground::obstacle` 用 `< 0`
//! （`src/ground/ground.cpp:282-287`，其格子被写成 `-1.0`）。本模块只实现 `+inf` 这一种，
//! `-1.0` 是普通高度值；需要负值障碍语义的调用方自行判断。
//!
//! 边界判断只有一处：[`Grid::in_bounds`]。原仓把同一判断写了三遍
//! （`Ground::is_valid(SqDot)`、`Ground::is_valid(int,int)`、`Ground::obstacle`，
//! `src/ground/ground.cpp:274-287`），本模块所有越界判定都走 `in_bounds`。

use crate::error::{CoreError, Result};
use crate::geom::{Dir4, Dot, Idx, index_scale};

/// 障碍格的哨兵值。
pub const OBSTACLE: f64 = f64::INFINITY;

/// 行优先栅格：`cells[i * cols + j]`，注意 `Idx { x, y }` 里 `x` 是行、`y` 是列
/// （沿用原 C++ `map[x][y]` 的约定）。
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    cells: Vec<f64>,
    rows: usize,
    cols: usize,
}

impl Grid {
    /// 用行优先数据构造；`cells.len() != rows * cols` 时返回
    /// [`CoreError::GridShapeMismatch`]（`rows * cols` 溢出时同样按不匹配拒绝，不做 panic）。
    ///
    /// `0 x 0` 的空栅格可以通过（`0 == 0`）：它所有访问器都返回 `None` / 空，
    /// 但 [`Grid::scale_down`] 会拒绝它（退化输入）。
    pub fn new(cells: Vec<f64>, rows: usize, cols: usize) -> Result<Self> {
        if rows.checked_mul(cols) != Some(cells.len()) {
            return Err(CoreError::GridShapeMismatch {
                rows,
                cols,
                got: cells.len(),
            });
        }
        Ok(Self { cells, rows, cols })
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    /// 越界返回 `None`。
    pub fn cell(&self, i: Idx) -> Option<f64> {
        self.flat(i).map(|k| self.cells[k])
    }

    pub fn in_bounds(&self, i: Idx) -> bool {
        i.x >= 0 && i.y >= 0 && (i.x as usize) < self.rows && (i.y as usize) < self.cols
    }

    /// 越界或非障碍都返回 `false`；只有**逐位等于** [`OBSTACLE`] 才算障碍
    /// （NaN 不等于任何值，因此 NaN 格不是障碍）。
    pub fn is_obstacle(&self, i: Idx) -> bool {
        self.cell(i) == Some(OBSTACLE)
    }

    /// 四邻域，**顺序固定**：[`Dir4::ALL`]，即左、右、下、上
    /// （`dx = {-1, 1, 0, 0}`、`dy = {0, 0, -1, 1}`，`src/utils/geometry.cpp:263-266`）。
    ///
    /// 只返回 [`Grid::in_bounds`] 为真的邻居（原 `get_neighbour(x_ceil, y_ceil)` 的过滤版，
    /// `geometry.cpp:302-313`）；原仓没有任何调用点需要「未过滤的 4 个」版本
    /// （`aStar.cpp:29` 用有效邻居、`aStar.cpp:100` 用过滤后洗牌版）。
    /// 这里**不洗牌** —— 洗牌由 `rng::Rng` 显式完成，不隐藏在邻域函数里。
    pub fn neighbours4(&self, i: Idx) -> Vec<Idx> {
        Dir4::ALL
            .iter()
            .map(|dir| {
                let (dx, dy) = dir.delta();
                Idx::new(i.x + dx, i.y + dy)
            })
            .filter(|neighbour| self.in_bounds(*neighbour))
            .collect()
    }

    /// 从 `from` 走到相邻格 `to` 的代价：`|dx| + |dy| + 目标格高度`。
    ///
    /// 原 `SqPlane::cost`（`src/utils/geometry.cpp:889-895`）是
    /// `manhattan_distance(at, to) + map[to.x][to.y]` —— 曼哈顿距离**加上目标格的高度值**。
    /// 因此走到 [`OBSTACLE`] 格得到 `+inf`，调用方靠它把障碍排除掉。
    ///
    /// 原实现对越界 `to` 直接索引 `map[to.x][to.y]`（UB）；这里确定化为返回 `f64::INFINITY`
    /// （与障碍同一条路径，fail-closed）。
    pub fn cost(&self, from: Idx, to: Idx) -> f64 {
        let Some(height) = self.cell(to) else {
            return f64::INFINITY;
        };
        let dx = (from.x as i64 - to.x as i64).abs() as f64;
        let dy = (from.y as i64 - to.y as i64).abs() as f64;
        dx + dy + height
    }

    /// 均值池化降采样（原 `SqPlane::scale_graph`，`src/utils/geometry.cpp:657-688`）。
    ///
    /// 与源码逐行对齐的规则：
    /// - `scale <= 0` 或非有限 ⇒ [`CoreError::InvalidScale`]（原实现会除零 / UB）；
    /// - 输出尺寸 `ceil(rows * scale) x ceil(cols * scale)`（用 [`index_scale`]，不另立公式）；
    /// - 每个输出格的采样中心 = `trunc((i + 0.5) / scale)`，再钳到 `rows - 1` / `cols - 1`
    ///   （`geometry.cpp:669-674`）；
    /// - 窗口半边长 = `max(1, trunc(1.0 / scale))`（`geometry.cpp:679-680`）—— 注意 `scale > 1`
    ///   时截断成 0 再抬到 1，所以**放大也是 3x3 均值**；
    /// - 每格取窗口内**落在原图内**的所有格子的算术平均（原 `summary`，
    ///   `geometry.cpp:794-813`），窗口里没有任何格子时返回 `+inf`。
    ///
    /// **障碍的传播方式**（实测源码与 golden，不是二选一的猜测）：`summary` 对窗口内所有格子
    /// 直接求和，`+inf` 参与求和后平均值仍是 `+inf`，所以**含 [`OBSTACLE`] 的窗口整格记障碍**；
    /// 但它不是「先判障碍再整块赋值」，而是 `inf` 参与均值的自然结果 —— 纯有限值的窗口照常取均值。
    /// 空栅格（`rows == 0 || cols == 0`）⇒ [`CoreError::DegeneratePlane`]（原实现对 `map[0]` 取 UB）。
    pub fn scale_down(&self, scale: f64) -> Result<Grid> {
        if !scale.is_finite() || scale <= 0.0 {
            return Err(CoreError::InvalidScale(scale));
        }
        if self.rows == 0 || self.cols == 0 {
            return Err(CoreError::DegeneratePlane);
        }

        let new_rows = index_scale(self.rows as f64, scale).max(0) as usize;
        let new_cols = index_scale(self.cols as f64, scale).max(0) as usize;
        let side_length = ((1.0 / scale) as i32).max(1);

        let mut cells = Vec::with_capacity(new_rows * new_cols);
        for i in 0..new_rows {
            for j in 0..new_cols {
                let center_x = ((i as f64 + 0.5) / scale) as i32;
                let center_y = ((j as f64 + 0.5) / scale) as i32;
                let center_x = center_x.min(self.rows as i32 - 1);
                let center_y = center_y.min(self.cols as i32 - 1);
                cells.push(self.summary(center_x, center_y, side_length));
            }
        }

        Grid::new(cells, new_rows, new_cols)
    }

    /// 把缩放图上的区块 `block` 还原成原图上的两个角索引，返回 `(近端, 远端)`：
    /// `近端 = clamp(trunc(block.x / scale), trunc(block.y / scale))`、
    /// `远端 = clamp(trunc((block.x + 1) / scale), trunc((block.y + 1) / scale))`。
    ///
    /// 换算方向与 [`Grid::scale_down`] / [`Idx::central_restore`] **一致**：`scale` 是
    /// 「原始尺寸 × scale = 缩放尺寸」的那个比例，所以从缩放区块回到原图坐标要**除**
    /// （`scale_down` 的采样中心是 `(i + 0.5) / scale`，`central_restore` 是 `x / scale + 0.5 / scale`）。
    /// 截断向零，对齐原实现的 `static_cast<int>`（`src/utils/geometry.cpp:150-153` 同款转换）。
    ///
    /// 远端取 `(block + 1) / scale` 而不是减一：区块覆盖的是半开区间
    /// `[block/scale, (block+1)/scale)`，取两端角后由调用方（`Terrain::steep_extend`）
    /// 按闭区间 `min..=max` 扫描，所以相邻区块在边界格上有一格重叠。
    ///
    /// 钳位沿用原 `orth_near`（`src/utils/geometry.cpp:549-553`：`max(0, min(v, size - 1))`），未改。
    ///
    /// 与 [`Grid::restore_dot`] 的关系：两者现在**方向一致**（都从缩放坐标除回原图坐标），
    /// 区别只在落点 —— 本函数给区块的两个**角**，`restore_dot` 给区块**中心**落格后的一点。
    ///
    /// 原 C++ 的 `SqPlane::restore`（`src/utils/geometry.cpp:919-929`）走的是**乘**
    /// （`Intex::scale` → `index_scale` = `ceil(v * scale)`），与它自己的 `scale_graph` 方向相反：
    /// 每个缩放区块都被映射回地图左上角同一小块，`steep_extend` 因而恒不判死、高层地形约束空转。
    /// 这是原实现的缺陷，不是移植引入的；登记见 `doc/known_divergences.md` D4。
    pub fn restore_block(&self, block: Idx, scale: f64) -> (Idx, Idx) {
        let near = self.clamp(Idx::new(
            (block.x as f64 / scale) as i32,
            (block.y as f64 / scale) as i32,
        ));
        let far = self.clamp(Idx::new(
            ((block.x as f64 + 1.0) / scale) as i32,
            ((block.y as f64 + 1.0) / scale) as i32,
        ));
        (near, far)
    }

    /// 缩放图上区块 `block` 落到本栅格内的格：由 [`Idx::central_restore`] 算出区块中心的
    /// 原图坐标，再按 `static_cast<int>`（**截断向零**）+ `orth_near` 钳位取格
    /// （原 `SqPlane::restore_dot(Intex&, scale)`，`src/utils/geometry.cpp:908-910`，
    /// 内部正是 `orth_near(dot.central_restore(scale))`）。
    ///
    /// 与 [`Idx::central_restore`] 的分工：那个是**无栅格上下文、不截断**的浮点中心
    /// （定义域 = 任意索引），本函数是**本栅格内**的落格结果（定义域 = 缩放图上属于本图的区块）。
    /// 中心公式只有一份 —— 本函数**委托** `central_restore`，不重写公式。
    pub fn restore_dot(&self, block: Idx, scale: f64) -> Dot {
        let center = block.central_restore(scale);
        self.clamp(Idx::new(center.x() as i32, center.y() as i32))
            .as_dot()
    }

    /// `Idx` → 一维下标；越界返回 `None`。所有取格的唯一入口。
    fn flat(&self, i: Idx) -> Option<usize> {
        if self.in_bounds(i) {
            Some(i.x as usize * self.cols + i.y as usize)
        } else {
            None
        }
    }

    /// 原 `SqPlane::orth_near`（`geometry.cpp:543-553`）：`max(0, min(v, size - 1))`。
    fn clamp(&self, i: Idx) -> Idx {
        let max_x = (self.rows as i32 - 1).max(0);
        let max_y = (self.cols as i32 - 1).max(0);
        Idx::new(i.x.clamp(0, max_x), i.y.clamp(0, max_y))
    }

    /// 原 `SqPlane::summary`（`geometry.cpp:794-813`）：窗口内原图格子的算术平均，
    /// 一个都没有时返回 `+inf`。
    ///
    /// `side_length` 被钳到 `max(rows, cols)` —— 再大的窗口与它覆盖的格子集合相同，
    /// 只是空转（原实现允许 `int` 级别的巨大窗口）。
    fn summary(&self, center_x: i32, center_y: i32, side_length: i32) -> f64 {
        let side_length = side_length.min(self.rows.max(self.cols) as i32);
        let mut total = 0.0_f64;
        let mut count = 0.0_f64;
        for dx in -side_length..=side_length {
            for dy in -side_length..=side_length {
                let x = center_x + dx;
                let y = center_y + dy;
                if x >= 0 && y >= 0 && (x as usize) < self.rows && (y as usize) < self.cols {
                    total += self.cells[x as usize * self.cols + y as usize];
                    count += 1.0;
                }
            }
        }
        if count > 0.0 {
            total / count
        } else {
            f64::INFINITY
        }
    }
}
