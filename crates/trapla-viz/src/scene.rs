//! 场景几何：降采样、地形网格、地图坐标 -> 世界坐标、地形着色。
//!
//! ## 垂直尺度
//!
//! 水平方向 1 格 = 1 世界单位 = 1 cm；垂直方向由 `exaggeration` 决定「1 cm 高度 = 多少世界单位」：
//! `exaggeration = 1.0` 是**真尺度**（不再压缩），也是默认值。历史上默认是 `0.05`（压 20 倍），
//! 那会让 `map.csv` 的 96 cm 峰谷差只剩 4.8 世界单位、而水平跨度 3000 —— 几何上就是一张纸。
//!
//! ## 地形着色
//!
//! 顶点色 = **hypsometric 色带**（按数据实际 min/max 归一化）× **hillshade**。
//! hillshade 的坡度用**原始厘米高度**的相邻差分算（再乘一个显式的视觉效果放大系数），
//! 不是用被垂直尺度压过的世界几何法线 —— 否则起伏在法线里同样被压掉，算出来的光照明暗是平的。

use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;

use crate::data::{FootPath, Terrain};

/// 地形网格顶点数上限（默认降采样必须落在它以下）。
pub const MAX_VERTICES: usize = 100_000;

/// 默认垂直尺度：`1.0` = 真尺度（1 cm 高度 = 1 世界单位）。
pub const DEFAULT_EXAGGERATION: f32 = 1.0;

/// 垂直尺度允许范围（含端点）。
pub const EXAGGERATION_MIN: f32 = 0.01;
pub const EXAGGERATION_MAX: f32 = 200.0;

/// 路径 / 足标记离地高度（厘米，再按垂直尺度换算）。
pub const SURFACE_LIFT_CM: f32 = 6.0;

/// hillshade 的坡度放大系数：先用**原始厘米梯度**算法线，再乘它做视觉放大。
///
/// 这是显示参数，不是物理量；它不改变几何，只把「1 cm/格」量级的坡度换算成看得见的明暗。
const HILLSHADE_Z_FACTOR: f32 = 8.0;

/// hillshade 光源：方位角 315°（西北）、高度角 45°。
const HILLSHADE_AZIMUTH: f32 = 315.0_f32.to_radians();
const HILLSHADE_ALTITUDE: f32 = 45.0_f32.to_radians();

/// 阴面保留的基础亮度（纯黑会看不出地形，留一点环境光）。
const HILLSHADE_AMBIENT: f32 = 0.35;

/// 自动降采样步长：满足 `ceil(rows/s) * ceil(cols/s) <= MAX_VERTICES` 的最小 `s`。
///
/// 找不到时返回 `max(rows, cols)`（此时退化到单点）。
pub fn auto_stride(rows: usize, cols: usize) -> usize {
    if rows == 0 || cols == 0 {
        return 1;
    }
    for s in 1..=rows.max(cols) {
        let (r, c) = sample_shape(rows, cols, s);
        if r * c <= MAX_VERTICES {
            return s;
        }
    }
    rows.max(cols)
}

/// 采样后的网格形状 `(行数, 列数)`：每维每 `stride` 取一个点，上取整。
pub fn sample_shape(rows: usize, cols: usize, stride: usize) -> (usize, usize) {
    let s = stride.max(1);
    (rows.div_ceil(s), cols.div_ceil(s))
}

/// 厘米高度 -> 世界 Y。
pub fn height_to_world(height_cm: f32, exaggeration: f32) -> f32 {
    height_cm * exaggeration
}

/// 地形网格 + 它的形状元数据（自检摘要与网格构建共用同一份计算）。
pub struct TerrainMesh {
    /// 可直接挂到 `Mesh3d` 的网格。
    pub mesh: Mesh,
    /// 采样行数。
    pub n_rows: usize,
    /// 采样列数。
    pub n_cols: usize,
    /// 使用的降采样步长。
    pub stride: usize,
}

impl TerrainMesh {
    /// 顶点数（= `n_rows * n_cols`）。
    pub fn vertex_count(&self) -> usize {
        self.n_rows * self.n_cols
    }

    /// 三角形数（= `(n_rows-1) * (n_cols-1) * 2`）。
    pub fn triangle_count(&self) -> usize {
        self.n_rows.saturating_sub(1) * self.n_cols.saturating_sub(1) * 2
    }

    /// 顶点数算式，供报告与自检逐字复核。
    pub fn vertex_formula(&self, rows: usize, cols: usize) -> String {
        format!(
            "ceil({rows}/{s})={nr} x ceil({cols}/{s})={nc} = {v} <= {MAX_VERTICES}",
            s = self.stride,
            nr = self.n_rows,
            nc = self.n_cols,
            v = self.vertex_count(),
        )
    }
}

/// 显示网格的降采样口径。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Downsample {
    /// 窗口均值（默认）：每个采样点取它覆盖的 `stride × stride` 原始格的平均值。
    /// 最近邻会把原始高度图的高频混叠成地形上的一层斑点。
    WindowMean,
    /// 最近邻（改动前的口径）：只取采样点那一格。只用于负向对照与前后对比。
    Nearest,
}

/// 某原始格所属窗口的均值高度（厘米）；窗口按边界夹紧。
pub fn mean_height_at(terrain: &Terrain, stride: usize, row: usize, col: usize) -> f32 {
    let rows = terrain.rows();
    let cols = terrain.cols();
    if rows == 0 || cols == 0 {
        return 0.0;
    }
    let s = stride.max(1);
    let row0 = row.min(rows - 1) / s * s;
    let col0 = col.min(cols - 1) / s * s;
    let row1 = (row0 + s).min(rows);
    let col1 = (col0 + s).min(cols);
    let mut sum = 0.0_f64;
    let mut n = 0usize;
    for r in row0..row1 {
        for c in col0..col1 {
            sum += terrain.value(r, c).unwrap_or(0.0);
            n += 1;
        }
    }
    if n == 0 { 0.0 } else { (sum / n as f64) as f32 }
}

/// 显示网格：`rows × cols` 个采样高度（厘米）+ 采样口径。
///
/// 采样点的世界位置仍由**原始**行列号决定（`j * stride - (raw_cols-1)/2`），
/// 所以顶点/三角形数与采样口径无关 —— 换口径只改高度，不改网格规模。
pub struct DisplayGrid {
    /// 行优先高度（厘米）。
    pub heights: Vec<f32>,
    /// 采样行数。
    pub rows: usize,
    /// 采样列数。
    pub cols: usize,
    /// 降采样步长。
    pub stride: usize,
    /// 采样口径。
    pub mode: Downsample,
}

impl DisplayGrid {
    /// 采样格高度（厘米）；空网格返回 0。
    pub fn height(&self, row: usize, col: usize) -> f32 {
        if self.heights.is_empty() {
            return 0.0;
        }
        self.heights[row.min(self.rows - 1) * self.cols + col.min(self.cols - 1)]
    }

    /// `(row, col)` 落在哪个采样格。
    pub fn cell_of(&self, map_x: f64, map_y: f64) -> (usize, usize) {
        if self.rows == 0 || self.cols == 0 {
            return (0, 0);
        }
        let s = self.stride as f64;
        let i = ((map_x / s).floor().max(0.0) as usize).min(self.rows - 1);
        let j = ((map_y / s).floor().max(0.0) as usize).min(self.cols - 1);
        (i, j)
    }

    /// 地图坐标处的地面高度（厘米，米制 = 采样格值）。
    pub fn height_at_map(&self, map_x: f64, map_y: f64) -> f32 {
        let (i, j) = self.cell_of(map_x, map_y);
        self.height(i, j)
    }

    /// 采样高度的 `(min, max)`（厘米）。
    pub fn min_max(&self) -> (f32, f32) {
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for h in &self.heights {
            min = min.min(*h);
            max = max.max(*h);
        }
        if self.heights.is_empty() {
            (0.0, 0.0)
        } else {
            (min, max)
        }
    }

    /// 采样点在原始行 / 列轴上的世界坐标（与 `build_terrain_mesh` 的位置公式一致）。
    pub fn world_col(&self, col: usize, raw_cols: usize) -> f32 {
        (col * self.stride).min(raw_cols.saturating_sub(1)) as f32 - (raw_cols as f32 - 1.0) / 2.0
    }

    /// 采样点在原始行轴上的世界坐标。
    pub fn world_row(&self, row: usize, raw_rows: usize) -> f32 {
        (row * self.stride).min(raw_rows.saturating_sub(1)) as f32 - (raw_rows as f32 - 1.0) / 2.0
    }
}

/// 按 [`Downsample`] 口径建立显示网格。
pub fn display_grid(terrain: &Terrain, stride: usize, mode: Downsample) -> DisplayGrid {
    let stride = stride.max(1);
    let (rows, cols) = sample_shape(terrain.rows(), terrain.cols(), stride);
    let mut heights = Vec::with_capacity(rows * cols);
    for i in 0..rows {
        for j in 0..cols {
            let row = (i * stride).min(terrain.rows().saturating_sub(1));
            let col = (j * stride).min(terrain.cols().saturating_sub(1));
            let h = match mode {
                Downsample::WindowMean => mean_height_at(terrain, stride, row, col),
                Downsample::Nearest => terrain.value(row, col).unwrap_or(0.0) as f32,
            };
            heights.push(h);
        }
    }
    DisplayGrid {
        heights,
        rows,
        cols,
        stride,
        mode,
    }
}

/// 地图坐标 -> 世界坐标：X = 列、Z = 行、Y = 显示网格高度 × `exaggeration`。
///
/// 用**显示网格**（窗口均值）而不是原始最近邻：足 / 起点 / 目标必须落在真正渲染出来的
/// 那张地面上，否则在 6×6 窗口里会浮起或陷进去几个厘米。
pub fn world_from_map(
    terrain: &Terrain,
    grid: &DisplayGrid,
    map_x: f64,
    map_y: f64,
    exaggeration: f32,
) -> Vec3 {
    let half_cols = (terrain.cols() as f32 - 1.0) / 2.0;
    let half_rows = (terrain.rows() as f32 - 1.0) / 2.0;
    let h = grid.height_at_map(map_x, map_y);
    Vec3::new(
        map_y as f32 - half_cols,
        height_to_world(h, exaggeration),
        map_x as f32 - half_rows,
    )
}

/// 路径上每个点的世界坐标（抬离地面一点，避免与地形共面闪烁）。
pub fn path_world_points(
    terrain: &Terrain,
    grid: &DisplayGrid,
    path: &FootPath,
    exaggeration: f32,
) -> Vec<Vec3> {
    let lift = Vec3::Y * height_to_world(SURFACE_LIFT_CM, exaggeration);
    path.points()
        .iter()
        .map(|&(x, y)| world_from_map(terrain, grid, x, y, exaggeration) + lift)
        .collect()
}

/// 色带的低处拉伸指数：`t' = t^gamma`。地图数据里一半格子挤在 29–30 cm、
/// 高处只占末尾一小段，线性归一化会让视野里的地形全落在前缀绿色段 —— 用 <1 的
/// 指数把低处拉开，让「同一张图里的低处 / 高处」有可见的色相差别。
const BAND_GAMMA: f32 = 0.35;

/// hypsometric 色带（低 → 高）；`t` 是按数据 min/max 归一化后的高度。
///
/// 低端用冷色（深青 → 海绿）而不是纯绿：plateau 与缓坡都落在低端，
/// 冷 → 暖的过渡才能在同一个画面里区分开。
const BAND: [(f32, [f32; 3]); 8] = [
    (0.00, [0.08, 0.20, 0.30]),
    (0.12, [0.12, 0.34, 0.40]),
    (0.25, [0.22, 0.52, 0.35]),
    (0.40, [0.45, 0.66, 0.28]),
    (0.55, [0.78, 0.74, 0.35]),
    (0.70, [0.85, 0.68, 0.42]),
    (0.85, [0.70, 0.60, 0.48]),
    (1.00, [0.97, 0.97, 0.95]),
];

/// 高度 -> 色带颜色（`t = 0..1`，先按 [`BAND_GAMMA`] 拉伸，再线性插值 8 段）。
pub fn band_color(t: f32) -> [f32; 3] {
    let t = t.clamp(0.0, 1.0).powf(BAND_GAMMA);
    for pair in BAND.windows(2) {
        let (t0, c0) = pair[0];
        let (t1, c1) = pair[1];
        if t <= t1 {
            let k = if (t1 - t0).abs() < f32::EPSILON {
                0.0
            } else {
                (t - t0) / (t1 - t0)
            };
            return [
                c0[0] + (c1[0] - c0[0]) * k,
                c0[1] + (c1[1] - c0[1]) * k,
                c0[2] + (c1[2] - c0[2]) * k,
            ];
        }
    }
    BAND[BAND.len() - 1].1
}

/// 色带 -> 线性 RGB（网格顶点色要求线性空间）。
fn band_linear(t: f32, brightness: f32) -> [f32; 4] {
    let base = band_color(t);
    let c = Color::srgb(
        base[0] * brightness,
        base[1] * brightness,
        base[2] * brightness,
    );
    let l = c.to_linear();
    [l.red, l.green, l.blue, 1.0]
}

/// 显示网格某格的**着色法线**：坡度用厘米高度的中心差分（水平间距 = `stride` 厘米），
/// 再乘 [`HILLSHADE_Z_FACTOR`] 做视觉放大。hillshade 与斑点指标都用它。
fn grid_normal(grid: &DisplayGrid, i: usize, j: usize) -> Vec3 {
    let spacing = grid.stride as f32;
    let up = grid.height(i.saturating_sub(1), j);
    let down = grid.height((i + 1).min(grid.rows - 1), j);
    let left = grid.height(i, j.saturating_sub(1));
    let right = grid.height(i, (j + 1).min(grid.cols - 1));
    let gx = (right - left) / (2.0 * spacing) * HILLSHADE_Z_FACTOR;
    let gy = (down - up) / (2.0 * spacing) * HILLSHADE_Z_FACTOR;
    Vec3::new(-gx, -gy, 1.0).normalize_or_zero()
}

/// 光照方向（与 [`HILLSHADE_AZIMUTH`] / [`HILLSHADE_ALTITUDE`] 同源）。
fn light_direction() -> Vec3 {
    Vec3::new(
        HILLSHADE_ALTITUDE.cos() * HILLSHADE_AZIMUTH.cos(),
        HILLSHADE_ALTITUDE.cos() * HILLSHADE_AZIMUTH.sin(),
        HILLSHADE_ALTITUDE.sin(),
    )
}

/// 法线 -> hillshade 光照系数（`0..1`）。
fn shade_from_normal(normal: Vec3) -> f32 {
    HILLSHADE_AMBIENT + (1.0 - HILLSHADE_AMBIENT) * normal.dot(light_direction()).max(0.0)
}

/// 显示网格某格处的 hillshade 光照系数（`0..1`）。
///
/// 被垂直尺度压过的世界几何在这里完全不参与 —— 否则起伏在法线里同样被压掉。
/// 这是光照的**唯一**实现（网格顶点色、斑点指标与测试都用它）。
pub fn hillshade_illumination(grid: &DisplayGrid, i: usize, j: usize) -> f32 {
    shade_from_normal(grid_normal(grid, i, j))
}

/// 构建地形三角网格：顶点为显示网格的高度场，索引按行优先两三角一格。
///
/// `exaggeration` 只改世界 Y，`mode` 只改高度取值；两者都不改顶点数 / 索引数，
/// 因此自检打印的计数与网格断言不受影响。
pub fn build_terrain_mesh(
    terrain: &Terrain,
    stride: usize,
    exaggeration: f32,
    mode: Downsample,
) -> TerrainMesh {
    let grid = display_grid(terrain, stride, mode);
    let (n_rows, n_cols) = (grid.rows, grid.cols);
    let (min_h, max_h) = grid.min_max();
    let span = max_h - min_h;
    let half_cols = (terrain.cols() as f32 - 1.0) / 2.0;
    let half_rows = (terrain.rows() as f32 - 1.0) / 2.0;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n_rows * n_cols);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(n_rows * n_cols);
    for i in 0..n_rows {
        for j in 0..n_cols {
            let h = grid.height(i, j);
            let row = (i * grid.stride).min(terrain.rows().saturating_sub(1));
            let col = (j * grid.stride).min(terrain.cols().saturating_sub(1));
            positions.push([
                col as f32 - half_cols,
                height_to_world(h, exaggeration),
                row as f32 - half_rows,
            ]);
            let t = if span > f32::EPSILON {
                (h - min_h) / span
            } else {
                0.5
            };
            colors.push(band_linear(t, hillshade_illumination(&grid, i, j)));
        }
    }

    let mut indices: Vec<u32> =
        Vec::with_capacity(n_rows.saturating_sub(1) * n_cols.saturating_sub(1) * 6);
    for i in 0..n_rows.saturating_sub(1) {
        for j in 0..n_cols.saturating_sub(1) {
            let a = (i * n_cols + j) as u32;
            let b = a + 1;
            let c = a + n_cols as u32;
            let d = c + 1;
            // 绕序保证法线朝 +Y（a=(i,j), b=(i,j+1), c=(i+1,j), d=(i+1,j+1)）。
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
    mesh.compute_normals();

    TerrainMesh {
        mesh,
        n_rows,
        n_cols,
        stride: stride.max(1),
    }
}

/// 地形底座：四圈侧壁 + 一块底盖，从地形边缘一路下到 `min_h - base_depth`。
///
/// 侧壁顶点色取自边缘高度的色带（越往下越暗），所以底座不会变成一块死黑的板。
/// 材质用双面（`cull_mode: None`），因此不必纠结每面墙的绕序。
pub fn build_base_mesh(
    terrain: &Terrain,
    grid: &DisplayGrid,
    exaggeration: f32,
    base_depth: f32,
) -> Mesh {
    let (min_h, max_h) = grid.min_max();
    let span = max_h - min_h;
    let bottom = min_h - base_depth.max(0.0);
    let (raw_rows, raw_cols) = (terrain.rows(), terrain.cols());
    let top_of = |h: f32, brightness: f32| {
        let t = if span > f32::EPSILON {
            (h - min_h) / span
        } else {
            0.5
        };
        band_linear(t, brightness)
    };

    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut colors: Vec<[f32; 4]> = Vec::new();
    let push =
        |positions: &mut Vec<[f32; 3]>, colors: &mut Vec<[f32; 4]>, x: f32, h: f32, z: f32| {
            positions.push([x, height_to_world(h, exaggeration), z]);
            colors.push(top_of(h, 0.85));
            positions.push([x, height_to_world(bottom, exaggeration), z]);
            colors.push(top_of(h, 0.22));
        };

    // 四条边：每条边相邻采样点之间一个四边形（两个三角形）。
    let mut quads: Vec<(u32, u32, u32, u32)> = Vec::new();
    let base_index = |k: usize| (k * 2) as u32;
    // row = 0 与 row = last
    for row in [0usize, grid.rows.saturating_sub(1)] {
        let start = positions.len() / 2;
        for j in 0..grid.cols {
            let h = grid.height(row, j);
            let x = grid.world_col(j, raw_cols);
            let z = grid.world_row(row, raw_rows);
            push(&mut positions, &mut colors, x, h, z);
        }
        for j in 0..grid.cols.saturating_sub(1) {
            let k = start + j;
            quads.push((
                base_index(k),
                base_index(k + 1),
                base_index(k + 1) + 1,
                base_index(k) + 1,
            ));
        }
    }
    // col = 0 与 col = last
    for col in [0usize, grid.cols.saturating_sub(1)] {
        let start = positions.len() / 2;
        for i in 0..grid.rows {
            let h = grid.height(i, col);
            let x = grid.world_col(col, raw_cols);
            let z = grid.world_row(i, raw_rows);
            push(&mut positions, &mut colors, x, h, z);
        }
        for i in 0..grid.rows.saturating_sub(1) {
            let k = start + i;
            quads.push((
                base_index(k),
                base_index(k + 1),
                base_index(k + 1) + 1,
                base_index(k) + 1,
            ));
        }
    }

    let mut indices: Vec<u32> = Vec::new();
    for (a, b, c, d) in quads {
        indices.extend_from_slice(&[a, b, c, a, c, d]);
    }

    // 底盖：四个角，双面材质，颜色用色带最低端再压暗。
    let cap = positions.len() as u32;
    let xs = [
        grid.world_col(0, raw_cols),
        grid.world_col(grid.cols.saturating_sub(1), raw_cols),
    ];
    let zs = [
        grid.world_row(0, raw_rows),
        grid.world_row(grid.rows.saturating_sub(1), raw_rows),
    ];
    let cap_color = band_linear(0.0, 0.25);
    for (x, z) in [
        (xs[0], zs[0]),
        (xs[1], zs[0]),
        (xs[1], zs[1]),
        (xs[0], zs[1]),
    ] {
        positions.push([x, height_to_world(bottom, exaggeration), z]);
        colors.push(cap_color);
    }
    indices.extend_from_slice(&[cap, cap + 1, cap + 2, cap, cap + 2, cap + 3]);

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
    mesh.compute_normals();
    mesh
}

/// 显示网格的「相邻顶点法线夹角的中位数」（度）——**着色法线**（与 hillshade 同一个法线），
/// 不是 `compute_normals` 的面法线平均（后者会把规则高频图案整片抵消掉）。
///
/// 用法是同一命令下 `--nearest-sampling` 的前后对比：最近邻把高频混叠进法线，
/// 窗口均值把它压下去。空网格返回 0。
pub fn median_adjacent_normal_angle_deg(terrain: &Terrain, stride: usize, mode: Downsample) -> f32 {
    let grid = display_grid(terrain, stride, mode);
    if grid.rows == 0 || grid.cols == 0 {
        return 0.0;
    }
    let mut angles: Vec<f32> = Vec::with_capacity(grid.rows * grid.cols * 2);
    for i in 0..grid.rows {
        for j in 0..grid.cols {
            let n = grid_normal(&grid, i, j);
            if j + 1 < grid.cols {
                angles.push(n.angle_between(grid_normal(&grid, i, j + 1)).to_degrees());
            }
            if i + 1 < grid.rows {
                angles.push(n.angle_between(grid_normal(&grid, i + 1, j)).to_degrees());
            }
        }
    }
    median(&mut angles)
}

/// 显示网格的「二阶差分绝对值中位数」（厘米）——**斑点的直接量化**（高频能量）。
///
/// 窗口均值把每个采样点覆盖的 `stride²` 格平均掉，二阶差分的中位数会成倍下降；
/// 这条比法线夹角更灵敏（法线夹角被宏观起伏主导）。
pub fn median_high_frequency_cm(terrain: &Terrain, stride: usize, mode: Downsample) -> f32 {
    let grid = display_grid(terrain, stride, mode);
    if grid.rows < 3 || grid.cols < 3 {
        return 0.0;
    }
    let mut values: Vec<f32> = Vec::with_capacity(grid.rows * grid.cols * 2);
    for i in 1..grid.rows - 1 {
        for j in 1..grid.cols - 1 {
            let c = grid.height(i, j);
            values.push((grid.height(i, j + 1) + grid.height(i, j - 1) - 2.0 * c).abs());
            values.push((grid.height(i + 1, j) + grid.height(i - 1, j) - 2.0 * c).abs());
        }
    }
    median(&mut values)
}

/// 一组数的中位数（会就地排序）；空集返回 0。
fn median(values: &mut [f32]) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f32::total_cmp);
    values[values.len() / 2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::Terrain;

    fn terrain(rows: usize, cols: usize) -> Terrain {
        terrain_from(rows, cols, |r, c| ((r + c) % 256) as f64)
    }

    /// 用给定函数写一张矩阵地形 CSV 并读回。
    fn terrain_from(rows: usize, cols: usize, f: impl Fn(usize, usize) -> f64) -> Terrain {
        let text: String = (0..rows)
            .map(|r| {
                (0..cols)
                    .map(|c| f(r, c).to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            })
            .collect::<Vec<_>>()
            .join("\n");
        let path = std::env::temp_dir().join(format!(
            "trapla-viz-test-{rows}x{cols}-{}.csv",
            std::process::id()
        ));
        std::fs::write(&path, text).unwrap();
        let t = Terrain::read_csv(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        t
    }

    fn vertex_colors(mesh: &Mesh) -> Vec<[f32; 4]> {
        match mesh.attribute(Mesh::ATTRIBUTE_COLOR).unwrap() {
            bevy::mesh::VertexAttributeValues::Float32x4(v) => v.clone(),
            other => panic!("顶点色不是 Float32x4: {other:?}"),
        }
    }

    /// 真实地图尺寸下，自动步长必须让顶点数落在上限内（这是默认路径的硬约束）。
    #[test]
    fn auto_stride_keeps_vertices_within_cap() {
        let (rows, cols) = (1000, 3000);
        let s = auto_stride(rows, cols);
        let (r, c) = sample_shape(rows, cols, s);
        assert!(r * c <= MAX_VERTICES, "stride={s} -> {} 顶点", r * c);
        // 再小一步就会超限，说明取的是最小可行步长
        assert!(
            s == 1 || {
                let (pr, pc) = sample_shape(rows, cols, s - 1);
                pr * pc > MAX_VERTICES
            }
        );
        assert_eq!(s, 6);
    }

    /// 不同形状下都要满足上限。
    #[test]
    fn auto_stride_cap_holds_for_various_shapes() {
        for (rows, cols) in [(1, 1), (3, 7), (1000, 3000), (3000, 1000), (4096, 4096)] {
            let s = auto_stride(rows, cols);
            let (r, c) = sample_shape(rows, cols, s);
            assert!(
                r * c <= MAX_VERTICES,
                "{rows}x{cols} stride={s} -> {}",
                r * c
            );
        }
    }

    /// 网格顶点数与算式一致（自检打印的就是渲染用的那个网格）。
    #[test]
    fn mesh_vertex_count_matches_formula() {
        let t = terrain(20, 30);
        let mesh = build_terrain_mesh(&t, 3, DEFAULT_EXAGGERATION, Downsample::WindowMean);
        let (r, c) = sample_shape(20, 30, 3);
        assert_eq!(mesh.vertex_count(), r * c);
        assert_eq!(mesh.mesh.count_vertices(), r * c);
        assert_eq!(mesh.triangle_count(), (r - 1) * (c - 1) * 2);
    }

    /// 畸形输入不 panic：空行跳过、非数值报错。
    #[test]
    fn terrain_reads_skipping_blank_lines_and_rejects_non_numeric() {
        let p = std::env::temp_dir().join("trapla-viz-test-blank.csv");
        std::fs::write(&p, "1,2\n\n3,4\n").unwrap();
        let t = Terrain::read_csv(&p).unwrap();
        assert_eq!((t.rows(), t.cols()), (2, 2));
        std::fs::write(&p, "1,2\nx,4\n").unwrap();
        assert!(Terrain::read_csv(&p).is_err());
        let _ = std::fs::remove_file(&p);
    }

    /// `exaggeration` 只改世界 Y：顶点数/三角形数/水平坐标都不动。
    #[test]
    fn exaggeration_only_scales_height() {
        let t = terrain(10, 12);
        let low = build_terrain_mesh(&t, 1, 0.05, Downsample::WindowMean);
        let high = build_terrain_mesh(&t, 1, 1.0, Downsample::WindowMean);
        assert_eq!(low.vertex_count(), high.vertex_count());
        assert_eq!(low.triangle_count(), high.triangle_count());
        let positions = |m: &Mesh| match m.attribute(Mesh::ATTRIBUTE_POSITION).unwrap() {
            bevy::mesh::VertexAttributeValues::Float32x3(v) => v.clone(),
            other => panic!("顶点位置不是 Float32x3: {other:?}"),
        };
        let a = positions(&low.mesh);
        let b = positions(&high.mesh);
        for (p, q) in a.iter().zip(&b) {
            assert_eq!((p[0], p[2]), (q[0], q[2]));
            assert!((q[1] - p[1] * 20.0).abs() < 1e-3);
        }
    }

    /// 色带按**数据实际 min/max** 归一化：27..123 的地图应当把 8 段全部用到，
    /// 低处与高处的颜色差异要肉眼可辨（这正是「同一片绿」要治的点）。
    #[test]
    fn band_uses_real_data_range() {
        let t = terrain_from(40, 60, |r, c| 27.0 + ((r + c) % 97) as f64);
        let mesh = build_terrain_mesh(&t, 1, DEFAULT_EXAGGERATION, Downsample::WindowMean);
        let colors = vertex_colors(&mesh.mesh);
        let lum = |c: [f32; 4]| 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2];
        let min = colors.iter().map(|c| lum(*c)).fold(f32::INFINITY, f32::min);
        let max = colors
            .iter()
            .map(|c| lum(*c))
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            max / min > 2.0,
            "色带对比不足：min={min} max={max}（低处与高处区分不开）"
        );
    }

    /// hillshade 必须用**原始厘米梯度**：1 cm/格的坡要算成真正的坡，
    /// 而「1 cm/格 × 0.05」这种被压 20 倍后的几何坡度必须仍然接近平坦。
    /// 若把几何法线（世界 Y 已被压过）拿来算光照，前一条会退化到后一条、断言立刻红。
    #[test]
    fn hillshade_uses_raw_centimeter_gradient() {
        let flat = terrain_from(4, 40, |_r, _c| 0.0);
        let steep = terrain_from(4, 40, |_r, c| c as f64); // 1.0 cm/格
        let gentle = terrain_from(4, 40, |_r, c| c as f64 * 0.05); // ≈ 被压 20 倍后的坡度

        // stride = 1 ⇒ 显示网格就是原始格，梯度是「每格 1 cm」的原始厘米梯度。
        let flat_grid = display_grid(&flat, 1, Downsample::WindowMean);
        let steep_grid = display_grid(&steep, 1, Downsample::WindowMean);
        let gentle_grid = display_grid(&gentle, 1, Downsample::WindowMean);
        let i_flat = hillshade_illumination(&flat_grid, 2, 5);
        let i_steep = hillshade_illumination(&steep_grid, 2, 5);
        let i_gentle = hillshade_illumination(&gentle_grid, 2, 5);

        assert!(
            (i_steep - i_flat).abs() > 0.3,
            "1 cm/格的坡没被算成坡：flat={i_flat} steep={i_steep}"
        );
        assert!(
            i_gentle > i_flat - 0.25,
            "0.05 cm/格的缓坡不该接近全黑：flat={i_flat} gentle={i_gentle}"
        );
        // 关键判据：1 cm/格 与 0.05 cm/格 必须差出一大截（20 倍梯度差）。
        // 若用被 HEIGHT_SCALE 压过的几何坡度，两者会被压成同一个值，这条立刻红。
        assert!(
            i_gentle - i_steep > 0.2,
            "1 cm/格 与 0.05 cm/格 的光照差只有 {}（说明坡度被整体压过，用的不是原始厘米梯度）",
            i_gentle - i_steep
        );
    }

    /// 度量本身先在**已知答案**的输入上验证：平地与纯斜面（平面）两条指标都必须是 0；
    /// 高频条纹两条都必须很大。度量不对就先修度量，别去改被测对象。
    #[test]
    fn mesh_metric_probes_match_known_inputs() {
        let flat = terrain_from(12, 12, |_r, _c| 0.0);
        assert_eq!(
            median_adjacent_normal_angle_deg(&flat, 1, Downsample::WindowMean),
            0.0
        );
        assert_eq!(
            median_high_frequency_cm(&flat, 1, Downsample::WindowMean),
            0.0
        );

        // 线性斜面 = 平面：所有法线相同、二阶差分为 0。
        let ramp = terrain_from(20, 20, |r, c| (r + c) as f64 * 0.5);
        let ramp_angle = median_adjacent_normal_angle_deg(&ramp, 1, Downsample::WindowMean);
        assert!(
            ramp_angle < 0.01,
            "平面上的相邻法线夹角应为 0：{ramp_angle}"
        );
        assert_eq!(
            median_high_frequency_cm(&ramp, 1, Downsample::WindowMean),
            0.0
        );

        // 高频（且与采样格同尺度）的模式：中心差分非零、方向乱跳，两条指标都必须大。
        // 注意周期 2 的条纹不行 —— 中心差分正好把它抵消掉（Nyquist）。
        let noise = terrain_from(24, 24, |r, c| ((r * 7 + c * 13) % 11) as f64 * 10.0);
        let noise_angle = median_adjacent_normal_angle_deg(&noise, 1, Downsample::WindowMean);
        let noise_rough = median_high_frequency_cm(&noise, 1, Downsample::WindowMean);
        assert!(noise_angle > 20.0, "高频输入上的夹角应很大：{noise_angle}");
        assert!(
            noise_rough > 20.0,
            "高频输入上的二阶差分应很大：{noise_rough}"
        );
    }

    /// 窗口均值必须压低两条斑点指标（高频能量成倍下降），
    /// 且顶点数 / 三角形数两个口径完全一致（V1 的计数因此不变）。
    #[test]
    fn window_mean_reduces_spike_metrics() {
        // 单格尺度的小幅噪声（±5 cm，接近真实地图的局部起伏）。
        let t = terrain_from(60, 60, |r, c| {
            let h = ((r as u64 * 73_856_093) ^ (c as u64 * 19_349_663)) % 11;
            30.0 + h as f64 - 5.0
        });
        let stride = 3;
        let nearest = build_terrain_mesh(&t, stride, DEFAULT_EXAGGERATION, Downsample::Nearest);
        let mean = build_terrain_mesh(&t, stride, DEFAULT_EXAGGERATION, Downsample::WindowMean);
        assert_eq!(nearest.vertex_count(), mean.vertex_count());
        assert_eq!(nearest.triangle_count(), mean.triangle_count());

        let angle_n = median_adjacent_normal_angle_deg(&t, stride, Downsample::Nearest);
        let angle_m = median_adjacent_normal_angle_deg(&t, stride, Downsample::WindowMean);
        assert!(
            angle_m < angle_n,
            "法线夹角没有下降：最近邻 {angle_n}° vs 均值 {angle_m}°"
        );

        let rough_n = median_high_frequency_cm(&t, stride, Downsample::Nearest);
        let rough_m = median_high_frequency_cm(&t, stride, Downsample::WindowMean);
        assert!(
            rough_m < rough_n * 0.6,
            "高频能量没被压住：最近邻 {rough_n} cm vs 均值 {rough_m} cm"
        );
    }
}
