//! 场景几何：降采样、地形网格、地图坐标 -> 世界坐标。

use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;

use crate::data::{FootPath, Terrain};

/// 地形网格顶点数上限（默认降采样必须落在它以下）。
pub const MAX_VERTICES: usize = 100_000;

/// 高度缩放：灰度值 `0..=255` 乘以它得到世界 Y。
pub const HEIGHT_SCALE: f32 = 0.05;

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

/// 地图坐标 -> 世界坐标：X = 列、Z = 行、Y = 最近邻高度 * [`HEIGHT_SCALE`]。
pub fn world_from_map(terrain: &Terrain, map_x: f64, map_y: f64) -> Vec3 {
    let half_cols = (terrain.cols() as f32 - 1.0) / 2.0;
    let half_rows = (terrain.rows() as f32 - 1.0) / 2.0;
    let h = terrain.value_nearest(map_x, map_y) as f32;
    Vec3::new(
        map_y as f32 - half_cols,
        h * HEIGHT_SCALE,
        map_x as f32 - half_rows,
    )
}

/// 路径上每个点的世界坐标（抬离地面一点，避免与地形共面闪烁）。
pub fn path_world_points(terrain: &Terrain, path: &FootPath) -> Vec<Vec3> {
    path.points()
        .iter()
        .map(|&(x, y)| world_from_map(terrain, x, y) + Vec3::Y * 6.0)
        .collect()
}

/// 构建地形三角网格：顶点为降采样后的高度场，索引按行优先两三角一格。
pub fn build_terrain_mesh(terrain: &Terrain, stride: usize) -> TerrainMesh {
    let (n_rows, n_cols) = sample_shape(terrain.rows(), terrain.cols(), stride);
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n_rows * n_cols);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(n_rows * n_cols);
    for i in 0..n_rows {
        for j in 0..n_cols {
            let row = (i * stride).min(terrain.rows().saturating_sub(1));
            let col = (j * stride).min(terrain.cols().saturating_sub(1));
            let h = terrain.value(row, col).unwrap_or(0.0) as f32;
            positions.push([
                col as f32 - (terrain.cols() as f32 - 1.0) / 2.0,
                h * HEIGHT_SCALE,
                row as f32 - (terrain.rows() as f32 - 1.0) / 2.0,
            ]);
            colors.push(height_color(h));
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

/// 按高度上色（低处偏绿、高处偏亮），供网格顶点色使用。
fn height_color(h: f32) -> [f32; 4] {
    let t = (h / 255.0).clamp(0.0, 1.0);
    let c = Color::srgb(0.22 + 0.55 * t, 0.55 + 0.35 * t, 0.22 + 0.72 * t);
    let l = c.to_linear();
    [l.red, l.green, l.blue, 1.0]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::Terrain;

    fn terrain(rows: usize, cols: usize) -> Terrain {
        let text: String = (0..rows)
            .map(|r| {
                (0..cols)
                    .map(|c| ((r + c) % 256).to_string())
                    .collect::<Vec<_>>()
                    .join(",")
            })
            .collect::<Vec<_>>()
            .join("\n");
        let path = std::env::temp_dir().join(format!("trapla-viz-test-{rows}x{cols}.csv"));
        std::fs::write(&path, text).unwrap();
        let t = Terrain::read_csv(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        t
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
        let mesh = build_terrain_mesh(&t, 3);
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
}
