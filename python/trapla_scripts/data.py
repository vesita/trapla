"""地形高度图生成。

对应原 ``scripts/data_generator/gen_map.py``：7 段高度地形 + 高斯噪声，
1000x3000 的 uint8 矩阵，落盘为灰度 JPEG。

与原版的差别：噪声改用 ``np.random.default_rng(seed)``，seed 由调用方显式给出，
同一 seed 两次生成的文件逐字节相同。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

#: 地图契约：1000 行 x 3000 列，值域 0~255。
ROWS = 1000
COLS = 3000

#: gen-map 的默认随机种子；默认值本身就是确定性的，不传也复现。
DEFAULT_SEED = 0

#: (列起, 列止, 高度 cm)，依次覆盖 0~COLS 的全部列。
HEIGHT_PROFILES: tuple[tuple[int, int, int], ...] = (
    (0, 1000, 30),
    (1000, 1060, 60),
    (1060, 1120, 90),
    (1120, 1300, 120),
    (1300, 1500, 90),
    (1500, 2000, 60),
    (2000, 3000, 30),
)


def generate_terrain_map(
    rows: int = ROWS, cols: int = COLS, seed: int = DEFAULT_SEED
) -> np.ndarray:
    """返回 rows x cols 的 uint8 高度矩阵；同 seed 结果逐字节一致。"""
    rng = np.random.default_rng(seed)
    grid = np.ones((rows, cols), dtype=np.float64)
    for start, stop, height in HEIGHT_PROFILES:
        grid[:, start:stop] = height
    grid += rng.standard_normal((rows, cols))
    return np.clip(grid, 0, 255).astype(np.uint8)


def write_jpeg(grid: np.ndarray, out_path: str | Path) -> Path:
    """把高度矩阵写成灰度 JPEG，返回落盘路径。"""
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(grid).save(out, "JPEG")
    return out


def gen_map(
    out_path: str | Path,
    seed: int = DEFAULT_SEED,
    rows: int = ROWS,
    cols: int = COLS,
) -> dict:
    """生成地形图并落盘到 ``out_path``（路径由调用方显式给出，不猜 CWD）。"""
    grid = generate_terrain_map(rows=rows, cols=cols, seed=seed)
    write_jpeg(grid, out_path)
    return {
        "path": Path(out_path),
        "rows": rows,
        "cols": cols,
        "seed": seed,
        "min": int(grid.min()),
        "max": int(grid.max()),
    }
