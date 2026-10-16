"""gen-map 的可复现性与数据契约。"""

import hashlib
from pathlib import Path

import numpy as np

from trapla_scripts.data import COLS, DEFAULT_SEED, ROWS, generate_terrain_map, gen_map


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_default_seed_is_reproducible(tmp_path):
    a, b = tmp_path / "a.jpg", tmp_path / "b.jpg"
    gen_map(a)
    gen_map(b)
    assert _sha256(a) == _sha256(b), "默认 seed 下两次生成必须逐字节一致"


def test_seed_42_is_reproducible(tmp_path):
    a, b = tmp_path / "m1.jpg", tmp_path / "m2.jpg"
    gen_map(a, seed=42)
    gen_map(b, seed=42)
    assert _sha256(a) == _sha256(b)


def test_seed_43_differs(tmp_path):
    a, b = tmp_path / "m1.jpg", tmp_path / "m3.jpg"
    gen_map(a, seed=42)
    gen_map(b, seed=43)
    assert _sha256(a) != _sha256(b), "不同 seed 必须产出不同图像"


def test_shape_dtype_and_range():
    grid = generate_terrain_map(seed=DEFAULT_SEED)
    assert grid.shape == (ROWS, COLS)
    assert grid.dtype == np.uint8
    assert 0 < int(grid.min()) and int(grid.max()) <= 255


def test_height_profiles_present():
    grid = generate_terrain_map(seed=DEFAULT_SEED)
    profiles = ((500, 30), (1030, 60), (1090, 90), (1200, 120), (1400, 90), (1700, 60), (2500, 30))
    for col, expected in profiles:
        mean = float(grid[:, col].mean())
        assert abs(mean - expected) < 3, f"列 {col} 段高度应约等于 {expected}，实测 {mean:.2f}"
