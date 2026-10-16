"""``map.jpg`` 与 ``map.csv`` 互转。

对应原 ``scripts/transor/jpg2csv.py``。CSV 契约：无表头、逗号分隔、LF 换行、
每行一个像素行、值域 0~255。
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
from PIL import Image


def jpg_to_csv(jpg_path: str | Path, csv_path: str | Path) -> tuple[int, int]:
    """把灰度 JPEG 写成无表头数值 CSV，返回 (行数, 列数)。"""
    src = Path(jpg_path)
    if not src.is_file():
        raise FileNotFoundError(f"输入 JPEG 不存在: {src}")
    with Image.open(src) as img:
        grid = np.array(img.convert("L"))
    out = Path(csv_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh, lineterminator="\n")
        writer.writerows(grid)
    return grid.shape


def csv_to_jpg(csv_path: str | Path, jpg_path: str | Path) -> tuple[int, int]:
    """把无表头数值 CSV 写回灰度 JPEG，返回 (行数, 列数)。"""
    src = Path(csv_path)
    if not src.is_file():
        raise FileNotFoundError(f"输入 CSV 不存在: {src}")
    grid = np.loadtxt(src, delimiter=",", dtype=np.float64, ndmin=2)
    if grid.size and (grid.min() < 0 or grid.max() > 255):
        raise ValueError(f"CSV 含超出 0~255 的值: min={grid.min()}, max={grid.max()}")
    data = grid.astype(np.uint8)
    out = Path(jpg_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(data, mode="L").save(out, "JPEG")
    return data.shape
