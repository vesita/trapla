"""行走路径与引导点的可视化。

对应原 ``scripts/plot_path.py`` / ``scripts/plot_guides.py``。两个函数都只把图
保存到显式给到的 PNG 路径；只有 ``show=True`` 才会调用 ``plt.show()``，
因此无 DISPLAY 的环境下不会阻塞。
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["SimHei", "FangSong", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False


def read_csv_points(path: str | Path) -> list[tuple[float, float]]:
    """读取带 ``x,y`` 表头的点文件。"""
    src = Path(path)
    if not src.is_file():
        raise FileNotFoundError(f"点文件不存在: {src}")
    points: list[tuple[float, float]] = []
    with src.open(encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            points.append((float(row["x"]), float(row["y"])))
    return points


def _save(fig, out_path: str | Path) -> Path:
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=300, bbox_inches="tight")
    return out


def plot_path(
    csv_path: str | Path, out_path: str | Path, show: bool = False
) -> dict:
    """按奇偶下标分足绘制 ``path_result.csv`` 中的路径。"""
    points = read_csv_points(csv_path)
    if not points:
        raise ValueError(f"没有路径点: {csv_path}")
    even, odd = points[::2], points[1::2]

    fig, ax = plt.subplots(figsize=(15, 10))
    ax.plot(
        [p[0] for p in even], [p[1] for p in even],
        "b-o", linewidth=2, markersize=6, label="足迹序列1", alpha=0.7,
    )
    ax.plot(
        [p[0] for p in odd], [p[1] for p in odd],
        "r-s", linewidth=2, markersize=6, label="足迹序列2", alpha=0.7,
    )
    ax.scatter(*points[0], color="green", s=150, label="起始点", zorder=5)
    ax.scatter(*points[-1], color="black", s=150, label="终点", zorder=5)
    for a, b in zip(points, points[1:]):
        ax.plot([a[0], b[0]], [a[1], b[1]], "g-", alpha=0.3, linewidth=1)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X 坐标", fontsize=12)
    ax.set_ylabel("Y 坐标", fontsize=12)
    ax.set_title("双足机器人行走路径可视化", fontsize=14)
    ax.legend(fontsize=10)
    ax.axis("equal")
    fig.tight_layout()

    out = _save(fig, out_path)
    if show:
        plt.show()
    plt.close(fig)
    return {
        "path": out,
        "total": len(points),
        "even": len(even),
        "odd": len(odd),
        "start": points[0],
        "end": points[-1],
    }


def plot_guides(
    guides_path: str | Path,
    directions_path: str | Path,
    out_path: str | Path,
    show: bool = False,
) -> dict:
    """绘制引导点（guides.csv）与指向点（direction.csv）。"""
    guides = read_csv_points(guides_path)
    directions = read_csv_points(directions_path)

    fig, ax = plt.subplots(figsize=(12, 10))
    if guides:
        ax.plot(
            [p[0] for p in guides], [p[1] for p in guides],
            "b-o", label="Guide Points", markersize=4,
        )
        ax.scatter(*guides[0], color="green", s=100, label="Start", zorder=5)
        ax.scatter(*guides[-1], color="red", s=100, label="End", zorder=5)
    if directions:
        ax.scatter(
            [p[0] for p in directions], [p[1] for p in directions],
            color="orange", s=80, label="Direction Point", zorder=5,
        )
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    ax.set_title("Guide Points and Direction Points Visualization")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis("equal")

    out = _save(fig, out_path)
    if show:
        plt.show()
    plt.close(fig)
    return {"path": out, "guides": len(guides), "directions": len(directions)}
