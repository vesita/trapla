"""绘图在无显示环境下必须能存出 PNG 且不阻塞（不调 plt.show()）。"""

import os
import subprocess
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

from trapla_scripts.plot import plot_guides, plot_path

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _write_points(path: Path, points) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        fh.write("x,y\n")
        for x, y in points:
            fh.write(f"{x},{y}\n")


def _cli_env() -> dict:
    env = dict(os.environ)
    env.pop("DISPLAY", None)
    env.pop("MPLBACKEND", None)
    return env


def test_plot_path_cli_without_display(tmp_path):
    """无 DISPLAY 下真进程跑 CLI：若 plt.show() 阻塞，timeout 会杀掉进程而非退 0。"""
    csv_path = tmp_path / "path_result.csv"
    _write_points(csv_path, [(i * 3.0, (i % 7) * 2.0) for i in range(12)])
    out = tmp_path / "path.png"
    proc = subprocess.run(
        [sys.executable, "-m", "trapla_scripts", "plot-path", "--csv", str(csv_path), "--out", str(out)],
        capture_output=True, text=True, timeout=60, env=_cli_env(),
    )
    assert proc.returncode == 0, f"stderr=\n{proc.stderr}"
    assert out.read_bytes()[:8] == PNG_MAGIC


def test_plot_path_never_calls_show_by_default(tmp_path, monkeypatch):
    csv_path = tmp_path / "path_result.csv"
    _write_points(csv_path, [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0), (3.0, 1.0)])
    out = tmp_path / "path.png"

    def _boom(*_args, **_kwargs):
        raise AssertionError("默认不应调用 plt.show()")

    monkeypatch.setattr(matplotlib.pyplot, "show", _boom)
    stats = plot_path(csv_path, out)
    assert stats["total"] == 4
    assert out.read_bytes()[:8] == PNG_MAGIC


def test_plot_guides_saves_png(tmp_path, monkeypatch):
    guides = tmp_path / "guides.csv"
    directions = tmp_path / "direction.csv"
    _write_points(guides, [(0.0, 0.0), (5.0, 5.0), (10.0, 0.0)])
    _write_points(directions, [(2.0, 3.0), (8.0, -1.0)])
    out = tmp_path / "guides.png"

    def _boom(*_args, **_kwargs):
        raise AssertionError("默认不应调用 plt.show()")

    monkeypatch.setattr(plt, "show", _boom)
    stats = plot_guides(guides, directions, out)
    assert stats == {"path": out, "guides": 3, "directions": 2}
    assert out.read_bytes()[:8] == PNG_MAGIC


def test_plot_path_cli_env_display_unset_via_subprocess_missing_file(tmp_path):
    """显式路径不存在时 fail-closed，退出码 1 而不是静默成功。"""
    proc = subprocess.run(
        [sys.executable, "-m", "trapla_scripts", "plot-path",
         "--csv", str(tmp_path / "nope.csv"), "--out", str(tmp_path / "p.png")],
        capture_output=True, text=True, timeout=60, env=_cli_env(),
    )
    assert proc.returncode == 1
    assert "错误" in proc.stderr
