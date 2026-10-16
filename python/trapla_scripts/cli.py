"""``trapla-data`` 命令行入口。

约定：每个写文件的子命令都必须显式给出输出路径（``--out`` / ``--csv`` / ``--jpg``），
不会回落到 CWD 或仓库里的 ``data/``；stdout 只打印产出的文件路径，进度信息走 stderr。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from trapla_scripts import __version__
from trapla_scripts.convert import csv_to_jpg, jpg_to_csv
from trapla_scripts.data import DEFAULT_SEED, gen_map


def _cmd_gen_map(args: argparse.Namespace) -> int:
    stats = gen_map(args.out, seed=args.seed)
    print(stats["path"])
    print(
        f"尺寸 {stats['cols']}x{stats['rows']}，seed={stats['seed']}，"
        f"高度范围 {stats['min']}~{stats['max']}",
        file=sys.stderr,
    )
    return 0


def _cmd_jpg2csv(args: argparse.Namespace) -> int:
    rows, cols = jpg_to_csv(args.jpg, args.csv)
    print(Path(args.csv))
    print(f"图像尺寸 {cols} x {rows}", file=sys.stderr)
    return 0


def _cmd_csv2jpg(args: argparse.Namespace) -> int:
    rows, cols = csv_to_jpg(args.csv, args.jpg)
    print(Path(args.jpg))
    print(f"图像尺寸 {cols} x {rows}", file=sys.stderr)
    return 0


def _cmd_plot_path(args: argparse.Namespace) -> int:
    if not args.show:
        os.environ.setdefault("MPLBACKEND", "Agg")
    from trapla_scripts.plot import plot_path

    stats = plot_path(args.csv, args.out, show=args.show)
    print(stats["path"])
    print(
        f"路径点总数 {stats['total']}（序列1 {stats['even']}，序列2 {stats['odd']}）",
        file=sys.stderr,
    )
    return 0


def _cmd_plot_guides(args: argparse.Namespace) -> int:
    if not args.show:
        os.environ.setdefault("MPLBACKEND", "Agg")
    from trapla_scripts.plot import plot_guides

    stats = plot_guides(args.guides, args.direction, args.out, show=args.show)
    print(stats["path"])
    print(
        f"引导点 {stats['guides']}，指向点 {stats['directions']}",
        file=sys.stderr,
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="trapla-data",
        description="TraPLa 数据工具箱：地形图生成、JPEG/CSV 互转、路径可视化。",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True, metavar="命令")

    p = sub.add_parser("gen-map", help="生成 1000x3000 地形高度图 JPEG")
    p.add_argument("--out", required=True, type=Path, help="输出 JPEG 路径（必填）")
    p.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
        help=f"随机种子（默认 {DEFAULT_SEED}；同 seed 输出逐字节一致）",
    )
    p.set_defaults(handler=_cmd_gen_map)

    p = sub.add_parser("jpg2csv", help="灰度 JPEG -> 无表头数值 CSV")
    p.add_argument("--jpg", required=True, type=Path, help="输入 JPEG 路径")
    p.add_argument("--csv", required=True, type=Path, help="输出 CSV 路径")
    p.set_defaults(handler=_cmd_jpg2csv)

    p = sub.add_parser("csv2jpg", help="无表头数值 CSV -> 灰度 JPEG")
    p.add_argument("--csv", required=True, type=Path, help="输入 CSV 路径")
    p.add_argument("--jpg", required=True, type=Path, help="输出 JPEG 路径")
    p.set_defaults(handler=_cmd_csv2jpg)

    p = sub.add_parser("plot-path", help="绘制 path_result.csv（表头 x,y）中的行走路径")
    p.add_argument("--csv", required=True, type=Path, help="输入 path_result.csv（表头 x,y）")
    p.add_argument("--out", required=True, type=Path, help="输出 PNG 路径")
    p.add_argument("--show", action="store_true", help="保存后额外弹交互窗口（默认只保存）")
    p.set_defaults(handler=_cmd_plot_path)

    p = sub.add_parser("plot-guides", help="绘制引导点与指向点")
    p.add_argument("--guides", required=True, type=Path, help="输入 guides.csv（表头 x,y）")
    p.add_argument("--direction", required=True, type=Path, help="输入 direction.csv（表头 x,y）")
    p.add_argument("--out", required=True, type=Path, help="输出 PNG 路径")
    p.add_argument("--show", action="store_true", help="保存后额外弹交互窗口（默认只保存）")
    p.set_defaults(handler=_cmd_plot_guides)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.handler(args)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(f"trapla-data: 错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
