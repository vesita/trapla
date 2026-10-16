#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T0 猫步指标基线：从 baseline/frozen/seed-*.csv 的 x,y 序列算指标，写 baseline/catwalk_metrics.json。

分足规则（实测，见 baseline/catwalk_parity_scan.txt 与 baseline/catwalk_foot_probe.txt）：
  在 32 个未崩溃的 (seed, goal) 运行上，path 点严格交替，index 为偶数者=feet[0]=左脚，
  奇数者=feet[1]=右脚（first_foot 恒为 0）。因此按下标奇偶分足是成立的。
  注意：该结论依赖"点序列是严格交替的两足落足序列"；一旦后续实现改变了 path 的写入时机
  （例如只在 avoid_feet_cross 分支里 push 两个点），这个假设必须重新验证。

指标定义（全部是读码后自定的、写在这里作为唯一事实源）：
  n_points                      = CSV 行数
  total_length                  = sum |p[i]-p[i-1]|
  stride_sequence[i]            = |p[i+1]-p[i]|                     (长度 n-1)
  turn_angle_sequence_rad[i]    = atan2(cross(v_i, v_{i+1}), dot(...))，v_i = p[i+1]-p[i]  (长度 n-2，带符号)
  foot_separation_sequence[i]   = |p[i]-p[i-1]|  （奇偶分足成立时这就是"两足间距"）  (长度 n-1)
  separation_lateral_sequence[i]（i=2..n-2）= 以 p[i-1]->p[i+1] 为行进方向 u 时，
                                  p[i]-p[i-1] 在 u 法向上的分量（"相对行进方向的横向间距"）
  separation_along_sequence[i]  = 同一分解的沿行进方向分量
  foot_polyline_intersections   = 左右脚两条折线的线段相交对数（用项目自己的 segments_intersect
                                  语义：cross 乘积 <= 0 即算相交，包含共线/端点接触）
  proper_crossings              = 严格跨立（两侧异号）的相交对数，排除接触
  goal_error                    = |最后一个点 - goal|
  spacing_violations_2_10       = foot_separation 落在 [2,10] 之外的点对数
                                  （注意：这是"足中心距"代理，与 C++ 的 satisfy_spacing
                                    几何定义不同，后者用四角到支撑脚中心线的垂距再减半宽）
"""
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASELINE = os.path.dirname(HERE)
FROZEN = os.path.join(BASELINE, "frozen")

CASES = [
    (3, "default", (500.0, 500.0)),
    (3, "alt", (900.0, 1200.0)),
    (6, "default", (500.0, 500.0)),
    (6, "alt", (900.0, 1200.0)),
    (7, "default", (500.0, 500.0)),
    (7, "alt", (900.0, 1200.0)),
]


def read_points(path):
    pts = []
    with open(path, newline="", encoding="utf-8") as fh:
        rd = csv.reader(fh)
        for i, row in enumerate(rd):
            if not row:
                continue
            if i == 0 and row[0].strip() == "x":
                continue
            pts.append((float(row[0]), float(row[1])))
    return pts


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def norm(v):
    return math.hypot(v[0], v[1])


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def segments_intersect_predicate(p1, p2, p3, p4):
    """与 src/utils/geometry.cpp:1242-1258 segments_intersect 同语义。"""
    if (min(p1[0], p2[0]) > max(p3[0], p4[0]) or
            min(p3[0], p4[0]) > max(p1[0], p2[0]) or
            min(p1[1], p2[1]) > max(p3[1], p4[1]) or
            min(p3[1], p4[1]) > max(p1[1], p2[1])):
        return False
    c1 = cross(sub(p2, p1), sub(p3, p1))
    c2 = cross(sub(p2, p1), sub(p4, p1))
    c3 = cross(sub(p4, p3), sub(p1, p3))
    c4 = cross(sub(p4, p3), sub(p2, p3))
    return (c1 * c2 <= 0) and (c3 * c4 <= 0)


def proper_crossing(p1, p2, p3, p4):
    """严格跨立（两端分别在对方两侧且不为 0）。"""
    c1 = cross(sub(p2, p1), sub(p3, p1))
    c2 = cross(sub(p2, p1), sub(p4, p1))
    c3 = cross(sub(p4, p3), sub(p1, p3))
    c4 = cross(sub(p4, p3), sub(p2, p3))
    return (c1 * c2 < 0) and (c3 * c4 < 0)


def polyline_intersections(even, odd):
    pred = 0
    proper = 0
    for j in range(len(even) - 1):
        for k in range(len(odd) - 1):
            if segments_intersect_predicate(even[j], even[j + 1], odd[k], odd[k + 1]):
                pred += 1
                if proper_crossing(even[j], even[j + 1], odd[k], odd[k + 1]):
                    proper += 1
    return pred, proper


def metrics_for(points, goal):
    n = len(points)
    strides = [norm(sub(points[i + 1], points[i])) for i in range(n - 1)]
    turns = []
    for i in range(n - 2):
        v_i = sub(points[i + 1], points[i])
        v_j = sub(points[i + 2], points[i + 1])
        turns.append(math.atan2(cross(v_i, v_j), dot(v_i, v_j)))
    separations = [norm(sub(points[i], points[i - 1])) for i in range(1, n)]
    lateral = []
    along = []
    for i in range(1, n - 1):
        u = sub(points[i + 1], points[i - 1])
        v = sub(points[i], points[i - 1])
        lu = norm(u)
        if lu == 0:
            lateral.append(0.0)
            along.append(0.0)
            continue
        lateral.append(abs(cross(u, v)) / lu)
        along.append(dot(u, v) / lu)
    even = [points[i] for i in range(0, n, 2)]
    odd = [points[i] for i in range(1, n, 2)]
    pred, proper = polyline_intersections(even, odd)
    violations = sum(1 for s in separations if s < 2.0 or s > 10.0)
    return {
        "n_points": n,
        "total_length": sum(strides),
        "step_count_transitions": n - 1,
        "stride_sequence": strides,
        "turn_angle_sequence_rad": turns,
        "foot_separation_sequence": separations,
        "separation_lateral_sequence": lateral,
        "separation_along_sequence": along,
        "min_foot_separation": min(separations) if separations else None,
        "max_foot_separation": max(separations) if separations else None,
        "mean_foot_separation": (sum(separations) / len(separations)) if separations else None,
        "spacing_violations_2_10": violations,
        "foot_polyline_intersections_predicate": pred,
        "proper_crossings": proper,
        "goal_error": norm(sub(points[-1], goal)),
    }


def main():
    out = {
        "generated_by": "baseline/tools/catwalk_metrics.py",
        "foot_split_rule": {
            "rule": "even index -> feet[0] (left), odd index -> feet[1] (right)",
            "evidence_strength": "实测",
            "evidence": "baseline/catwalk_parity_scan.txt: 32/32 completed runs strictly alternating, first_foot=0; 8/40 combos crashed (exit=134) and are excluded",
            "verified_against": "baseline/catwalk_foot_probe.txt",
        },
        "metric_definitions": "见本脚本 docstring（唯一事实源）",
        "note": "SqDot::scale 的 const/非 const 双语义、satisfy_spacing 的几何定义等陷阱见 baseline/README.md",
        "cases": [],
    }
    for seed, iname, goal in CASES:
        path = os.path.join(FROZEN, "seed-%d_%s_run1.csv" % (seed, iname))
        pts = read_points(path)
        m = metrics_for(pts, goal)
        m.update({
            "seed": seed,
            "input_label": iname,
            "input_goal": list(goal),
            "input_note": "main.cpp 只把 goal 传给 find_path；start 参数读了但从未使用",
            "source_csv": "baseline/frozen/seed-%d_%s_run1.csv" % (seed, iname),
        })
        out["cases"].append(m)
    dst = os.path.join(BASELINE, "catwalk_metrics.json")
    with open(dst, "w", encoding="utf-8") as fh:
        json.dump(out, fh, ensure_ascii=False, indent=2, sort_keys=False)
        fh.write("\n")
    print("written", dst)
    for c in out["cases"]:
        print("seed=%d %-7s n=%3d len=%8.3f goal_err=%7.3f sep[min,mean,max]=%.3f/%.3f/%.3f viol2_10=%d inter=%d proper=%d"
              % (c["seed"], c["input_label"], c["n_points"], c["total_length"], c["goal_error"],
                 c["min_foot_separation"], c["mean_foot_separation"], c["max_foot_separation"],
                 c["spacing_violations_2_10"], c["foot_polyline_intersections_predicate"], c["proper_crossings"]))


if __name__ == "__main__":
    sys.exit(main())
