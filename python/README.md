# TraPLa Python 工具箱（`trapla-data`）

把仓库里散落的 Python 脚本收成一个 **uv 管理的按需工具箱**：单一入口 `trapla-data`，
输出路径一律显式给出，不猜 CWD。

## 安装与运行

```bash
uv sync                       # 建 .venv、装本项目（含 dev 组的 pytest）与全部依赖
uv run trapla-data --help
uv run trapla-data --version
```

`uv run` 直接在项目 venv 里执行，不需要手动 activate。

## 子命令

| 命令 | 作用 | 参数 |
|---|---|---|
| `gen-map` | 生成 1000x3000 地形高度图（灰度 JPEG） | `--out`（必填）、`--seed`（默认 0） |
| `jpg2csv` | 灰度 JPEG → 无表头数值 CSV | `--jpg`、`--csv`（都必填） |
| `csv2jpg` | 无表头数值 CSV → 灰度 JPEG | `--csv`、`--jpg`（都必填） |
| `plot-path` | 画 `path_result.csv` 的行走路径（按奇偶下标分足） | `--csv`、`--out`、`--show`（可选） |
| `plot-guides` | 画引导点与指向点 | `--guides`、`--direction`、`--out`、`--show`（可选） |

```bash
uv run trapla-data gen-map --seed 42 --out /tmp/map.jpg
uv run trapla-data jpg2csv --jpg /tmp/map.jpg --csv /tmp/map.csv
uv run trapla-data csv2jpg --csv /tmp/map.csv --jpg /tmp/map_rt.jpg
uv run trapla-data plot-path --csv data/csv/path_result.csv --out /tmp/path.png
uv run trapla-data plot-guides --guides log/guides.csv --direction log/direction.csv --out /tmp/guides.png
```

stdout 只打印产出的文件路径，进度与统计信息走 stderr；参数错误退 2，输入缺失等运行错误退 1。

## `gen-map` 的可复现性

- `--seed` 默认 **0**。
- 噪声走 `numpy.random.default_rng(seed)`：**同一 seed 两次生成的 JPEG sha256 相同**，
  **不同 seed 产出不同图像**。
- 输出尺寸与列分段固定：1000 行 x 3000 列，7 段高度 30/60/90/120/90/60/30（cm），
  加高斯噪声后 `clip(0, 255)`。

## 数据契约

- `map.jpg`：1000x3000 灰度 JPEG。
- `map.csv`：无表头、逗号分隔、LF 换行、1000 行 x 3000 列、值域 0~255。
- `path_result.csv` / `guides.csv` / `direction.csv`：带表头 `x,y`。

## 依赖

运行依赖在 `pyproject.toml` 的 `[project.dependencies]`（唯一事实源）：
**numpy、pillow、matplotlib**；开发依赖 `pytest` 在 `[dependency-groups] dev` 组。

入口点与包布局：

```toml
[project.scripts]
trapla-data = "trapla_scripts.cli:main"

[tool.uv.build-backend]
module-root = "python"
module-name = "trapla_scripts"
```

## 可视化行为

绘图默认只把 PNG 存到 `--out` 指定的路径，**不调用 `plt.show()`**：
无 `DISPLAY` 的环境下不会阻塞。需要交互窗口时显式加 `--show`。

## 与旧脚本、MATLAB 版的关系

- MATLAB 版 `scripts/data_generator/gen_map.m` 已由 Python 版取代，不移植、不维护，
  只保留 Python 一份实现。
- 仓库原有的 `scripts/` 脚本保持原样，本工具箱不复用它们相对 CWD 的默认路径。

## 测试

```bash
uv run pytest python/tests -q
```

覆盖：`gen-map` 的 seed 可复现与不同 seed 差异、`jpg2csv` 的 1000 行 x 3000 列与值域、
`csv2jpg` 往返、绘图在无 `DISPLAY` 的真进程里退 0 且不调用 `plt.show()`、
以及"缺 `--out` 必须失败"的路径约束。
