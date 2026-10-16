# 脚本系统

仓库里的自动化脚本分两类，各有唯一的家：

| 类别 | 位置 | 细节在哪 |
|---|---|---|
| Python 工具箱 `trapla-data`（数据生成、格式转换、绘图） | `python/trapla_scripts/` | **`python/README.md`**（子命令、参数、数据契约、依赖、测试） |
| 跨设备同步未提交改动 | `scripts/sync/` | `doc/sync.md` |

```bash
uv sync                                   # 建 venv 并安装 trapla-data
uv run trapla-data --help                 # 子命令一览
uv run pytest python/tests -q             # 工具箱测试
```

常用子命令（完整参数见 `python/README.md`）：

```bash
uv run trapla-data gen-map  --seed 42 --out /tmp/map.jpg
uv run trapla-data jpg2csv  --jpg /tmp/map.jpg --csv /tmp/map.csv
uv run trapla-data plot-path --csv data/csv/path_result.csv --out /tmp/path.png
```

约定：输入输出路径一律显式给出（不按当前工作目录猜），stdout 只打印产出路径，进度与统计走 stderr。
