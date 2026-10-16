"""CLI 参数面：显式输出路径是强制项，缺参数必须失败。"""

import numpy as np
import pytest
from PIL import Image

from trapla_scripts.cli import main


def test_requires_subcommand():
    with pytest.raises(SystemExit) as exc:
        main([])
    assert exc.value.code == 2


def test_gen_map_requires_out(tmp_path):
    with pytest.raises(SystemExit) as exc:
        main(["gen-map", "--seed", "1"])
    assert exc.value.code == 2, "缺少 --out 必须失败，不能回落到默认路径"


def test_gen_map_writes_explicit_out(tmp_path):
    out = tmp_path / "m.jpg"
    assert main(["gen-map", "--seed", "7", "--out", str(out)]) == 0
    assert out.is_file() and out.stat().st_size > 0


def _small_jpg(path) -> None:
    data = (np.arange(12, dtype=np.uint8).reshape(3, 4) * 20)
    Image.fromarray(data, mode="L").save(path, "JPEG")


def test_convert_roundtrip_via_cli(tmp_path):
    jpg = tmp_path / "small.jpg"
    _small_jpg(jpg)
    csv_path = tmp_path / "small.csv"
    rt_jpg = tmp_path / "small_rt.jpg"
    assert main(["jpg2csv", "--jpg", str(jpg), "--csv", str(csv_path)]) == 0
    assert len(csv_path.read_text(encoding="utf-8").splitlines()) == 3
    assert main(["csv2jpg", "--csv", str(csv_path), "--jpg", str(rt_jpg)]) == 0
    assert rt_jpg.is_file()


def test_missing_input_exits_1(tmp_path):
    code = main(["jpg2csv", "--jpg", str(tmp_path / "nope.jpg"), "--csv", str(tmp_path / "x.csv")])
    assert code == 1
