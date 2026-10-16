"""jpg2csv / csv2jpg 的形状、值域与往返。"""

import numpy as np
import pytest
from PIL import Image

from trapla_scripts.convert import csv_to_jpg, jpg_to_csv
from trapla_scripts.data import gen_map


@pytest.fixture(scope="module")
def roundtrip(tmp_path_factory):
    work = tmp_path_factory.mktemp("roundtrip")
    src_jpg = work / "map.jpg"
    csv_path = work / "map.csv"
    rt_jpg = work / "map_rt.jpg"
    gen_map(src_jpg, seed=42)
    csv_shape = jpg_to_csv(src_jpg, csv_path)
    jpg_shape = csv_to_jpg(csv_path, rt_jpg)
    return src_jpg, csv_path, rt_jpg, csv_shape, jpg_shape


def test_jpg_to_csv_shape(roundtrip):
    assert roundtrip[3] == (1000, 3000)


def test_csv_contract(roundtrip):
    csv_path = roundtrip[1]
    lines = csv_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 1000, "map.csv 必须是 1000 行"
    first = lines[0].split(",")
    assert len(first) == 3000, "map.csv 每行必须是 3000 列"
    assert len({len(line.split(",")) for line in lines}) == 1, "所有行列数必须一致"
    values = [int(v) for v in lines[0].split(",")]
    assert all(0 <= v <= 255 for v in values)


def test_csv2jpg_shape_and_roundtrip_close(roundtrip):
    src_jpg, _, rt_jpg, _, jpg_shape = roundtrip
    assert jpg_shape == (1000, 3000)
    original = np.asarray(Image.open(src_jpg).convert("L"), dtype=np.int16)
    repeated = np.asarray(Image.open(rt_jpg).convert("L"), dtype=np.int16)
    assert original.shape == repeated.shape
    assert float(np.abs(original - repeated).mean()) < 5.0, "JPEG 有损，但往返差异应很小"


def test_csv_out_of_range_rejected(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_text("0,1,300\n", encoding="utf-8")
    with pytest.raises(ValueError):
        csv_to_jpg(bad, tmp_path / "bad.jpg")


def test_missing_input_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        jpg_to_csv(tmp_path / "nope.jpg", tmp_path / "x.csv")
    with pytest.raises(FileNotFoundError):
        csv_to_jpg(tmp_path / "nope.csv", tmp_path / "x.jpg")
