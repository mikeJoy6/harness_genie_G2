"""颜色表的读写,以及从点选的像素计算 HSV 范围。"""
from pathlib import Path

import numpy as np
import yaml

HEADER = ("# 可识别的颜色。每项是一组 HSV 范围 [[H,S,V 下限], [H,S,V 上限]],OpenCV 的 H 范围是 0~179。\n"
          "# 新颜色用 scripts/hsv_pick.py 点选添加,这里的名字会自动提供给模型。\n")


def load_colors(path):
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def save_color(path, name, ranges):
    colors = load_colors(path)
    colors[name] = ranges
    text = yaml.safe_dump(colors, allow_unicode=True, default_flow_style=None, sort_keys=False)
    Path(path).write_text(HEADER + text, encoding="utf-8")


def range_from_samples(samples, h_margin=8, s_margin=40, v_margin=50):
    """samples:N×3 的 HSV 像素。返回 HSV 范围列表;红色这种跨过 0 的色相会拆成两段。"""
    samples = np.asarray(samples, dtype=int).reshape(-1, 3)
    s_low, s_high = max(int(samples[:, 1].min()) - s_margin, 0), min(int(samples[:, 1].max()) + s_margin, 255)
    v_low, v_high = max(int(samples[:, 2].min()) - v_margin, 0), min(int(samples[:, 2].max()) + v_margin, 255)

    def band(h_low, h_high):
        return [[int(h_low), s_low, v_low], [int(h_high), s_high, v_high]]

    if np.median(samples[:, 1]) < 40:      # 黑、白、灰的色相没有意义,只靠 S 和 V 区分
        return [band(0, 179)]

    hues = np.unique(samples[:, 0])        # 色相是一个圆:最大的空隙之外就是物体的色相弧
    gaps = np.diff(np.r_[hues, hues[0] + 180])
    i = int(np.argmax(gaps))
    start, end = int(hues[(i + 1) % len(hues)]), int(hues[i])
    if end < start:
        end += 180
    low, high = start - h_margin, end + h_margin
    if high - low >= 179:
        return [band(0, 179)]
    if low < 0:
        return [band(low + 180, 179), band(0, high)]
    if high > 179:
        return [band(low, 179), band(0, high - 180)]
    return [band(low, high)]
