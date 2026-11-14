"""harness/perception/wrist_camera.py 的解码(不需要 ROS,消息用替身)。

    python3 -m pytest tests/test_wrist_camera.py -q
"""
import cv2
import numpy as np
import pytest

from harness.perception.wrist_camera import decode_color, decode_depth


class Msg:
    def __init__(self, **fields):
        for key, value in fields.items():
            setattr(self, key, value)


def test_decode_color_jpeg():
    image = np.zeros((480, 848, 3), np.uint8)
    image[:, :, 2] = 200
    ok, jpeg = cv2.imencode(".jpg", image)
    bgr = decode_color(Msg(data=jpeg.tobytes(), format="jpeg"))
    assert bgr.shape == (480, 848, 3) and bgr[0, 0, 2] > 150


def test_decode_depth_16uc1_with_row_padding():
    """step 比 width*2 大(行尾有填充)也要按 width 裁掉。"""
    raw = np.zeros((2, 6), np.uint16)
    raw[:, :4] = [[100, 200, 0, 300], [400, 500, 600, 700]]
    depth = decode_depth(Msg(encoding="16UC1", height=2, width=4, step=12, data=raw.tobytes()))
    assert depth.shape == (2, 4)
    assert depth[0, 1] == pytest.approx(0.2) and depth[0, 2] == 0.0


def test_decode_depth_unknown_encoding():
    with pytest.raises(ValueError):
        decode_depth(Msg(encoding="rgb8", height=1, width=1, step=3, data=b"\x00\x00\x00"))
