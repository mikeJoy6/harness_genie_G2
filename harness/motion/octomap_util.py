#!/usr/bin/env python3
"""octomap_util.py —— 把 MoveIt planning scene 里的 octomap(octomap_msgs/Octomap) 解码成占据体素中心点。

MoveIt 发的是【完整 OcTree】(binary=False, id="OcTree")：先序递归，每个节点 = float32 log-odds + uint8 子节点存在位，
没有子节点的就是叶子；叶子可以在任意层（粗层叶子代表一整块同值体素）。log-odds > 0 即占据。
不支持 binary=True 的 2bit 压缩格式（MoveIt 不发这种）。
"""
import struct

import numpy as np

TREE_DEPTH = 16


def occupied_centers(octomap_msg, max_points=200000):
    """返回 (N,3) 占据体素中心（octomap 的 frame，MoveIt 里就是 octomap_frame=base_link）。"""
    if octomap_msg.binary:
        raise ValueError("只支持完整 OcTree（binary=False），MoveIt planning scene 发的就是这种")
    res = octomap_msg.resolution
    data = np.asarray(octomap_msg.data, dtype=np.int8).view(np.uint8).tobytes()
    if not data:
        return np.empty((0, 3))
    keys = []
    stack = [((0, 0, 0), 0)]
    pos = 0
    # 先序遍历：用显式栈复现递归顺序（子节点按 0..7 逆序压栈，弹出时就是 0..7 顺序）
    while stack:
        key, depth = stack.pop()
        val = struct.unpack_from("<f", data, pos)[0]
        ch = data[pos + 4]
        pos += 5
        if ch == 0:
            if val > 0:
                s = 1 << (TREE_DEPTH - depth)
                if s == 1:
                    keys.append(key)
                else:
                    r = np.arange(s)
                    g = np.stack(np.meshgrid(r, r, r, indexing="ij"), -1).reshape(-1, 3) + np.array(key)
                    keys.extend(map(tuple, g))
                if len(keys) > max_points:
                    break
            continue
        half = 1 << (TREE_DEPTH - depth - 1)
        for i in reversed(range(8)):
            if ch & (1 << i):
                stack.append(((key[0] + (half if i & 1 else 0), key[1] + (half if i & 2 else 0),
                               key[2] + (half if i & 4 else 0)), depth + 1))
    if not keys:
        return np.empty((0, 3))
    return (np.asarray(keys, dtype=float) - (1 << (TREE_DEPTH - 1)) + 0.5) * res
