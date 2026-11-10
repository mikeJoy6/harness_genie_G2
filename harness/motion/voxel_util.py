"""voxel_util.py —— 3D 体素格的小工具：格坐标打包成 int64 键、26 邻域连通块。

head_depth_cloud（每帧确认后删小块）和 octomap_sweeper（清 octomap 里的零星格）共用。
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

_OFF = 1 << 20          # 每轴 21 位，格坐标 ±2^20（3cm 格 = ±31km）
NEIGHBORS26 = np.array([(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)])


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pack(k):
    """(N,3) 整数格坐标 → (N,) int64 键。"""
    k = np.asarray(k, np.int64) + _OFF
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def unpack(key):
    key = np.asarray(key, np.int64)
    return np.stack([(key >> 42) & 0x1FFFFF, (key >> 21) & 0x1FFFFF, key & 0x1FFFFF], axis=1) - _OFF


def keys_of(P, res):
    """(N,3) 点 → 所在格的键。"""
    return pack(np.floor(np.asarray(P) / res).astype(np.int64))


def components(ukeys, reach=1):
    """ukeys：排好序、不重复的格键。返回 (每格的连通块编号, 每块格数)。
    两格在每个轴上相差 ≤ reach 就算连着（reach=1 即 26 邻域；reach=2 容许中间空一格）。"""
    n = len(ukeys)
    if n == 0:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    k = unpack(ukeys)
    r_ = range(-reach, reach + 1)
    offs = [d for d in ((a, b, c) for a in r_ for b in r_ for c in r_) if d > (0, 0, 0)]   # 一半方向即可（无向图）
    rows, cols = [], []
    for d in offs:
        nk = pack(k + np.array(d))
        j = np.searchsorted(ukeys, nk)
        j[j >= n] = 0
        hit = ukeys[j] == nk
        rows.append(np.nonzero(hit)[0])
        cols.append(j[hit])
    r, c = np.concatenate(rows), np.concatenate(cols)
    _, lab = connected_components(coo_matrix((np.ones(len(r), np.int8), (r, c)), shape=(n, n)), directed=False)
    return lab, np.bincount(lab)


def dilate(ukeys):
    """格集合外扩一圈（26 邻域），返回排好序的不重复键。"""
    k = unpack(ukeys)
    return np.unique(np.concatenate([ukeys] + [pack(k + d) for d in NEIGHBORS26]))
