"""物理进程(MuJoCo,Python 3.12)和 ROS 桥(rclpy,Python 3.10)之间的通信。只用标准库,两边都能 import。

为什么分两个进程:MuJoCo 4.x 要 Python ≥3.11,ROS Humble 的 rclpy 只有 3.10,装不进同一个 Python。

一条消息 = 一段 JSON 头 + 若干段原始字节(图像)。不用 pickle:两边 numpy 版本不同(2.x / 1.21),pickle 的数组读不回来。
    send(conn, {"type": "camera", ...}, [jpeg, depth, seg])
    header, blobs = recv(conn)
头里的 "blobs" 是字节段数,由 send 自动填。
"""
import json
import os
from multiprocessing.connection import Client, Listener

SOCKET_PATH = os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "g2_sim.sock")


def send(conn, header, blobs=()):
    header = dict(header)
    header["blobs"] = len(blobs)
    conn.send_bytes(json.dumps(header).encode("utf-8"))
    for blob in blobs:
        conn.send_bytes(blob)


def recv(conn):
    header = json.loads(conn.recv_bytes().decode("utf-8"))
    blobs = []
    for _ in range(header.get("blobs", 0)):
        blobs.append(conn.recv_bytes())
    return header, blobs


def listen(path=SOCKET_PATH):
    """物理进程用:开一个 Unix 套接字等桥连上来。"""
    if os.path.exists(path):
        os.unlink(path)
    return Listener(path, family="AF_UNIX")


def connect(path=SOCKET_PATH):
    """ROS 桥用。物理进程还没起来时抛 FileNotFoundError / ConnectionRefusedError。"""
    return Client(path, family="AF_UNIX")
