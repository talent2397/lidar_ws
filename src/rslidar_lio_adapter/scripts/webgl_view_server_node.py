#!/usr/bin/env python3
"""webgl_view_server_node.py

自建免费 WebGL 浏览器点云查看器 (不需要 Foxglove / 任何账号会员):

  1. 订阅轻量点云话题 (默认 /merged_points_lite, /merged_points_bev_lite)
  2. 把 PointCloud2 转成紧凑二进制帧 (WPCB: x/y/z/intensity float32)
  3. 同一端口(默认 8899)提供页面 + WebSocket 推流
  4. 浏览器用 three.js WebGL 渲染, 用调试电脑的 GPU, 不占 Jetson CPU

用法 (一般通过 web_view.launch.py 启动):
  ros2 run rslidar_lio_adapter webgl_view_server_node.py \
    --ros-args -p topics:=/merged_points_lite,/merged_points_bev_lite \
               -p port:=8899
"""

import asyncio
import array
import json
import socket
import struct
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import websockets
from websockets.http import Headers
from websockets.http11 import Response

FRAME_MAGIC = b"WPCB"
FRAME_VERSION = 1
FLOAT32 = 7
FLOAT64 = 8

_SCRIPT_DIR = Path(__file__).resolve().parent
_STATIC_CANDIDATES = [
    _SCRIPT_DIR / "webgl_viewer_static",                              # 源码目录直接运行
    _SCRIPT_DIR.parents[1] / "share" / "rslidar_lio_adapter" / "webgl_viewer_static",  # 安装后
]
STATIC_DIR = next((p for p in _STATIC_CANDIDATES if p.is_dir()), _STATIC_CANDIDATES[0])

MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".png": "image/png",
    ".ico": "image/x-icon",
}


def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


class WebGLViewServer(Node):
    def __init__(self):
        super().__init__("webgl_view_server")
        # HTTP 页面和 WebSocket 共用同一个端口, 只需放行一个端口
        self.port = int(self.declare_parameter("port", 8899).value)
        topics_cfg = self.declare_parameter(
            "topics", "/merged_points_lite,/merged_points_bev_lite").value
        self.topic_names = [t.strip() for t in topics_cfg.split(",") if t.strip()]
        self.topic_ids = {name: i + 1 for i, name in enumerate(self.topic_names)}
        self.topic_meta = [
            {"id": self.topic_ids[n], "name": n} for n in self.topic_names
        ]
        self.latest = {}      # topic_id -> (seq, frame_bytes)
        self.seq = {}         # topic_id -> next seq
        self.lock = threading.Lock()

        qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        for name in self.topic_names:
            tid = self.topic_ids[name]
            self.seq[tid] = 1
            self.create_subscription(
                PointCloud2, name,
                self._make_cb(tid, name), qos)
            self.get_logger().info(f"订阅: {name} -> topic_id={tid}")

    def _make_cb(self, tid, name):
        def cb(msg):
            try:
                frame = self._build_frame(msg, tid)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"{name} 解析失败: {exc}")
                return
            with self.lock:
                self.latest[tid] = (self.seq[tid], frame)
            self.seq[tid] += 1
        return cb

    def _build_frame(self, msg, tid):
        n = msg.width * msg.height
        if n == 0:
            return b""
        field_off = {f.name: f.offset for f in msg.fields}
        field_type = {f.name: f.datatype for f in msg.fields}
        xo = field_off.get("x", 0)
        yo = field_off.get("y", 4)
        zo = field_off.get("z", 8)
        io = field_off.get("intensity")
        it = field_type.get("intensity")
        step = msg.point_step
        data = bytes(msg.data)
        arr = array.array("f")

        for k in range(n):
            base = k * step
            x = struct.unpack_from("<f", data, base + xo)[0]
            y = struct.unpack_from("<f", data, base + yo)[0]
            z = struct.unpack_from("<f", data, base + zo)[0]
            if io is not None:
                if it == FLOAT32:
                    intensity = struct.unpack_from("<f", data, base + io)[0]
                elif it == FLOAT64:
                    intensity = struct.unpack_from("<d", data, base + io)[0]
                elif it == 2:      # UINT8
                    intensity = float(data[base + io])
                elif it == 4:      # UINT16
                    intensity = float(struct.unpack_from("<H", data, base + io)[0])
                elif it == 3:      # INT16
                    intensity = float(struct.unpack_from("<h", data, base + io)[0])
                elif it == 6:      # UINT32
                    intensity = float(struct.unpack_from("<I", data, base + io)[0])
                else:
                    intensity = 0.0
            else:
                intensity = 0.0
            arr.extend((x, y, z, intensity))

        header = struct.pack(
            "<4sBBQQI", FRAME_MAGIC, FRAME_VERSION, tid,
            self.seq[tid], msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            n)
        return header + arr.tobytes()

    async def _ws_handler(self, ws):
        subs = set()
        last_sent = {}
        try:
            await ws.send(json.dumps({"op": "hello", "topics": self.topic_meta}))
        except Exception:  # noqa: BLE001
            return

        async def sender():
            while True:
                to_send = []
                with self.lock:
                    for tid in list(subs):
                        item = self.latest.get(tid)
                        if item and last_sent.get(tid) != item[0]:
                            to_send.append(item)
                            last_sent[tid] = item[0]
                for item in to_send:
                    try:
                        await ws.send(item[1])
                    except Exception:  # noqa: BLE001
                        return
                await asyncio.sleep(0.04)

        async def receiver():
            async for raw in ws:
                try:
                    obj = json.loads(raw)
                except Exception:  # noqa: BLE001
                    continue
                if not isinstance(obj, dict):
                    continue
                op = obj.get("op")
                if op == "sub":
                    wanted = obj.get("topics", [])
                    subs.clear()
                    for name in wanted:
                        tid = self.topic_ids.get(name)
                        if tid:
                            subs.add(tid)
                elif op == "ping":
                    try:
                        await ws.send(json.dumps({"op": "pong"}))
                    except Exception:  # noqa: BLE001
                        return

        await asyncio.gather(sender(), receiver())

    def _serve_static(self, path):
        rel = path.lstrip("/").split("?", 1)[0]
        if not rel:
            rel = "index.html"
        target = (STATIC_DIR / rel).resolve()
        try:
            target.relative_to(STATIC_DIR.resolve())
        except ValueError:
            return Response(403, "Forbidden", Headers(), b"403 Forbidden")
        if not target.is_file():
            return Response(404, "Not Found", Headers(), b"404 Not Found")
        ctype = MIME.get(target.suffix.lower(), "application/octet-stream")
        return Response(200, "OK", Headers({"Content-Type": ctype}), target.read_bytes())

    def _process_request(self, connection, request):
        # WebSocket 升级请求必须放行 (返回 None), 否则会被当成页面请求
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None
        # 非 WebSocket 升级请求 -> 直接返回静态页面/脚本
        return self._serve_static(request.path)

    def _start_ws(self):
        async def serve_forever():
            async with websockets.serve(
                    self._ws_handler, "0.0.0.0", self.port,
                    max_size=None, ping_interval=10, ping_timeout=30,
                    process_request=self._process_request):
                await asyncio.Future()  # 永久运行

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(serve_forever())

    def start_servers(self):
        threading.Thread(target=self._start_ws, daemon=True).start()
        ip = get_lan_ip()
        self.get_logger().info(
            "\n"
            "===============================================\n"
            "  WebGL 查看器已启动 (免费自建, 无需任何账号)\n"
            f"  浏览器打开: http://{ip}:{self.port}\n"
            f"  WebSocket 同端口: ws://{ip}:{self.port}\n"
            f"  同网段只需放行 {self.port} 端口\n"
            "===============================================")


def main():
    rclpy.init()
    node = WebGLViewServer()
    node.start_servers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
