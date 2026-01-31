#!/usr/bin/env python3
"""ROS2 -> Web bridge for fusion perception visualization."""

from __future__ import annotations

import argparse
import collections
import math
import os
import pathlib
import threading
import time
from typing import Dict, List, Optional

try:
    import yaml
except Exception:  # pragma: no cover - optional dependency
    yaml = None

try:
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge

from flask import Flask, Response, jsonify, render_template, request

DEFAULT_IMAGE_TOPIC = "/fusion_perception/visualization"
DEFAULT_MARKER_TOPIC = "/fusion_perception/obstacles"
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_JPEG_QUALITY = 80
DEFAULT_MAX_FPS = 15.0
DEFAULT_MAP_RANGE = {
    "x_min": -5.0,
    "x_max": 20.0,
    "y_min": -5.0,
    "y_max": 5.0,
}
KNOWN_ENCODINGS = {
    "bgr8",
    "rgb8",
    "bgra8",
    "rgba8",
    "mono8",
    "mono16",
    "16uc1",
    "16uc3",
    "32fc1",
    "32fc3",
}


class SharedState:
    def __init__(self, map_range: Dict[str, float], configured_cameras: List[str]):
        self.lock = threading.RLock()
        self.cond = threading.Condition(self.lock)
        self.images: Dict[str, bytes] = {}
        self.image_meta: Dict[str, Dict[str, float]] = {}
        self.image_seq = 0
        self.image_times = collections.deque(maxlen=60)
        self.default_camera: Optional[str] = None
        self.configured_cameras = configured_cameras
        self.markers: List[Dict[str, float]] = []
        self.marker_seq = 0
        self.marker_times = collections.deque(maxlen=60)
        self.last_marker_stamp: Optional[float] = None
        self.map_range = map_range
        self.placeholder = None
        self.max_fps = DEFAULT_MAX_FPS

    def update_image(self, camera: str, jpeg_bytes: bytes, stamp: float, width: int, height: int) -> None:
        with self.cond:
            self.images[camera] = jpeg_bytes
            self.image_meta[camera] = {
                "stamp": stamp,
                "width": width,
                "height": height,
            }
            if self.default_camera is None:
                self.default_camera = camera
            self.image_seq += 1
            self.image_times.append(time.time())
            self.cond.notify_all()

    def update_markers(self, markers: List[Dict[str, float]], stamp: float) -> None:
        with self.cond:
            self.markers = markers
            self.marker_seq += 1
            self.marker_times.append(time.time())
            self.last_marker_stamp = stamp
            self.cond.notify_all()

    def get_cameras(self) -> List[str]:
        with self.lock:
            cameras = set(self.images.keys()) | set(self.configured_cameras)
            return sorted(cameras)


class WebVisualizerNode(Node):
    def __init__(
        self,
        state: SharedState,
        image_topic: str,
        marker_topic: str,
        jpeg_quality: int,
        max_fps: float,
    ) -> None:
        super().__init__("web_visualizer_node")
        self.state = state
        self.bridge = CvBridge()
        self.jpeg_quality = jpeg_quality
        self.max_fps = max_fps
        self._last_image_time = 0.0
        self._warned_no_cv2 = False
        self._warned_encodings = set()

        self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.create_subscription(MarkerArray, marker_topic, self.marker_callback, 10)
        self.get_logger().info(f"Listening for images on {image_topic}")
        self.get_logger().info(f"Listening for markers on {marker_topic}")

    def image_callback(self, msg: Image) -> None:
        if cv2 is None:
            if not self._warned_no_cv2:
                self.get_logger().error("OpenCV not available; image stream disabled")
                self._warned_no_cv2 = True
            return

        now = time.time()
        if self.max_fps > 0 and (now - self._last_image_time) < (1.0 / self.max_fps):
            return
        self._last_image_time = now

        camera = msg.header.frame_id or "camera"
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        cv_image = self.decode_to_bgr(msg)
        if cv_image is None:
            return

        jpeg_bytes = encode_jpeg(cv_image, self.jpeg_quality)
        if not jpeg_bytes:
            return

        self.state.update_image(camera, jpeg_bytes, stamp, cv_image.shape[1], cv_image.shape[0])

    def decode_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        encoding = (msg.encoding or "").lower()
        if encoding and encoding not in KNOWN_ENCODINGS and encoding not in self._warned_encodings:
            self.get_logger().warning(f"Unknown image encoding '{msg.encoding}', attempting conversion")
            self._warned_encodings.add(encoding)

        try:
            if encoding in {"mono16", "16uc1", "32fc1"}:
                raw = self.bridge.imgmsg_to_cv2(msg)
                return normalize_to_bgr(raw)
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            if encoding not in self._warned_encodings:
                self.get_logger().warning(f"Failed to convert image ({msg.encoding}): {exc}")
                self._warned_encodings.add(encoding)
            return None

    def marker_callback(self, msg: MarkerArray) -> None:
        stamp = 0.0
        if msg.markers:
            m0 = msg.markers[0]
            stamp = m0.header.stamp.sec + m0.header.stamp.nanosec * 1e-9

        markers = []
        for marker in msg.markers:
            if marker.action != marker.ADD:
                continue
            yaw = yaw_from_quat(marker.pose.orientation)
            classification = class_hint(marker.color.r, marker.color.g, marker.color.b)
            entry = {
                "id": marker.id,
                "ns": marker.ns,
                "x": marker.pose.position.x,
                "y": marker.pose.position.y,
                "z": marker.pose.position.z,
                "size_x": marker.scale.x,
                "size_y": marker.scale.y,
                "size_z": marker.scale.z,
                "yaw": yaw,
                "color_r": marker.color.r,
                "color_g": marker.color.g,
                "color_b": marker.color.b,
                "color_a": marker.color.a,
                "class_hint": classification,
                "distance": math.hypot(marker.pose.position.x, marker.pose.position.y),
            }
            markers.append(entry)

        self.state.update_markers(markers, stamp)


def encode_jpeg(image: np.ndarray, quality: int) -> Optional[bytes]:
    if cv2 is None:
        return None
    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ok, encoded = cv2.imencode(".jpg", image, params)
    if not ok:
        return None
    return encoded.tobytes()


def normalize_to_bgr(image: np.ndarray) -> np.ndarray:
    if cv2 is None:
        return image
    if image.dtype == np.uint16:
        image = (image / 256).astype(np.uint8)
    elif image.dtype != np.uint8:
        image = cv2.convertScaleAbs(image)

    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def class_hint(r: float, g: float, b: float) -> str:
    if r > 0.8 and g < 0.3:
        return "person"
    if g > 0.8 and r < 0.3:
        return "car"
    return "other"


def create_placeholder(width: int = 960, height: int = 540) -> Optional[bytes]:
    if cv2 is None:
        return None
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (30, 34, 40)
    cv2.putText(
        image,
        "Waiting for image stream...",
        (40, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (220, 220, 220),
        2,
        cv2.LINE_AA,
    )
    return encode_jpeg(image, DEFAULT_JPEG_QUALITY)


def find_default_config() -> Optional[str]:
    candidates: List[pathlib.Path] = []
    env_path = os.getenv("FUSION_CPP_PATH")
    if env_path:
        candidates.append(pathlib.Path(env_path))

    script_dir = pathlib.Path(__file__).resolve().parent
    candidates.append(script_dir)
    candidates.extend(script_dir.parents)
    candidates.append(pathlib.Path.cwd())

    seen = set()
    for base in candidates:
        if base in seen:
            continue
        seen.add(base)
        config_path = base / "config" / "fusion_config.yaml"
        if config_path.exists():
            return str(config_path)
    return None


def load_config(path: Optional[str]) -> Dict:
    if not path or yaml is None:
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def parse_map_range(config: Dict) -> Dict[str, float]:
    result = dict(DEFAULT_MAP_RANGE)
    node = config.get("perception_range") or {}
    for key in result:
        if key in node:
            try:
                result[key] = float(node[key])
            except Exception:
                continue
    return result


def parse_cameras(config: Dict) -> List[str]:
    cameras = []
    for entry in config.get("sensors", {}).get("cameras", []) or []:
        name = entry.get("name")
        if not name:
            continue
        if entry.get("enabled") is False:
            continue
        cameras.append(name)
    return cameras


def resolve_image_topic(config: Dict, fallback: str) -> str:
    try:
        cameras = config.get("sensors", {}).get("cameras", []) or []
    except Exception:
        cameras = []

    for entry in cameras:
        if entry.get("enabled") is False:
            continue
        topic = entry.get("topic")
        if topic:
            return str(topic)

    return resolve_topic(config, "viz_topic", fallback)


def resolve_topic(config: Dict, key: str, fallback: str) -> str:
    try:
        return str(config.get("publisher", {}).get(key) or fallback)
    except Exception:
        return fallback


def build_app(state: SharedState) -> Flask:
    app = Flask(__name__, static_folder="static", template_folder="templates")

    @app.route("/")
    def index() -> str:
        return render_template("index.html")

    @app.route("/api/status")
    def api_status():
        camera = request.args.get("camera")
        now = time.time()
        with state.lock:
            cameras = state.get_cameras()
            if not camera:
                camera = state.default_camera
            image_meta = state.image_meta.get(camera or "") if camera else None
            image_stamp = image_meta.get("stamp") if image_meta else None
            image_age = (now - image_meta.get("stamp")) if image_meta and image_meta.get("stamp") else None
            image_fps = compute_fps(state.image_times)
            marker_age = (now - state.last_marker_stamp) if state.last_marker_stamp else None
            marker_fps = compute_fps(state.marker_times)
            marker_count = len(state.markers)

        return jsonify({
            "ok": True,
            "now": now,
            "cameras": cameras,
            "default_camera": state.default_camera,
            "image": {
                "camera": camera,
                "available": bool(image_meta),
                "stamp": image_stamp,
                "age": image_age,
                "fps": image_fps,
                "width": image_meta.get("width") if image_meta else None,
                "height": image_meta.get("height") if image_meta else None,
            },
            "markers": {
                "count": marker_count,
                "stamp": state.last_marker_stamp,
                "age": marker_age,
                "fps": marker_fps,
            },
        })

    @app.route("/api/markers")
    def api_markers():
        with state.lock:
            markers = list(state.markers)
            seq = state.marker_seq
            stamp = state.last_marker_stamp
            map_range = dict(state.map_range)
        return jsonify({
            "seq": seq,
            "stamp": stamp,
            "markers": markers,
            "range": map_range,
        })

    @app.route("/api/cameras")
    def api_cameras():
        return jsonify({"cameras": state.get_cameras(), "default": state.default_camera})

    @app.route("/stream")
    @app.route("/stream/<camera_id>")
    def stream(camera_id: Optional[str] = None):
        return Response(
            mjpeg_generator(state, camera_id),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def compute_fps(samples: collections.deque) -> float:
    if len(samples) < 2:
        return 0.0
    duration = samples[-1] - samples[0]
    if duration <= 0:
        return 0.0
    return float(len(samples) - 1) / duration


def mjpeg_generator(state: SharedState, camera_id: Optional[str]):
    last_seq = -1
    last_emit = 0.0
    while True:
        with state.cond:
            if state.image_seq == last_seq:
                state.cond.wait(timeout=1.0)
            image_seq = state.image_seq
            camera = camera_id or state.default_camera
            payload = state.images.get(camera or "")
            if payload is None and state.default_camera:
                payload = state.images.get(state.default_camera)
            if payload is None and state.placeholder is not None:
                payload = state.placeholder

        if payload is None:
            time.sleep(0.1)
            continue

        if state.max_fps > 0:
            now = time.time()
            wait = (1.0 / state.max_fps) - (now - last_emit)
            if wait > 0:
                time.sleep(wait)
            last_emit = time.time()

        header = (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            + f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii")
        )
        yield header + payload + b"\r\n"
        last_seq = image_seq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fusion perception web visualizer")
    parser.add_argument("--config", default=None, help="Path to fusion_config.yaml")
    parser.add_argument("--image-topic", default=None, help="Image topic (sensor_msgs/Image)")
    parser.add_argument("--marker-topic", default=None, help="Marker topic (MarkerArray)")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Web server host")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Web server port")
    parser.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY, help="JPEG quality 1-100")
    parser.add_argument("--max-fps", type=float, default=DEFAULT_MAX_FPS, help="Max MJPEG FPS")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = args.config or find_default_config()
    config = load_config(config_path)
    map_range = parse_map_range(config)
    cameras = parse_cameras(config)

    image_topic = args.image_topic or resolve_image_topic(config, DEFAULT_IMAGE_TOPIC)
    marker_topic = args.marker_topic or resolve_topic(config, "topic", DEFAULT_MARKER_TOPIC)

    state = SharedState(map_range, cameras)
    state.max_fps = args.max_fps
    state.placeholder = create_placeholder()

    rclpy.init()
    node = WebVisualizerNode(state, image_topic, marker_topic, args.jpeg_quality, args.max_fps)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    app = build_app(state)
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
