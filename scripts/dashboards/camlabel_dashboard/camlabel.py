#!/usr/bin/env python3
"""
RACECAR Neo camera label dashboard: collect frames, then label them, YOLO out.

Two-stage workflow on port 8085 (stdlib HTTP + rclpy, wallfollow pattern):
  Collect: Snap saves the current camera frame to unlabeled/.
  Label:   pick an unlabeled image, drag boxes, save. The file moves to
           images/ with a YOLO label file in labels/ (class_id cx cy w h,
           normalized) and classes.txt stays in sync.

Everything is written under datasets/camlabel in the data root, created at
startup when absent.

camlabel.yaml sets the live-preview resolution sent to the browser. Saved
images are always the full camera frame. Thumbnails, immutable file names
with long browser caching, and one combined /info endpoint keep the
many-cars-one-room WiFi load low.
"""

import json
from pathlib import Path
import re
import sys
import threading
from typing import Any

import cv2
import numpy as np
from racecar_neo_ros2_driver import apps, data_root, imaging
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

PORT = apps.port('camlabel')
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import webui  # noqa: E402, I100

# Dataset lives under the data root (data_root.py), outside the code tree, so
# reinstalling or re-cloning never touches collected frames or labels.
DATASET = data_root.current()[0] / 'datasets' / 'camlabel'
UNLABELED = DATASET / 'unlabeled'
IMAGES = DATASET / 'images'
LABELS = DATASET / 'labels'

CACHE_FOREVER = 'max-age=31536000, immutable'

_lock = threading.Lock()  # latest frame
_fs_lock = threading.Lock()  # file numbering and moves
_live = b''
_preview = b''
_cam_res = [0, 0]
_thumbs: dict[str, bytes] = {}

_cfg: dict[str, Any] = {'preview_width': 320, 'preview_quality': 45}
try:
    _cfg.update(yaml.safe_load((BASE / 'camlabel.yaml').read_text()) or {})
except OSError:
    pass


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__('camlabel')
        self.create_subscription(Image, '/camera/color', self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Image) -> None:
        global _live, _preview, _cam_res
        arr = imaging.bgr(msg)
        if arr is None:
            return
        # A JPEG frame is saved as it came; a raw one is encoded once.
        data = bytes(msg.data) if msg.encoding == 'jpeg' else imaging.jpeg(arr, 95)
        h, w = arr.shape[:2]
        res = [w, h]
        prev = imaging.jpeg(
            imaging.fit_width(arr, int(_cfg['preview_width'])), _cfg['preview_quality']
        )
        with _lock:
            _live = data
            _preview = prev
            _cam_res = res


def _num(name: str) -> int:
    """u7.jpg or img7.jpg -> 7."""
    m = re.search(r'(\d+)\.jpg$', name)
    return int(m.group(1)) if m else 0


def _nums(folder: Path, pattern: str) -> list[int]:
    return sorted(_num(f.name) for f in folder.glob(pattern)) if folder.exists() else []


def _entries(folder: Path, pattern: str) -> list[list[int]]:
    """
    Return [[number, mtime_ns], ...] for the files matching pattern.

    The browser can cache files forever yet never show a stale image if a
    dataset is wiped and numbers restart.
    """
    if not folder.exists():
        return []
    return sorted([_num(f.name), f.stat().st_mtime_ns] for f in folder.glob(pattern))


def _next_unlabeled_number() -> int:
    """
    Return the next unlabeled number; names are never reused.

    Unique names let the browser cache every image and thumbnail forever.
    """
    f = DATASET / 'ucounter'
    n = (int(f.read_text()) if f.exists() else 0) + 1
    f.write_text(str(n))
    return n


def _class_id(name: str) -> int:
    """Index of name in classes.txt, appending it if new."""
    path = DATASET / 'classes.txt'
    classes = path.read_text().splitlines() if path.exists() else []
    if name not in classes:
        classes.append(name)
        path.write_text('\n'.join(classes) + '\n')
    return classes.index(name)


def _thumb(path: Path) -> bytes:
    """
    Return a 160-wide JPEG thumbnail, cached in memory.

    Safe because file names are never reused within a dataset.
    """
    key = f'{path}:{path.stat().st_mtime_ns}'
    t = _thumbs.get(key)
    if t is None:
        arr = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return b''
        h, w = arr.shape[:2]
        small = cv2.resize(arr, (160, max(1, round(160 * h / w))))
        t = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()
        if len(_thumbs) > 600:
            _thumbs.clear()
        _thumbs[key] = t
    return t


class Handler(webui.AppHandler):
    def _query_n(self) -> int:
        return int(self.path.split('n=')[1].split('&')[0])

    def do_GET(self) -> None:
        if self.serve_asset():
            return
        if self.path == '/':
            self.page(BASE / 'camlabel.html')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self.send(200, data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/info':
            with _lock:
                res = list(_cam_res)
            body = json.dumps(
                {
                    'cam': res,
                    'preview_width': int(_cfg['preview_width']),
                    'unlabeled': _entries(UNLABELED, 'u*.jpg'),
                    'labeled': _entries(IMAGES, 'img*.jpg'),
                }
            )
            self.send(200, body.encode(), 'application/json')
        elif self.path.startswith('/uimg?'):
            self._send_file(UNLABELED / f'u{self._query_n()}.jpg')
        elif self.path.startswith('/img?'):
            self._send_file(IMAGES / f'img{self._query_n()}.jpg')
        elif self.path.startswith('/thumb?'):
            pool, stem = (UNLABELED, 'u') if 'src=u' in self.path else (IMAGES, 'img')
            f = pool / f'{stem}{self._query_n()}.jpg'
            data = _thumb(f) if f.is_file() else b''
            if data:
                self.send(200, data, 'image/jpeg', cache=CACHE_FOREVER)
            else:
                self.send_error(404)
        elif self.path.startswith('/label?'):
            f = LABELS / f'img{self._query_n()}.txt'
            path = DATASET / 'classes.txt'
            classes = path.read_text().splitlines() if path.exists() else []
            boxes = []
            if f.is_file():
                for line in f.read_text().splitlines():
                    i, cx, cy, w, h = line.split()
                    name = classes[int(i)] if int(i) < len(classes) else i
                    boxes.append(
                        {
                            'cls': name,
                            'cx': float(cx),
                            'cy': float(cy),
                            'w': float(w),
                            'h': float(h),
                        }
                    )
            self.send(200, json.dumps({'boxes': boxes}).encode(), 'application/json')
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        if self.path == '/snap':
            with _lock:
                data = _live
            if not data:
                self.send_error(503)
                return
            with _fs_lock:
                _ensure_dataset()
                n = _next_unlabeled_number()
                (UNLABELED / f'u{n}.jpg').write_bytes(data)
            self.send(200, json.dumps({'n': n}).encode(), 'application/json')
        elif self.path == '/label':
            req = self.read_json()
            src = UNLABELED / f'u{int(req["n"])}.jpg'
            if not src.is_file():
                self.send_error(404)
                return
            with _fs_lock:
                _ensure_dataset()
                m = max(_nums(IMAGES, 'img*.jpg'), default=0) + 1
                src.rename(IMAGES / f'img{m}.jpg')
                lines = [
                    f'{_class_id(b["cls"])} '
                    f'{b["cx"]:.6f} {b["cy"]:.6f} {b["w"]:.6f} {b["h"]:.6f}'
                    for b in req['boxes']
                ]
                (LABELS / f'img{m}.txt').write_text('\n'.join(lines) + '\n' if lines else '')
            self.send(200, json.dumps({'img': m}).encode(), 'application/json')
        elif self.path == '/dell':
            req = self.read_json()
            n = int(req['n'])
            for f in (IMAGES / f'img{n}.jpg', LABELS / f'img{n}.txt'):
                if f.is_file():
                    f.unlink()
            self.send(200, b'ok', 'text/plain')
        elif self.path == '/delu':
            req = self.read_json()
            f = UNLABELED / f'u{int(req["n"])}.jpg'
            if f.is_file():
                f.unlink()
            self.send(200, b'ok', 'text/plain')
        else:
            self.send_error(404)

    def _send_file(self, f: Path) -> None:
        if f.is_file():
            self.send(200, f.read_bytes(), 'image/jpeg', cache=CACHE_FOREVER)
        else:
            self.send_error(404)


def _ensure_dataset() -> None:
    for d in (UNLABELED, IMAGES, LABELS):
        d.mkdir(parents=True, exist_ok=True)


def main() -> None:
    _ensure_dataset()
    rclpy.init()
    node = CameraNode()
    print(
        f'Camera label dashboard on http://0.0.0.0:{PORT} '
        f'(preview {_cfg["preview_width"]}px, dataset {DATASET})'
    )
    webui.serve(PORT, Handler, spin=webui.ros_spinner(node), cleanup=webui.ros_shutdown)


if __name__ == '__main__':
    main()
