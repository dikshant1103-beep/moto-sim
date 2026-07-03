"""yolo_node.py — YOLOv11 obstacle perception sidecar (runs in CONDA python on GPU).

Runs OUTSIDE rclpy (ultralytics+torch live in conda py3.13; rclpy is system py3.10),
talking to the stack over rosbridge (ws://localhost:9090) — the same channel the
dashboard uses. No ROS env needed.

Flow:
  subscribe  /moto/onboard_camera/image_raw/compressed   (onboard camera)
  → YOLOv11 (yolo11n.pt, GPU) detect cars/people/bikes/etc.
  → compute a lateral AVOID bias from close obstacles ahead
  publish    /moto/yolo/avoid   std_msgs/Float64   (+ = steer right, − = steer left)
  publish    /moto/yolo/image   sensor_msgs/CompressedImage (annotated, for dashboard)

The roam_controller adds /moto/yolo/avoid to its steer command → the bike steers
around what YOLO SEES (complements lidar). Real-time, GPU, ~8 Hz.

Run (conda python):  python ros2_stack/ai/yolo_node.py
"""

from __future__ import annotations

import base64
import json
import os
import time

import numpy as np

ROSBRIDGE = os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090")
WEIGHTS = os.environ.get("YOLO_WEIGHTS", "/home/dikshant/Desktop/drone_inspector/yolo11n.pt")
RATE_HZ = float(os.environ.get("YOLO_RATE", "8"))
# COCO classes that count as on-road obstacles
OBSTACLE_CLASSES = {"person", "bicycle", "car", "motorcycle", "bus", "truck", "train"}


def main():
    import cv2
    from ultralytics import YOLO
    from websocket import create_connection  # websocket-client (sync)

    model = YOLO(WEIGHTS)
    try:
        model.to("cuda")
        dev = "cuda"
    except Exception:
        dev = "cpu"
    print(f"[yolo] model loaded on {dev}, connecting {ROSBRIDGE}", flush=True)

    ws = create_connection(ROSBRIDGE, timeout=10)
    ws.send(json.dumps({"op": "advertise", "topic": "/moto/yolo/avoid", "type": "std_msgs/Float64"}))
    ws.send(json.dumps({"op": "advertise", "topic": "/moto/yolo/image",
                        "type": "sensor_msgs/CompressedImage"}))
    ws.send(json.dumps({"op": "subscribe", "topic": "/moto/onboard_camera/image_raw/compressed",
                        "type": "sensor_msgs/CompressedImage", "throttle_rate": int(1000 / RATE_HZ)}))
    print("[yolo] subscribed to camera; detecting…", flush=True)

    last = 0.0
    while True:
        try:
            raw = ws.recv()
        except Exception as e:
            print(f"[yolo] ws error ({e}); reconnecting…", flush=True)
            time.sleep(1.0)
            try:
                ws = create_connection(ROSBRIDGE, timeout=10)
                ws.send(json.dumps({"op": "advertise", "topic": "/moto/yolo/avoid", "type": "std_msgs/Float64"}))
                ws.send(json.dumps({"op": "subscribe", "topic": "/moto/onboard_camera/image_raw/compressed",
                                    "type": "sensor_msgs/CompressedImage", "throttle_rate": int(1000 / RATE_HZ)}))
            except Exception:
                pass
            continue

        try:
            m = json.loads(raw)
        except Exception:
            continue
        if m.get("op") != "publish":
            continue
        now = time.time()
        if now - last < 1.0 / RATE_HZ:
            continue
        last = now

        # decode compressed image (base64 JPEG)
        try:
            buf = base64.b64decode(m["msg"]["data"])
            img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
        except Exception:
            continue

        h, w = img.shape[:2]
        res = model.predict(img, verbose=False, conf=0.35, device=dev)[0]
        avoid = 0.0
        annotated = res.plot()  # BGR with boxes
        for b in res.boxes:
            cls = model.names[int(b.cls)]
            if cls not in OBSTACLE_CLASSES:
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            cx = (x1 + x2) / 2.0
            area_frac = ((x2 - x1) * (y2 - y1)) / (w * h)
            bottom_frac = y2 / h          # closer obstacles sit lower in frame
            # closeness weight: big + low in frame = close ahead
            close = area_frac * bottom_frac
            if close < 0.01:
                continue
            # obstacle left of center → steer right (+); right of center → steer left (−)
            side = (cx - w / 2.0) / (w / 2.0)   # −1 (left) … +1 (right)
            avoid += -np.sign(side) * close * 6.0   # gain → steer away

        avoid = float(np.clip(avoid, -0.15, 0.15))   # bounded steer-bias contribution
        ws.send(json.dumps({"op": "publish", "topic": "/moto/yolo/avoid",
                            "msg": {"data": avoid}}))

        # publish annotated frame (lower quality to save bandwidth)
        ok, jpg = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            ws.send(json.dumps({"op": "publish", "topic": "/moto/yolo/image",
                                "msg": {"format": "jpeg",
                                        "data": base64.b64encode(jpg.tobytes()).decode()}}))


if __name__ == "__main__":
    main()
