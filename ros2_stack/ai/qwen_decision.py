"""qwen_decision.py — Qwen2.5-VL high-level decision sidecar (CPU via Ollama).

Runs in conda python, talks to the stack over rosbridge (ws://localhost:9090) and to
Qwen2.5-VL via the local Ollama HTTP API (http://localhost:11434). Every few seconds
it grabs the onboard camera frame, asks Qwen what a rider should do, and publishes a
high-level decision. The roam_controller uses it to BIAS which way it turns at the
next intersection — strategic, not real-time (Qwen is slow on CPU).

Flow:
  subscribe /moto/onboard_camera/image_raw/compressed   (latest frame)
  every PERIOD s → POST frame+prompt to Ollama qwen2.5vl (CPU)
  parse one of: straight | left | right
  publish  /moto/decision   std_msgs/String

Needs:  `ollama serve` running and the model pulled (`ollama pull qwen2.5vl`).
If Ollama is unreachable it publishes "straight" and logs a warning (graceful).
"""

from __future__ import annotations

import base64
import json
import os
import time
import urllib.request

ROSBRIDGE = os.environ.get("ROSBRIDGE_URL", "ws://localhost:9090")
OLLAMA = os.environ.get("OLLAMA_URL", "http://localhost:11434")
MODEL = os.environ.get("QWEN_MODEL", "qwen2.5vl:7b")
PERIOD = float(os.environ.get("QWEN_PERIOD", "8"))   # seconds between decisions
# NOTE: 7B VL on CPU is slow (~20-60 s/answer). PERIOD is a floor; the node asks
# again only after the previous answer returns. Strategic layer only — not real-time.

PROMPT = (
    "You are the navigation brain of a motorcycle riding through a city, seeing this "
    "onboard camera image. Choose the best way to continue along the road and avoid "
    "obstacles/buildings ahead. Answer with EXACTLY ONE word: straight, left, or right."
)


def ask_qwen(jpg_b64: str) -> str | None:
    body = json.dumps({
        "model": MODEL,
        "prompt": PROMPT,
        "images": [jpg_b64],
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 8},
    }).encode()
    try:
        req = urllib.request.Request(OLLAMA + "/api/generate", data=body,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            out = json.loads(r.read()).get("response", "").strip().lower()
        for word in ("left", "right", "straight"):
            if word in out:
                return word
        return "straight"
    except Exception as e:
        print(f"[qwen] Ollama unreachable ({e}) — is `ollama serve` running? defaulting straight",
              flush=True)
        return None


def main():
    from websocket import create_connection

    ws = create_connection(ROSBRIDGE, timeout=10)
    ws.send(json.dumps({"op": "advertise", "topic": "/moto/decision", "type": "std_msgs/String"}))
    ws.send(json.dumps({"op": "subscribe", "topic": "/moto/onboard_camera/image_raw/compressed",
                        "type": "sensor_msgs/CompressedImage", "throttle_rate": 1000}))
    print(f"[qwen] connected {ROSBRIDGE}; model={MODEL} via {OLLAMA}; decision every {PERIOD}s",
          flush=True)

    latest_jpg = None
    next_decision = time.time() + PERIOD
    ws.settimeout(1.0)
    while True:
        # keep the latest frame
        try:
            raw = ws.recv()
            m = json.loads(raw)
            if m.get("op") == "publish" and m.get("msg", {}).get("data"):
                latest_jpg = m["msg"]["data"]
        except Exception:
            pass

        if time.time() >= next_decision:
            next_decision = time.time() + PERIOD
            if latest_jpg:
                t0 = time.time()
                decision = ask_qwen(latest_jpg)
                if decision:
                    print(f"[qwen] decision: {decision}  ({time.time()-t0:.1f}s)", flush=True)
                    try:
                        ws.send(json.dumps({"op": "publish", "topic": "/moto/decision",
                                            "msg": {"data": decision}}))
                    except Exception:
                        pass


if __name__ == "__main__":
    main()
