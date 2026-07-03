"""
ws_server.py — Non-blocking WebSocket telemetry server.

Fixed for websockets 14+ / 16  (asyncio API):
  - _broadcaster always yields (await asyncio.sleep(0) when no clients)
  - send() exceptions printed with type for diagnostics
  - Full lifecycle logging for connection events

Bidirectional:
  sim  → browser : telemetry frames  (JSON, ~50 Hz)
  browser → sim  : control commands

Supported commands from browser:
  {"cmd": "pause"}
  {"cmd": "resume"}
  {"cmd": "reset"}
  {"cmd": "set", "key": "torque", "value": 50.0}
  {"cmd": "set", "key": "vx",    "value": 5.0}
  {"cmd": "set", "key": "vy",    "value": 0.3}
"""

import asyncio
import json
import sys
import threading
import queue

try:
    import websockets
    _WS_OK = True
except ImportError:
    _WS_OK = False
    print("[ws_server] 'websockets' not installed — dashboard disabled.", file=sys.stderr)

# ── Shared state ──────────────────────────────────────────────────────────────
_frame_q: queue.Queue = queue.Queue(maxsize=120)
_cmd_q:   queue.Queue = queue.Queue(maxsize=20)
_clients: set         = set()
_paused:  bool        = False
_loop:    asyncio.AbstractEventLoop | None = None

_params: dict = {
    "torque": 0.0,   # drive torque [Nm]  (+ve = traction, −ve = braking)
    "vx":     5.0,
    "vy":     0.0,
}
_reset_flag: bool = False

# ── Diagnostic counters ───────────────────────────────────────────────────────
_frames_pushed   = 0
_frames_sent     = 0
_frames_dropped  = 0


# ── Public API ────────────────────────────────────────────────────────────────

def start(host: str = "localhost", port: int = 8765) -> None:
    if not _WS_OK:
        return
    global _loop
    _loop = asyncio.new_event_loop()
    t = threading.Thread(
        target=_loop.run_forever,          # run_forever — not run_until_complete
        daemon=True,
        name="ws-server",
    )
    t.start()
    asyncio.run_coroutine_threadsafe(_serve(host, port), _loop)


def push_frame(frame: dict) -> None:
    global _frames_pushed, _frames_dropped
    if not _WS_OK:
        return
    _frames_pushed += 1
    try:
        _frame_q.put_nowait(frame)
    except queue.Full:
        _frames_dropped += 1
        try:
            _frame_q.get_nowait()
        except queue.Empty:
            pass
        try:
            _frame_q.put_nowait(frame)
        except queue.Full:
            pass


def is_paused() -> bool:
    global _paused
    _drain_commands()
    return _paused


def get_params() -> dict:
    _drain_commands()
    return dict(_params)


def is_reset_requested() -> bool:
    global _reset_flag
    _drain_commands()
    if _reset_flag:
        _reset_flag = False
        return True
    return False


def init_params(defaults: dict) -> None:
    """Register param keys and their default values.  Must be called before start().
    Only registered keys can be updated by browser 'set' commands."""
    global _params
    _params.update(defaults)


def stats() -> dict:
    return {
        "pushed":  _frames_pushed,
        "sent":    _frames_sent,
        "dropped": _frames_dropped,
        "clients": len(_clients),
        "q_size":  _frame_q.qsize(),
    }


# ── Internal command drain ────────────────────────────────────────────────────

def _drain_commands():
    global _paused, _reset_flag, _params
    while True:
        try:
            cmd = _cmd_q.get_nowait()
        except queue.Empty:
            break
        action = cmd.get("cmd", "")
        if action == "pause":
            _paused = True
        elif action == "resume":
            _paused = False
        elif action == "reset":
            _reset_flag = True
        elif action == "set":
            key = cmd.get("key")
            val = cmd.get("value")
            if key in _params and val is not None:
                _params[key] = _coerce_like_existing(_params.get(key), val)
        elif action == "set_many":
            values = cmd.get("values", {})
            if isinstance(values, dict):
                for key, val in values.items():
                    if key in _params and val is not None:
                        _params[key] = _coerce_like_existing(_params.get(key), val)


def _coerce_like_existing(existing, value):
    if isinstance(existing, str):
        return str(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return existing


# ── Async internals ───────────────────────────────────────────────────────────

async def _handler(websocket) -> None:
    addr = getattr(websocket, 'remote_address', '?')
    print(f"[ws_server] client CONNECTED  {addr}  (total: {len(_clients)+1})",
          flush=True)
    _clients.add(websocket)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
                if isinstance(msg, dict):
                    try:
                        _cmd_q.put_nowait(msg)
                    except queue.Full:
                        pass
            except (json.JSONDecodeError, TypeError):
                pass
    except Exception as e:
        print(f"[ws_server] handler exception: {type(e).__name__}: {e}", flush=True)
    finally:
        _clients.discard(websocket)
        print(f"[ws_server] client DISCONNECTED {addr}  (remaining: {len(_clients)})",
              flush=True)


async def _broadcaster() -> None:
    global _frames_sent
    _diag_interval = 200   # print stats every N frames sent
    _diag_next     = _diag_interval

    while True:
        # ── Always yield to event loop — critical fix for websockets 14+ ──────
        # Without this, a full queue with no clients would spin without awaiting,
        # starving the event loop and preventing new connections from being accepted.
        try:
            frame = _frame_q.get_nowait()
        except queue.Empty:
            await asyncio.sleep(0.005)
            continue

        if not _clients:
            # Yield even when there are no clients — lets _handler tasks run
            await asyncio.sleep(0)
            continue

        msg  = json.dumps(frame)
        dead = set()

        for c in list(_clients):
            try:
                await c.send(msg)
                _frames_sent += 1
            except Exception as exc:
                # Print exception type — reveals websockets API mismatches
                print(f"[ws_server] send() failed: {type(exc).__name__}: {exc}",
                      flush=True)
                dead.add(c)

        if dead:
            _clients.difference_update(dead)

        # Periodic diagnostic
        if _frames_sent >= _diag_next:
            s = stats()
            print(f"[ws_server] stats: pushed={s['pushed']}  sent={s['sent']}  "
                  f"dropped={s['dropped']}  clients={s['clients']}  "
                  f"q={s['q_size']}",
                  flush=True)
            _diag_next += _diag_interval


async def _serve(host: str, port: int) -> None:
    async with websockets.serve(_handler, host, port):
        print(f"[ws_server] Listening on ws://{host}:{port}  "
              f"(websockets {websockets.__version__})",
              flush=True)
        await _broadcaster()
