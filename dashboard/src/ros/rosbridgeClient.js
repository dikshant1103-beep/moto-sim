/**
 * rosbridgeClient.js — minimal rosbridge v2 client over WebSocket (no dependency).
 *
 * Talks the rosbridge_suite JSON protocol directly, so the dashboard can render
 * live ROS 2 topics (SLAM map, lidar, camera, multi-vehicle odom) without roslib.
 * Requires `rosbridge_server` running:  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
 *
 * Protocol used:
 *   → {"op":"subscribe","topic":"/map","type":"nav_msgs/msg/OccupancyGrid"}
 *   ← {"op":"publish","topic":"/map","msg":{...}}
 *   → {"op":"unsubscribe","topic":"/map"}
 */

export class RosBridge {
  constructor(url = 'ws://localhost:9090') {
    this.url = url
    this.ws = null
    this.subs = new Map()        // topic -> { type, cbs:Set }
    this.statusCbs = new Set()
    this.status = 'disconnected'
    this._retry = null
  }

  onStatus(cb) { this.statusCbs.add(cb); cb(this.status); return () => this.statusCbs.delete(cb) }
  _setStatus(s) { this.status = s; this.statusCbs.forEach(cb => cb(s)) }

  connect() {
    if (this.ws && (this.ws.readyState === 0 || this.ws.readyState === 1)) return
    this._setStatus('connecting')
    let ws
    try { ws = new WebSocket(this.url) } catch { this._scheduleRetry(); return }
    this.ws = ws

    ws.onopen = () => {
      this._setStatus('connected')
      // (re)subscribe everything
      for (const [topic, { type }] of this.subs) this._send({ op: 'subscribe', topic, type })
    }
    ws.onclose = () => { this._setStatus('disconnected'); this._scheduleRetry() }
    ws.onerror = () => { /* close handler drives retry */ }
    ws.onmessage = (ev) => {
      let m
      try { m = JSON.parse(ev.data) } catch { return }
      if (m.op === 'publish' && this.subs.has(m.topic)) {
        this.subs.get(m.topic).cbs.forEach(cb => { try { cb(m.msg) } catch { /* cb error */ } })
      }
    }
  }

  _scheduleRetry() {
    if (this._retry) return
    this._retry = setTimeout(() => { this._retry = null; this.connect() }, 2000)
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(obj))
  }

  subscribe(topic, type, cb) {
    let entry = this.subs.get(topic)
    if (!entry) {
      entry = { type, cbs: new Set() }
      this.subs.set(topic, entry)
      this._send({ op: 'subscribe', topic, type })
    }
    entry.cbs.add(cb)
    return () => {
      const e = this.subs.get(topic)
      if (!e) return
      e.cbs.delete(cb)
      if (e.cbs.size === 0) { this._send({ op: 'unsubscribe', topic }); this.subs.delete(topic) }
    }
  }

  // advertise + publish a message (for ride controls → path_driver)
  publish(topic, type, msg) {
    if (!this._advertised) this._advertised = new Set()
    if (!this._advertised.has(topic)) {
      this._send({ op: 'advertise', topic, type })
      this._advertised.add(topic)
    }
    this._send({ op: 'publish', topic, msg })
  }

  close() {
    if (this._retry) { clearTimeout(this._retry); this._retry = null }
    if (this.ws) { try { this.ws.close() } catch { /* ignore */ } this.ws = null }
  }
}
