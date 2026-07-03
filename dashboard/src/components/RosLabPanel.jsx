/**
 * RosLabPanel.jsx — Robotics (ROS 2) tab: live run + view of the ROS 2 / Gazebo /
 * SLAM / multi-vehicle stack, inside the dashboard.
 *
 * LEFT  — launch control: start/stop each component (Electron spawns
 *         ros2_stack/moto_ros_launch.sh) + a live log tail.
 * RIGHT — live views streamed via rosbridge (ws://localhost:9090):
 *         SLAM map (/map), lidar (/moto/scan), onboard camera
 *         (/moto/camera/image_raw/compressed), and multi-vehicle top-down (/motoK/odom).
 *
 * The robotics stack runs as real ROS 2 processes; this tab is the single control
 * + view surface so you don't need separate Gazebo/rviz2 windows or 4 terminals.
 */
import { useEffect, useRef, useState, useCallback } from 'react'
import { RosBridge } from '../ros/rosbridgeClient.js'

const ros = typeof window !== 'undefined' ? window.rosAPI : undefined

// launch sequence (order matters): rosbridge first (for views), then sim → bridge → world → slam
const COMPONENTS = [
  { id: 'rosbridge', label: 'rosbridge (view link)', desc: 'ws://localhost:9090 for these live views — start FIRST' },
  { id: 'oval',      label: 'Oval race track ride',  desc: 'bounded oval track, bike corners CONTINUOUSLY & stays in view (cannot topple). Best to watch turning.', args: ['false', 'true'] },
  { id: 'world',     label: 'Endless city ride',     desc: 'infinite city + KINEMATIC bike (cannot topple/stick) + camera/lidar. Steered by AI.', args: ['true', 'false', 'true'] },
  { id: 'sim',       label: 'PyChrono sim (Live tab)',desc: 'physics for the Live Simulation tab — OPTIONAL for the city ride' },
  { id: 'bridge',    label: 'ROS 2 bridge (physics)', desc: 'only for physics-roam; SKIP for the kinematic ride (would clash on /moto/odom)' },
  { id: 'slam',      label: 'SLAM (slam_toolbox)',   desc: 'live map from /moto/scan (also feeds Nav2)' },
  { id: 'nav2',      label: 'Nav2 autonomy + roam',  desc: 'car-like planner avoids buildings via lidar; bike roams the city on its own' },
  { id: 'ai',        label: 'AI: YOLO + Qwen',       desc: 'YOLOv11 obstacle avoidance (GPU) + Qwen2.5-VL decisions (CPU/Ollama) feed the roamer' },
  { id: 'multi',     label: 'Multi-vehicle (n=3)',   desc: 'convoy — replaces single world', args: ['3', 'false'] },
]

function StatusDot({ on }) {
  return <span style={{ display: 'inline-block', width: 9, height: 9, borderRadius: '50%',
    background: on ? 'var(--green,#34d399)' : 'var(--red,#f87171)', marginRight: 6 }} />
}

// ── SLAM occupancy-grid renderer ──────────────────────────────────────────
function drawMap(canvas, grid) {
  if (!canvas || !grid || !grid.info) return
  const { width, height } = grid.info
  const data = grid.data
  if (!width || !height || !data) return
  canvas.width = width; canvas.height = height
  const ctx = canvas.getContext('2d')
  const img = ctx.createImageData(width, height)
  for (let i = 0; i < width * height; i++) {
    const v = data[i]
    let c = 128                          // unknown → grey
    if (v === 0) c = 240                 // free → light
    else if (v > 0) c = 255 - Math.min(255, Math.round(v * 2.55))  // occupied → dark
    const row = height - 1 - Math.floor(i / width)   // flip Y (ROS y-up → canvas y-down)
    const col = i % width
    const p = (row * width + col) * 4
    img.data[p] = img.data[p + 1] = img.data[p + 2] = c
    img.data[p + 3] = 255
  }
  ctx.putImageData(img, 0, 0)
}

// ── Lidar polar plot ──────────────────────────────────────────────────────
function drawScan(canvas, scan) {
  if (!canvas) return
  const W = canvas.width, H = canvas.height
  const ctx = canvas.getContext('2d')
  ctx.fillStyle = '#0a1422'; ctx.fillRect(0, 0, W, H)
  const cx = W / 2, cy = H / 2
  ctx.strokeStyle = '#1e3050'
  for (const r of [0.33, 0.66, 1.0]) { ctx.beginPath(); ctx.arc(cx, cy, r * Math.min(cx, cy) * 0.95, 0, 2 * Math.PI); ctx.stroke() }
  if (!scan || !scan.ranges) return
  const maxR = scan.range_max || 60
  const scale = Math.min(cx, cy) * 0.95 / maxR
  ctx.fillStyle = '#34d399'
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = scan.ranges[i]
    if (!isFinite(r) || r <= 0 || r > maxR) continue
    const a = scan.angle_min + i * scan.angle_increment
    // ROS x fwd (up on screen), y left → screen x = -y, screen y = -x
    const x = cx - (r * Math.sin(a)) * scale
    const y = cy - (r * Math.cos(a)) * scale
    ctx.fillRect(x - 1, y - 1, 2, 2)
  }
  ctx.fillStyle = '#f87171'; ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2 * Math.PI); ctx.fill()  // bike
}

// ── Multi-vehicle top-down ────────────────────────────────────────────────
function drawFleet(canvas, poses) {
  if (!canvas) return
  const W = canvas.width, H = canvas.height
  const ctx = canvas.getContext('2d')
  ctx.fillStyle = '#0a1422'; ctx.fillRect(0, 0, W, H)
  const entries = Object.entries(poses)
  if (!entries.length) return
  // autoscale around the centroid
  const xs = entries.map(([, p]) => p.x), ys = entries.map(([, p]) => p.y)
  const cx0 = (Math.min(...xs) + Math.max(...xs)) / 2, cy0 = (Math.min(...ys) + Math.max(...ys)) / 2
  const span = Math.max(20, Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys)) * 1.3
  const sc = Math.min(W, H) / span
  const colors = ['#f87171', '#60a5fa', '#34d399', '#fbbf24', '#c084fc']
  entries.forEach(([name, p], i) => {
    const sx = W / 2 + (p.x - cx0) * sc
    const sy = H / 2 - (p.y - cy0) * sc
    ctx.fillStyle = colors[i % colors.length]
    ctx.beginPath(); ctx.arc(sx, sy, 5, 0, 2 * Math.PI); ctx.fill()
    // heading tick
    ctx.strokeStyle = colors[i % colors.length]; ctx.beginPath(); ctx.moveTo(sx, sy)
    ctx.lineTo(sx + Math.cos(p.yaw) * 12, sy - Math.sin(p.yaw) * 12); ctx.stroke()
    ctx.fillStyle = '#cbd5e1'; ctx.font = '10px monospace'; ctx.fillText(name, sx + 7, sy - 7)
  })
}

export default function RosLabPanel() {
  const [running, setRunning] = useState([])          // component ids alive
  const [log, setLog] = useState([])
  const [bridgeStatus, setBridgeStatus] = useState('disconnected')
  const [fleet, setFleet] = useState({})
  const [camSrc, setCamSrc] = useState(null)
  const [yoloSrc, setYoloSrc] = useState(null)
  const [decision, setDecision] = useState(null)
  const [avoid, setAvoid] = useState(0)
  // ride controls + telemetry
  const [speed, setSpeed] = useState(8)
  const [paused, setPaused] = useState(false)
  const [telem, setTelem] = useState({ v: 0, yaw: 0, x: 0, y: 0 })
  const telemHistRef = useRef([])         // [{t, v, yaw}] rolling
  const graphRef = useRef(null)

  const mapRef = useRef(null), scanRef = useRef(null), fleetRef = useRef(null)
  const rosRef = useRef(null)
  const logBoxRef = useRef(null)

  // ── Electron process control ────────────────────────────────────────────
  const refreshStatus = useCallback(async () => {
    if (ros) setRunning(await ros.status())
  }, [])

  useEffect(() => {
    if (!ros) return
    const offLog = ros.onLog(({ component, text }) =>
      setLog(l => [...l.slice(-300), `[${component}] ${text.replace(/\n$/, '')}`]))
    const offExit = ros.onExit(({ component, code }) => {
      setLog(l => [...l.slice(-300), `[${component}] exited (code ${code})`])
      refreshStatus()
    })
    refreshStatus()
    return () => { offLog(); offExit() }
  }, [refreshStatus])

  useEffect(() => { if (logBoxRef.current) logBoxRef.current.scrollTop = logBoxRef.current.scrollHeight }, [log])

  const launch = async (c) => { if (ros) { await ros.launch(c.id, c.args || []); setTimeout(refreshStatus, 400) } }
  const stop   = async (id) => { if (ros) { await ros.stop(id); setTimeout(refreshStatus, 400) } }
  const stopAll = async () => { if (ros) { await ros.stopAll(); setTimeout(refreshStatus, 600) } }

  // ── rosbridge live views ──────────────────────────────────────────────────
  useEffect(() => {
    const rb = new RosBridge('ws://localhost:9090')
    rosRef.current = rb
    const offStatus = rb.onStatus(setBridgeStatus)
    rb.connect()

    const subs = []
    subs.push(rb.subscribe('/map', 'nav_msgs/msg/OccupancyGrid', (m) => drawMap(mapRef.current, m)))
    subs.push(rb.subscribe('/moto/scan', 'sensor_msgs/msg/LaserScan', (m) => drawScan(scanRef.current, m)))
    // Gazebo's camera publishes under the sensor name 'onboard_camera'.
    // Subscribe to both the single-bike (world) and lead-bike (multi) topics.
    const onCam = (m) => { if (m && m.data) setCamSrc(`data:image/jpeg;base64,${m.data}`) }
    subs.push(rb.subscribe('/moto/onboard_camera/image_raw/compressed', 'sensor_msgs/msg/CompressedImage', onCam))
    subs.push(rb.subscribe('/moto0/onboard_camera/image_raw/compressed', 'sensor_msgs/msg/CompressedImage', onCam))
    // AI: YOLO annotated frame + avoid bias, Qwen decision
    subs.push(rb.subscribe('/moto/yolo/image', 'sensor_msgs/msg/CompressedImage',
      (m) => { if (m && m.data) setYoloSrc(`data:image/jpeg;base64,${m.data}`) }))
    subs.push(rb.subscribe('/moto/yolo/avoid', 'std_msgs/msg/Float64', (m) => setAvoid(m?.data ?? 0)))
    subs.push(rb.subscribe('/moto/decision', 'std_msgs/msg/String', (m) => setDecision(m?.data ?? null)))
    // multi-vehicle odoms (lead + up to 4 ghosts)
    const odomTopics = ['/moto/odom', '/moto1/odom', '/moto2/odom', '/moto3/odom', '/moto4/odom']
    odomTopics.forEach((tp, idx) => {
      const name = idx === 0 ? 'moto0' : `moto${idx}`
      subs.push(rb.subscribe(tp, 'nav_msgs/msg/Odometry', (m) => {
        const p = m.pose.pose.position, q = m.pose.pose.orientation
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        setFleet(f => ({ ...f, [name]: { x: p.x, y: p.y, yaw } }))
        if (idx === 0) {
          const v = m.twist.twist.linear.x, yr = m.twist.twist.angular.z
          setTelem({ v, yaw: yr, x: p.x, y: p.y })
          const h = telemHistRef.current
          h.push({ t: Date.now(), v, yaw: yr })
          if (h.length > 240) h.shift()   // ~last 5 s at 50 Hz throttled
        }
      }))
    })
    return () => { subs.forEach(u => u()); offStatus(); rb.close() }
  }, [])

  useEffect(() => { drawFleet(fleetRef.current, fleet) }, [fleet])

  // ── ride controls → path_driver (over rosbridge) ──────────────────────────
  const pubSpeed = (v) => { setSpeed(v); rosRef.current?.publish('/moto/ride/speed', 'std_msgs/msg/Float64', { data: v }) }
  const togglePauseRide = () => {
    const p = !paused; setPaused(p)
    rosRef.current?.publish('/moto/ride/pause', 'std_msgs/msg/Bool', { data: p })
  }
  const resetRide = () => rosRef.current?.publish('/moto/ride/reset', 'std_msgs/msg/Empty', {})

  // ── telemetry graph (speed + yaw-rate traces) ─────────────────────────────
  useEffect(() => {
    let raf
    const draw = () => {
      raf = requestAnimationFrame(draw)
      const cv = graphRef.current; if (!cv) return
      const W = cv.width, H = cv.height, ctx = cv.getContext('2d')
      ctx.fillStyle = '#0a1422'; ctx.fillRect(0, 0, W, H)
      ctx.strokeStyle = '#1e3050'; ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke()
      const h = telemHistRef.current; if (h.length < 2) return
      const n = h.length
      const trace = (key, color, scale, mid) => {
        ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.beginPath()
        h.forEach((d, i) => {
          const x = (i / (n - 1)) * W
          const y = mid - d[key] * scale
          i ? ctx.lineTo(x, Math.max(2, Math.min(H - 2, y))) : ctx.moveTo(x, y)
        })
        ctx.stroke()
      }
      trace('v', '#38bdf8', H / 2 / 25, H * 0.75)          // speed 0..25 → bottom half
      trace('yaw', '#fbbf24', H / 2 / 0.6, H * 0.25)       // yaw rate ±0.6 → top half
    }
    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [])

  const card = { background: 'var(--bg-raised,#0e1726)', border: '1px solid var(--border,#1e2d44)', borderRadius: 10, padding: 12 }
  const btn = { fontSize: 11, padding: '4px 10px', borderRadius: 6, border: '1px solid var(--border,#26405f)',
    background: 'var(--bg,#0a1422)', color: 'var(--text,#cbd5e1)', cursor: 'pointer' }

  return (
    <div style={{ padding: 16, display: 'grid', gridTemplateColumns: '340px 1fr', gap: 14, height: '100%', overflow: 'hidden' }}>
      {/* ── LEFT: launch control + log ───────────────────────────────────── */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: 12, minHeight: 0 }}>
        <div style={card}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <h3 style={{ margin: 0, fontSize: 14, color: 'var(--text,#e2e8f0)' }}>Stack Control</h3>
            <button onClick={stopAll} style={{ ...btn, borderColor: 'var(--red,#f87171)', color: 'var(--red,#f87171)' }}>Stop all</button>
          </div>
          {!ros && (
            <div style={{ marginTop: 8, fontSize: 11, color: 'var(--amber,#fbbf24)' }}>
              Launch control needs the Electron app (run <code>npm start</code>). In a plain browser the
              live views still work if you start the stack manually.
            </div>
          )}
          <div style={{ marginTop: 10, display: 'flex', flexDirection: 'column', gap: 7 }}>
            {COMPONENTS.map(c => {
              const on = running.includes(c.id)
              return (
                <div key={c.id} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
                  <div style={{ minWidth: 0 }}>
                    <div style={{ fontSize: 12, color: 'var(--text,#e2e8f0)' }}><StatusDot on={on} />{c.label}</div>
                    <div style={{ fontSize: 9, color: 'var(--text-dim,#64748b)' }}>{c.desc}</div>
                  </div>
                  <button onClick={() => on ? stop(c.id) : launch(c)} disabled={!ros}
                    style={{ ...btn, opacity: ros ? 1 : 0.5, borderColor: on ? 'var(--red,#f87171)' : 'var(--green,#34d399)',
                      color: on ? 'var(--red,#f87171)' : 'var(--green,#34d399)' }}>
                    {on ? 'Stop' : 'Start'}
                  </button>
                </div>
              )
            })}
          </div>
          <div style={{ marginTop: 8, fontSize: 9, color: 'var(--text-dim,#64748b)' }}>
            Best ride: rosbridge → Oval race track → slam → ai (bike corners continuously, stays in view). City = endless variant.
          </div>
        </div>

        <div style={{ ...card, flex: 1, display: 'flex', flexDirection: 'column', minHeight: 0 }}>
          <h3 style={{ margin: '0 0 6px', fontSize: 13, color: 'var(--text,#e2e8f0)' }}>Log</h3>
          <div ref={logBoxRef} style={{ flex: 1, overflow: 'auto', fontFamily: 'monospace', fontSize: 10,
            color: 'var(--text-dim,#94a3b8)', background: 'var(--bg,#070e18)', borderRadius: 6, padding: 8, whiteSpace: 'pre-wrap' }}>
            {log.length ? log.join('\n') : 'No output yet — start a component.'}
          </div>
        </div>
      </div>

      {/* ── RIGHT: live views ────────────────────────────────────────────── */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: 10, minHeight: 0 }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
          <StatusDot on={bridgeStatus === 'connected'} />
          <span style={{ fontSize: 12, color: 'var(--text,#e2e8f0)' }}>
            rosbridge: <b>{bridgeStatus}</b> {bridgeStatus !== 'connected' && '— start rosbridge to see live views'}
          </span>
        </div>

        {/* ── Ride controls + live telemetry graph ───────────────────────── */}
        <div style={{ display: 'flex', gap: 10, alignItems: 'stretch' }}>
          <div style={{ ...card, flex: 1, display: 'flex', flexDirection: 'column', gap: 6, padding: 10 }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: 10, flexWrap: 'wrap' }}>
              <span style={{ fontSize: 11, color: 'var(--text-dim,#94a3b8)' }}>Ride controls</span>
              <label style={{ fontSize: 11, color: 'var(--text,#e2e8f0)', display: 'flex', alignItems: 'center', gap: 6 }}>
                Speed
                <input type="range" min={0} max={20} step={0.5} value={speed}
                  onChange={e => pubSpeed(parseFloat(e.target.value))} style={{ width: 120 }} />
                <b style={{ fontFamily: 'monospace', color: 'var(--cyan,#38bdf8)' }}>{speed.toFixed(1)} m/s</b>
              </label>
              <button onClick={togglePauseRide} style={{ ...btn, borderColor: paused ? 'var(--amber,#fbbf24)' : 'var(--border,#26405f)' }}>
                {paused ? '▶ Resume' : '⏸ Pause'}
              </button>
              <button onClick={resetRide} style={btn}>↺ Reset lap</button>
            </div>
            <div style={{ display: 'flex', gap: 16, fontSize: 11, fontFamily: 'monospace', color: 'var(--text,#e2e8f0)' }}>
              <span>v <b style={{ color: 'var(--cyan,#38bdf8)' }}>{telem.v.toFixed(1)}</b> m/s</span>
              <span>yaw <b style={{ color: 'var(--amber,#fbbf24)' }}>{(telem.yaw * 57.3).toFixed(0)}</b>°/s</span>
              <span>pos <b>{telem.x.toFixed(0)},{telem.y.toFixed(0)}</b></span>
            </div>
          </div>
          <div style={{ ...card, width: 260, padding: 8, display: 'flex', flexDirection: 'column' }}>
            <div style={{ fontSize: 10, color: 'var(--text-dim,#94a3b8)', marginBottom: 4 }}>
              <span style={{ color: '#38bdf8' }}>● speed</span> <span style={{ color: '#fbbf24' }}>● yaw rate</span>
            </div>
            <canvas ref={graphRef} width={240} height={70} style={{ width: '100%', height: 70, borderRadius: 4 }} />
          </div>
        </div>

        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gridTemplateRows: '1fr 1fr', gap: 10, flex: 1, minHeight: 0 }}>
          <ViewCard title="SLAM map  /map"><canvas ref={mapRef} style={canvasStyle} /></ViewCard>
          <ViewCard title="Lidar  /moto/scan"><canvas ref={scanRef} width={320} height={320} style={canvasStyle} /></ViewCard>
          <ViewCard title={yoloSrc ? "Onboard + YOLO detections  /moto/yolo/image" : "Onboard camera  /moto/onboard_camera"}>
            <div style={{ position: 'relative', width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
              {(yoloSrc || camSrc)
                ? <img src={yoloSrc || camSrc} alt="onboard" style={canvasStyle} />
                : <Placeholder text="no camera frames (start Endless city; AI view appears once YOLO runs)" />}
              {/* AI overlay: Qwen decision + YOLO avoid bias */}
              {(decision || Math.abs(avoid) > 0.001) && (
                <div style={{ position: 'absolute', left: 6, top: 6, padding: '3px 7px', borderRadius: 4,
                  background: 'rgba(15,23,42,.72)', fontSize: 10, color: '#e2e8f0', fontFamily: 'ui-monospace,monospace' }}>
                  {decision && <span>🧠 Qwen: <b style={{ color: 'var(--cyan,#38bdf8)' }}>{decision}</b></span>}
                  {Math.abs(avoid) > 0.001 && <span style={{ marginLeft: decision ? 8 : 0 }}>
                    👁 YOLO avoid: <b style={{ color: avoid > 0 ? 'var(--amber,#fbbf24)' : 'var(--green,#34d399)' }}>
                    {avoid > 0 ? '→' : '←'} {Math.abs(avoid).toFixed(2)}</b></span>}
                </div>
              )}
            </div>
          </ViewCard>
          <ViewCard title="Multi-vehicle  /motoK/odom"><canvas ref={fleetRef} width={320} height={320} style={canvasStyle} /></ViewCard>
        </div>
      </div>
    </div>
  )
}

const canvasStyle = { width: '100%', height: '100%', objectFit: 'contain', background: '#0a1422', borderRadius: 6, imageRendering: 'pixelated' }

function ViewCard({ title, children }) {
  return (
    <div style={{ background: 'var(--bg-raised,#0e1726)', border: '1px solid var(--border,#1e2d44)', borderRadius: 10,
      padding: 8, display: 'flex', flexDirection: 'column', minHeight: 0 }}>
      <div style={{ fontSize: 11, color: 'var(--text-dim,#94a3b8)', marginBottom: 6 }}>{title}</div>
      <div style={{ flex: 1, minHeight: 0, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>{children}</div>
    </div>
  )
}

function Placeholder({ text }) {
  return <div style={{ fontSize: 10, color: 'var(--text-dim,#64748b)', textAlign: 'center', padding: 12 }}>{text}</div>
}
