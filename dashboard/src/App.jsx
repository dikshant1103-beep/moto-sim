import { useState, useEffect, useRef, useCallback } from 'react'
import { useWebSocket }         from './hooks/useWebSocket.js'
import { useSessionRecorder }   from './hooks/useSessionRecorder.js'
import AllGraphsTab             from './components/AllGraphsTab.jsx'
import ThreeViewport      from './components/ThreeViewport.jsx'
import ForcePanel         from './components/ForcePanel.jsx'
import FrictionCircle     from './components/FrictionCircle.jsx'
import SlipPanel          from './components/SlipPanel.jsx'
import TorquePanel        from './components/TorquePanel.jsx'
import ChassisPanel       from './components/ChassisPanel.jsx'
import LoadPanel          from './components/LoadPanel.jsx'
import StatusBar          from './components/StatusBar.jsx'
import ControlPanel       from './components/ControlPanel.jsx'
import DebugPanel         from './components/DebugPanel.jsx'
import ParameterWorkspace from './components/ParameterWorkspace.jsx'
import AnalysisWorkspace  from './components/AnalysisWorkspace.jsx'
import SuspensionPanel    from './components/SuspensionPanel.jsx'
import ThermalPanel       from './components/ThermalPanel.jsx'
import Phase1Panel        from './components/Phase1Panel.jsx'
import Phase2Panel        from './components/Phase2Panel.jsx'
import CoeffFitterPanel   from './components/CoeffFitterPanel.jsx'
import RosLabPanel        from './components/RosLabPanel.jsx'
import ValidationPanel    from './components/ValidationPanel.jsx'
import ExpandModal        from './components/ExpandModal.jsx'

const WS_URL = 'ws://localhost:8765'
const R2D    = 180 / Math.PI

const DEFAULT_SETUP = {
  wheelbase: 1.37, frontWheelDiameter: 0.598, rearWheelDiameter: 0.636,
  frontTyreWidth: 0.12, rearTyreWidth: 0.18, rakeDegrees: 24, trail: 0.097,
  centreOfGravityHeight: 0.58, totalMass: 189, frontWeightFraction: 0.46,
  engineTorqueMaximum: 80, brakeTorqueMaximum: 250,
  frontSpringRate: 18500, rearSpringRate: 42000, frontDamping: 900, rearDamping: 1800,
  trackLength: 400, cornerRadius: 80, bumpHeight: 0.015, potholeDepth: 0.03,
  frictionCoefficient: 0.9, simulationSpeed: 12, trackMode: 'flat',
  straightLength: 50, cornerBankAngle: 0.15,
}

function fmt(v, d = 1) { return (v === undefined || v === null) ? '—' : Number(v).toFixed(d) }
function deg(v, d = 1) { return (v === undefined || v === null) ? '—' : (v * R2D).toFixed(d) }

function useSessionTimer() {
  const [seconds, setSeconds] = useState(0)
  const startRef = useRef(Date.now())
  useEffect(() => {
    const t = setInterval(() => setSeconds(Math.floor((Date.now() - startRef.current) / 1000)), 1000)
    return () => clearInterval(t)
  }, [])
  const mm = String(Math.floor(seconds / 60)).padStart(2, '0')
  const ss = String(seconds % 60).padStart(2, '0')
  return `${mm}:${ss}`
}

export default function App() {
  const {
    dataRef, latestRef,
    connected, paused, frameRate, totalFrames,
    sendCommand, togglePause, clearBuffer, exportCSV,
  } = useWebSocket(WS_URL)

  const [latest,      setLatest]      = useState(null)
  const [activeTab,   setActiveTab]   = useState('live')
  const [setup,       setSetup]       = useState(DEFAULT_SETUP)
  const [appliedAt,   setAppliedAt]   = useState(null)
  const [expanded,    setExpanded]    = useState(null)
  const [theme,       setTheme]       = useState('dark')
  const [recording,   setRecording]   = useState(false)
  const [lapCount,    setLapCount]    = useState(0)
  const [savedDir,    setSavedDir]    = useState(null)
  const [snapshotMsg, setSnapshotMsg] = useState(null)
  const sessionTimer = useSessionTimer()

  const { start: startRec, stop: stopRec, onFrame: recFrame } =
    useSessionRecorder(dataRef, latestRef)

  // Apply theme to document root
  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme)
  }, [theme])

  // Poll latest frame at 10 Hz for React state + recorder
  useEffect(() => {
    const id = setInterval(() => {
      const f = latestRef.current
      if (f) { setLatest({ ...f }); if (recording) recFrame(f) }
    }, 100)
    return () => clearInterval(id)
  }, [latestRef, recording, recFrame])

  const isCppEngine     = latest?.engine?.startsWith('C++') ?? false
  const isPhase2Engine  = isCppEngine && (latest?.engine?.includes('Phase2') || latest?.engine?.includes('FullMF'))
  const isCoeffFitActive= isCppEngine && (latest?.cf_updates ?? 0) > 0

  // Engine badge variant
  const engineBadgeClass = (() => {
    if (!latest?.engine) return 'python'
    if (latest.engine.includes('PINN'))   return 'pinn'
    if (latest.engine.includes('Phase2') || latest.engine.includes('FullMF')) return 'phase2'
    if (latest.engine.includes('C++'))    return 'cpp'
    return 'python'
  })()

  function updateSetup(key, value) { setSetup(p => ({ ...p, [key]: value })) }

  function applySetupToSimulation() {
    sendCommand({ cmd: 'set_many', values: { ...setup, v0: setup.simulationSpeed } })
    sendCommand({ cmd: 'reset' })
    setAppliedAt(new Date())
    setActiveTab('live')
  }

  const fzRatio = latest
    ? (() => { const t=(latest.f_Fz??0)+(latest.r_Fz??0); return t>0?(latest.f_Fz??0)/t:0.5 })()
    : 0.5

  // Telem strip data — 8 key channels
  const telemCells = latest ? [
    { key: 'SPEED',    val: fmt(latest.vx, 2),                unit: 'm/s',  cls: '' },
    { key: 'LEAN',     val: deg(latest.roll, 1),               unit: 'deg',  cls: Math.abs(latest.roll??0) > 0.5 ? 'alert' : '' },
    { key: 'YAW ω',    val: deg(latest.yaw_rate, 1),           unit: '°/s',  cls: '' },
    { key: 'STEER δ',  val: deg(latest.steer_angle, 2),        unit: 'deg',  cls: '' },
    { key: 'FZ FRONT', val: fmt(latest.f_Fz, 0),              unit: 'N',    cls: fzRatio > 0.65 ? 'warn' : '' },
    { key: 'FZ REAR',  val: fmt(latest.r_Fz, 0),              unit: 'N',    cls: fzRatio < 0.30 ? 'warn' : '' },
    { key: 'Fy FRONT', val: fmt(latest.f_Fy, 0),              unit: 'N',    cls: '' },
    { key: 'ROAD μ',   val: fmt(latest.road_mu_f ?? setup.frictionCoefficient, 2), unit: '',  cls: (latest.road_mu_f??1) < 0.7 ? 'warn' : 'good' },
    { key: 'TRAIL',    val: isCppEngine ? fmt((latest.effective_trail??0)*1000,1) : '—', unit: 'mm', cls: '' },
    { key: 'TIME',     val: fmt(latest.t, 2),                  unit: 's',    cls: '' },
  ] : []

  const panelCells = [
    { cls: 'cell-load',    title: 'Normal Load Transfer',         jsx: <LoadPanel dataRef={dataRef} latestRef={latestRef} /> },
    { cls: 'cell-force',   title: 'Tyre Forces Fx Fy',            jsx: <ForcePanel dataRef={dataRef} /> },
    { cls: 'cell-circle',  title: 'Friction Circle',              jsx: <FrictionCircle dataRef={dataRef} mu={setup.frictionCoefficient ?? 0.9} /> },
    { cls: 'cell-slip',    title: 'Slip Angles & Ratio',          jsx: <SlipPanel dataRef={dataRef} /> },
    { cls: 'cell-torque',  title: 'Aligning Torque Mz',           jsx: <TorquePanel dataRef={dataRef} /> },
    { cls: 'cell-geo',     title: 'Chassis Angles',               jsx: <ChassisPanel dataRef={dataRef} /> },
    { cls: 'cell-susp',    title: 'Suspension Travel',            jsx: <SuspensionPanel dataRef={dataRef} /> },
    { cls: 'cell-thermal', title: 'M5 Tyre Thermal & Wear',       jsx: <ThermalPanel dataRef={dataRef} /> },
    ...(isCppEngine      ? [{ cls: 'cell-phase1', title: 'C++ 13-DOF Engine Diagnostics',        jsx: <Phase1Panel      dataRef={dataRef} latestRef={latestRef} /> }] : []),
    ...(isPhase2Engine   ? [{ cls: 'cell-phase2', title: 'Phase 2 — Contact Patch & LuGre',       jsx: <Phase2Panel      dataRef={dataRef} latestRef={latestRef} /> }] : []),
    ...(isCppEngine      ? [{ cls: 'cell-cf',     title: 'Phase 3.1 — PINN Coefficient Fitter',   jsx: <CoeffFitterPanel dataRef={dataRef} latestRef={latestRef} /> }] : []),
  ]

  return (
    <div className="app">

      {/* ── Top bar ───────────────────────────────────────────────────────── */}
      <div className="app-topbar">
        <div className="topbar-left">
          <div className="brand-mark">
            <div className="brand-icon">🏍</div>
            <div className="brand-text">
              <span className="brand-name">moto_sim</span>
              <span className="brand-sub">Vehicle Dynamics Console</span>
            </div>
          </div>
          <div className="topbar-divider" />
          <div className="session-info">
            <div className="session-chip">
              <span className="chip-label">SESSION</span>
              <span className="chip-value">{sessionTimer}</span>
            </div>
            <div className="session-chip">
              <span className="chip-label">FRAMES</span>
              <span className="chip-value">#{totalFrames}</span>
            </div>
            <div className="session-chip">
              <span className="chip-label">FPS</span>
              <span className="chip-value" style={{ color: frameRate < 20 ? 'var(--red)' : frameRate < 40 ? 'var(--amber)' : 'var(--green)' }}>
                {frameRate}
              </span>
            </div>
            {recording && (
              <div className="session-chip" style={{ borderColor: 'var(--red)', color: 'var(--red)' }}>
                <span className="rec-dot" />REC
              </div>
            )}
          </div>
        </div>

        {/* Hero metrics */}
        <div className="hero-metrics">
          <div className="hero-card">
            <span className="hero-label">Speed</span>
            <span className="hero-value">{fmt(latest?.vx, 2)}</span>
            <span className="hero-unit">m/s</span>
          </div>
          <div className="hero-card">
            <span className="hero-label">Lean</span>
            <span className="hero-value" style={{ color: Math.abs(latest?.roll ?? 0) > 0.5 ? 'var(--red)' : undefined }}>
              {deg(latest?.roll, 1)}
            </span>
            <span className="hero-unit">°</span>
          </div>
          <div className="hero-card">
            <span className="hero-label">Yaw ω</span>
            <span className="hero-value">{deg(latest?.yaw_rate, 2)}</span>
            <span className="hero-unit">°/s</span>
          </div>
          <div className="hero-card">
            <span className="hero-label">Fz F/R</span>
            <span className="hero-value" style={{ color: fzRatio > 0.65 || fzRatio < 0.30 ? 'var(--amber)' : undefined }}>
              {fmt(latest?.f_Fz, 0)}/{fmt(latest?.r_Fz, 0)}
            </span>
            <span className="hero-unit">N</span>
          </div>
        </div>

        {/* Right controls */}
        <div className="topbar-right">
          {/* Connection indicator */}
          <div className="session-chip" style={{ borderColor: connected ? 'var(--green)' : 'var(--red)' }}>
            <span style={{ display:'inline-block', width:7, height:7, borderRadius:'50%',
              background: connected ? 'var(--green)' : 'var(--red)',
              boxShadow: `0 0 6px ${connected ? 'var(--green)' : 'var(--red)'}`,
              marginRight:5 }} />
            <span style={{ fontSize:10, fontWeight:700, color: connected ? 'var(--green)' : 'var(--red)' }}>
              {connected ? 'LIVE' : 'OFFLINE'}
            </span>
          </div>
          {/* Engine badge */}
          {latest?.engine && (
            <span className={`engine-badge ${engineBadgeClass}`}>
              {latest.engine.replace('C++ 13-DOF ', '').replace(' Pacejka','Pkj')}
            </span>
          )}
          {/* Record button */}
          <button
            className={`btn btn-record ${recording ? 'recording' : ''}`}
            onClick={async () => {
              if (recording) {
                setRecording(false)
                const dir = await stopRec()
                if (dir) {
                  setSavedDir(dir)
                  setSnapshotMsg(`Session saved → ${dir.split('/').pop()}`)
                  setTimeout(() => setSnapshotMsg(null), 5000)
                }
              } else {
                setRecording(true)
                clearBuffer()
                setLapCount(0)
                const dir = await startRec()
                setSavedDir(dir)
              }
            }}
            title={recording ? 'Stop recording & save session' : 'Start recording session'}
          >
            {recording ? '⏹ STOP' : '⏺ REC'}
          </button>
          {/* Lap counter */}
          <button
            className="icon-btn"
            title="Mark lap"
            onClick={() => setLapCount(n => n + 1)}
          >🏁</button>
          {lapCount > 0 && (
            <span className="session-chip">
              <span className="chip-label">LAP</span>
              <span className="chip-value">{lapCount}</span>
            </span>
          )}
          {/* Theme toggle */}
          <button
            className={`icon-btn ${theme === 'light' ? 'active' : ''}`}
            onClick={() => setTheme(t => t === 'dark' ? 'light' : 'dark')}
            title="Toggle dark/light mode"
          >
            {theme === 'dark' ? '☀️' : '🌙'}
          </button>
        </div>
      </div>

      {/* ── Tab bar ───────────────────────────────────────────────────────── */}
      <div className="tabs-bar">
        <button
          className={`tab-btn tab-live ${activeTab === 'live' ? 'active' : ''}`}
          onClick={() => setActiveTab('live')}
        >
          <span className="tab-dot" style={{ background: 'var(--cyan)' }} />
          Live Simulation
        </button>
        <button
          className={`tab-btn tab-params ${activeTab === 'inputs' ? 'active' : ''}`}
          onClick={() => setActiveTab('inputs')}
        >
          <span className="tab-dot" style={{ background: 'var(--violet)' }} />
          Parameters
        </button>
        <button
          className={`tab-btn tab-analysis ${activeTab === 'analysis' ? 'active' : ''}`}
          onClick={() => setActiveTab('analysis')}
        >
          <span className="tab-dot" style={{ background: 'var(--amber)' }} />
          Analysis
        </button>
        <button
          className={`tab-btn tab-graphs ${activeTab === 'graphs' ? 'active' : ''}`}
          onClick={() => setActiveTab('graphs')}
        >
          <span className="tab-dot" style={{ background: 'var(--green)' }} />
          All Graphs
        </button>
        <button
          className={`tab-btn tab-ros ${activeTab === 'ros' ? 'active' : ''}`}
          onClick={() => setActiveTab('ros')}
        >
          <span className="tab-dot" style={{ background: 'var(--red)' }} />
          Robotics (ROS 2)
        </button>
        <button
          className={`tab-btn tab-validation ${activeTab === 'validation' ? 'active' : ''}`}
          onClick={() => setActiveTab('validation')}
        >
          <span className="tab-dot" style={{ background: 'var(--green)' }} />
          Validation
        </button>
      </div>

      {/* Snapshot toast */}
      {snapshotMsg && (
        <div style={{
          position:'fixed', bottom:20, right:20, zIndex:999,
          background:'var(--bg-raised)', border:'1px solid var(--green)',
          borderRadius:'var(--radius-lg)', padding:'10px 18px',
          color:'var(--green)', fontSize:12, fontWeight:700,
          boxShadow:'0 8px 24px rgba(0,0,0,0.5)',
          animation:'fadeIn 0.2s ease',
        }}>
          ✓ {snapshotMsg}
        </div>
      )}

      {/* ── Parameter tab ─────────────────────────────────────────────────── */}
      {activeTab === 'inputs' && (
        <ParameterWorkspace
          setup={setup}
          onFieldChange={updateSetup}
          onApply={applySetupToSimulation}
          appliedAt={appliedAt}
        />
      )}

      {/* ── Analysis tab ──────────────────────────────────────────────────── */}
      {activeTab === 'analysis' && <AnalysisWorkspace setup={setup} />}

      {/* ── All Graphs tab ────────────────────────────────────────────────── */}
      {activeTab === 'graphs' && (
        <AllGraphsTab dataRef={dataRef} latestRef={latestRef} />
      )}

      {/* ── Robotics (ROS 2) tab ──────────────────────────────────────────── */}
      {activeTab === 'ros' && <RosLabPanel />}

      {/* ── Validation tab ────────────────────────────────────────────────── */}
      {activeTab === 'validation' && <ValidationPanel />}

      {/* ── Live tab ──────────────────────────────────────────────────────── */}
      {activeTab === 'live' && (
        <>
          {/* Status bar */}
          <StatusBar
            connected={connected} paused={paused} latest={latest}
            engineBadgeClass={engineBadgeClass}
            onTogglePause={togglePause} onClear={clearBuffer} onExport={exportCSV}
          />

          <div className="main-area">
            {/* ── LEFT: 3D + telem strip + controls ─────────────────────── */}
            <div className="panel-3d">
              <div className="panel-header">
                <span className="panel-title">3D Viewport</span>
                <div className="panel-header-right">
                  <span className="fps-badge">{frameRate} fps · #{totalFrames}</span>
                </div>
              </div>

              {/* Numeric telemetry strip */}
              {latest && (
                <div className="telem-strip">
                  {telemCells.map(c => (
                    <div key={c.key} className="telem-cell">
                      <span className="telem-key">{c.key}</span>
                      <span className={`telem-val ${c.cls}`}>{c.val}</span>
                      {c.unit && <span className="telem-unit">{c.unit}</span>}
                    </div>
                  ))}
                </div>
              )}

              <div className="viewport-wrap">
                <ThreeViewport latestRef={latestRef} vehicleSetup={setup} />

                {/* Overlay readout */}
                {latest && (
                  <div className="overlay-readout">
                    {[
                      { label: 'time',       val: `${fmt(latest.t, 3)} s`,                    color: 'var(--text-dim)' },
                      { label: 'speed',      val: `${fmt(latest.vx, 2)} m/s`,                  color: 'var(--cyan)' },
                      { label: 'lean',       val: `${deg(latest.roll, 2)}°`,                   color: 'var(--red)' },
                      { label: 'yaw rate',   val: `${deg(latest.yaw_rate, 3)}°/s`,             color: 'var(--blue)' },
                      { label: 'steer δ',    val: `${deg(latest.steer_angle, 2)}°`,            color: 'var(--amber)' },
                      { label: 'Fz front',   val: `${fmt(latest.f_Fz, 0)} N`,                  color: 'var(--green)' },
                      { label: 'Fz rear',    val: `${fmt(latest.r_Fz, 0)} N`,                  color: 'var(--cyan)' },
                      { label: 'Fz ratio',   val: `${(fzRatio*100).toFixed(1)}% F`,           color: fzRatio>0.65?'var(--red)':fzRatio<0.30?'var(--amber)':'var(--text-mid)' },
                      { label: 'road μ',     val: `${fmt(latest.road_mu_f ?? setup.frictionCoefficient, 2)}`, color: 'var(--violet)' },
                    ].map(({ label, val, color }) => (
                      <div key={label} className="or-row">
                        <span className="or-label" style={{ color }}>{label}</span>
                        <span className="or-value">{val}</span>
                      </div>
                    ))}
                  </div>
                )}

                <DebugPanel
                  latestRef={latestRef} dataRef={dataRef}
                  frameRate={frameRate} totalFrames={totalFrames} connected={connected}
                />
              </div>

              <ControlPanel
                sendCommand={sendCommand} paused={paused}
                onTogglePause={togglePause}
              />
            </div>

            {/* ── RIGHT: chart grid ──────────────────────────────────────── */}
            <div className="panel-charts">
              {panelCells.map(({ cls, title, jsx }) => (
                <div key={cls} className={`cell ${cls}`}>
                  {jsx}
                  <button
                    className="cell-expand-btn"
                    title={`Expand: ${title}`}
                    onClick={() => setExpanded({ title, jsx })}
                  >⤢</button>
                </div>
              ))}
            </div>

            {expanded && (
              <ExpandModal title={expanded.title} onClose={() => setExpanded(null)}>
                {expanded.jsx}
              </ExpandModal>
            )}
          </div>
        </>
      )}
    </div>
  )
}
