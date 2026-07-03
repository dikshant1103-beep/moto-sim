/**
 * DebugPanel — live raw telemetry table + pipeline health indicators.
 *
 * Reads directly from latestRef and dataRef (no React state — no re-renders on data).
 * Polls at 4 Hz via setInterval so it stays cheap.
 * frameRate / totalFrames come from useWebSocket (updated 1 Hz).
 */
import { useEffect, useRef, useState } from 'react'

const R2D = 180 / Math.PI

const KEYS = [
  { k: 't',           label: 't',      unit: 's',     dec: 3, warn: null },
  { k: 'vx',          label: 'vx',     unit: 'm/s',   dec: 2, warn: v => v === 0 ? 'stall?' : null },
  { k: 'roll',        label: 'roll',   unit: '°',     dec: 2, warn: v => Math.abs(v*R2D) > 5 ? '>5°!' : null, scale: R2D },
  { k: 'pitch',       label: 'pitch',  unit: '°',     dec: 2, warn: null, scale: R2D },
  { k: 'yaw_rate',    label: 'ψ̇',      unit: '°/s',   dec: 3, warn: null, scale: R2D },
  { k: 'f_Fz',        label: 'Fz_f',   unit: 'N',     dec: 0, warn: v => v < 50 ? 'low!' : null },
  { k: 'r_Fz',        label: 'Fz_r',   unit: 'N',     dec: 0, warn: v => v < 50 ? 'low!' : null },
  { k: 'f_alpha',     label: 'α_f',    unit: '°',     dec: 2, warn: null, scale: R2D },
  { k: 'r_alpha',     label: 'α_r',    unit: '°',     dec: 2, warn: null, scale: R2D },
  { k: 'f_kappa',     label: 'κ_f',    unit: '',      dec: 4, warn: null },
  { k: 'r_kappa',     label: 'κ_r',    unit: '',      dec: 4, warn: null },
  { k: 'f_Mz',        label: 'Mz_f',   unit: 'N·m',   dec: 3, warn: null },
  { k: 'steer_torque',label: 'τ_steer',unit: 'N·m',   dec: 1, warn: null },
  { k: 'throttle',    label: 'throt',  unit: '',      dec: 2, warn: null },
  { k: 'lean_delta',  label: 'lean',   unit: '°',     dec: 1, warn: null, scale: R2D },
  { k: 'fork_travel', label: 'fork',   unit: 'mm',    dec: 1, warn: null, scale: 1000 },
  { k: 'rear_travel', label: 'rear',   unit: 'mm',    dec: 1, warn: null, scale: 1000 },
  { k: 'road_mu_f',   label: 'μ_road', unit: '',      dec: 3, warn: v => v < 0.5 ? 'slippery' : null },
  { k: 'road_bank_f', label: 'bank',   unit: '°',     dec: 1, warn: null, scale: R2D },
]

export default function DebugPanel({ latestRef, dataRef, frameRate, totalFrames, connected }) {
  const [snap, setSnap] = useState(null)
  const [bufLen, setBufLen] = useState(0)
  const [prevTime, setPrevTime] = useState(null)
  const [evolving, setEvolving] = useState(false)   // true if time is changing

  useEffect(() => {
    const id = setInterval(() => {
      const f = latestRef.current
      if (!f) return
      setSnap({ ...f })
      setBufLen(dataRef.current.length)
      // Confirm data is evolving: time must increase
      setPrevTime(pt => {
        const now = f.t ?? 0
        setEvolving(pt !== null && now !== pt)
        return now
      })
    }, 250)   // 4 Hz poll — cheap
    return () => clearInterval(id)
  }, [latestRef, dataRef])

  // Chart buffer length probe — logs to console at 4 Hz when growing
  useEffect(() => {
    const id = setInterval(() => {
      const len = dataRef.current.length
      if (len > 0 && len % 50 === 0) {
        console.log(`[DebugPanel] buffer length = ${len}`)
      }
    }, 250)
    return () => clearInterval(id)
  }, [dataRef])

  const dot = connected ? '#22c55e' : '#ef4444'
  const evDot = evolving ? '#22c55e' : '#f59e0b'

  return (
    <div className="debug-panel">
      {/* ── Header ─────────────────────────────────────────────────── */}
      <div className="debug-header">
        <span style={{ fontWeight: 700, color: '#0f172a' }}>Debug</span>
        <span className="debug-pill" style={{ background: dot }}>
          {connected ? 'WS OK' : 'WS OFF'}
        </span>
        <span className="debug-pill" style={{ background: evDot }}>
          {evolving ? 'LIVE' : 'STALE'}
        </span>
        <span className="debug-stat">
          {frameRate} fps
        </span>
        <span className="debug-stat">
          #{totalFrames}
        </span>
        <span className="debug-stat">
          buf {bufLen}
        </span>
      </div>

      {/* ── Raw telemetry table ─────────────────────────────────────── */}
      {snap ? (
        <table className="debug-table">
          <tbody>
            {KEYS.map(({ k, label, unit, dec, warn, scale }) => {
              const raw = snap[k] ?? null
              const val = raw !== null ? (raw * (scale ?? 1)) : null
              const w   = warn && raw !== null ? warn(raw) : null
              return (
                <tr key={k} className={w ? 'debug-row-warn' : ''}>
                  <td className="debug-key">{label}</td>
                  <td className="debug-val" style={w ? { color: '#dc2626' } : {}}>
                    {val !== null ? val.toFixed(dec) : '—'}
                  </td>
                  <td className="debug-unit">{unit}{w ? ` ⚠ ${w}` : ''}</td>
                </tr>
              )
            })}
          </tbody>
        </table>
      ) : (
        <div className="debug-no-data">waiting for data…</div>
      )}
    </div>
  )
}
