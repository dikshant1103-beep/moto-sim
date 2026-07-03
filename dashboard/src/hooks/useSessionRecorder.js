/**
 * useSessionRecorder — Auto-snapshot hook
 *
 * Watches telemetry for significant events and saves PNG snapshots of every
 * visible chart canvas + a JSON event log to:
 *   ~/moto_sim_sessions/session_YYYY-MM-DD_HH-MM-SS/
 *     metadata.json          ← session info + event list
 *     events/
 *       001_start_t0.00s.png
 *       002_braking_t12.34s.png
 *       003_lean_peak_35.2deg_t18.00s.png
 *       ...
 *     telemetry.csv          ← written on stop()
 *
 * Events detected:
 *   start        — first frame after recording begins
 *   braking      — brake_front or brake_rear crosses 0.25 (rising edge)
 *   corner_entry — |roll| crosses 12° (rising edge)
 *   lean_peak    — local maximum of |roll| > 20° (falling edge after peak)
 *   throttle_on  — throttle crosses 0.3 (rising edge)
 *   interval     — every 30 s of simulation time
 */
import { useRef, useCallback } from 'react'

const R2D = 180 / Math.PI

function ts() {
  const n = new Date()
  return [n.getFullYear(),
    String(n.getMonth()+1).padStart(2,'0'),
    String(n.getDate()).padStart(2,'0')].join('-') + '_' +
    [String(n.getHours()).padStart(2,'0'),
     String(n.getMinutes()).padStart(2,'0'),
     String(n.getSeconds()).padStart(2,'0')].join('-')
}

async function getBaseDir() {
  if (!window.electronAPI?.homeDir) return null
  const home = await window.electronAPI.homeDir()
  return `${home}/moto_sim_sessions`
}

async function captureAllCharts(sessionDir, eventIdx, eventName, simTime) {
  if (!window.electronAPI?.savePng) return
  const canvases = document.querySelectorAll('canvas')
  let saved = 0
  for (let i = 0; i < canvases.length; i++) {
    const c = canvases[i]
    // Skip Three.js WebGL canvases (they report context type = webgl)
    try {
      const gl = c.getContext('webgl') || c.getContext('webgl2')
      if (gl) continue
    } catch {}
    try {
      const b64 = c.toDataURL('image/png')
      const label = eventName.replace(/[^a-z0-9_]/gi, '_')
      const fname = `${String(eventIdx).padStart(3,'0')}_${label}_t${simTime.toFixed(2)}s_chart${i+1}.png`
      await window.electronAPI.savePng(b64, `${sessionDir}/events/${fname}`)
      saved++
    } catch {}
  }
  return saved
}

export function useSessionRecorder(dataRef, latestRef) {
  const sessionDirRef  = useRef(null)
  const activeRef      = useRef(false)
  const eventCountRef  = useRef(0)
  const eventLogRef    = useRef([])
  const engineRef      = useRef('')
  const prevRef        = useRef({
    braking: false, corner: false, throttleOn: false,
    rollPeak: 0, lastInterval: -30,
  })

  const logEvent = useCallback(async (name, simTime, extra = {}) => {
    if (!activeRef.current || !sessionDirRef.current) return
    const idx = ++eventCountRef.current
    const entry = { idx, event: name, sim_t: simTime, wall_t: Date.now(), ...extra }
    eventLogRef.current.push(entry)
    console.log(`[recorder] event ${idx}: ${name} @ t=${simTime.toFixed(2)}s`)
    await captureAllCharts(sessionDirRef.current, idx, name, simTime)
    // Update metadata
    await saveMetadata()
  }, [])

  const saveMetadata = useCallback(async () => {
    if (!sessionDirRef.current) return
    const meta = {
      session_dir:  sessionDirRef.current,
      started_at:   eventLogRef.current[0]?.wall_t ?? Date.now(),
      engine:       engineRef.current,
      event_count:  eventCountRef.current,
      events:       eventLogRef.current,
    }
    await window.electronAPI?.saveText(
      JSON.stringify(meta, null, 2),
      `${sessionDirRef.current}/metadata.json`
    )
  }, [])

  const start = useCallback(async () => {
    const base = await getBaseDir()
    if (!base) return null
    const name = `session_${ts()}`
    const dir  = `${base}/${name}`
    await window.electronAPI?.ensureDir(`${dir}/events`)
    sessionDirRef.current = dir
    activeRef.current     = true
    eventCountRef.current = 0
    eventLogRef.current   = []
    prevRef.current = { braking: false, corner: false, throttleOn: false, rollPeak: 0, lastInterval: -30 }
    engineRef.current = latestRef.current?.engine ?? ''
    console.log(`[recorder] session started: ${dir}`)
    return dir
  }, [latestRef])

  const stop = useCallback(async () => {
    if (!activeRef.current || !sessionDirRef.current) return
    activeRef.current = false
    // Export telemetry CSV
    const buf = dataRef.current
    if (buf.length > 0) {
      const headers = Object.keys(buf[0]).join(',')
      const rows    = buf.map(r => Object.values(r).join(','))
      await window.electronAPI?.saveText(
        [headers, ...rows].join('\n'),
        `${sessionDirRef.current}/telemetry.csv`
      )
    }
    await saveMetadata()
    const dir = sessionDirRef.current
    sessionDirRef.current = null
    console.log(`[recorder] session stopped: ${dir}`)
    return dir
  }, [dataRef, saveMetadata])

  // Call this on every new telemetry frame to detect events
  const onFrame = useCallback((frame) => {
    if (!activeRef.current || !frame) return
    const p    = prevRef.current
    const t    = frame.t ?? 0
    const roll = Math.abs((frame.roll ?? 0) * R2D)
    const brakingNow    = (frame.brake_front ?? 0) > 0.25 || (frame.brake_rear ?? 0) > 0.25
    const cornerNow     = roll > 12
    const throttleNow   = (frame.throttle ?? 0) > 0.3

    // Start snapshot on first frame
    if (eventCountRef.current === 0) {
      logEvent('start', t, { engine: frame.engine })
    }

    // Braking rising edge
    if (brakingNow && !p.braking) {
      logEvent('braking', t, {
        brake_f: +(frame.brake_front ?? 0).toFixed(2),
        brake_r: +(frame.brake_rear  ?? 0).toFixed(2),
        vx:      +(frame.vx ?? 0).toFixed(2),
      })
    }

    // Corner entry rising edge
    if (cornerNow && !p.corner) {
      logEvent('corner_entry', t, { lean_deg: +roll.toFixed(1) })
    }

    // Lean peak (falling edge after peak > 20°)
    if (!cornerNow && p.corner && p.rollPeak > 20) {
      logEvent('lean_peak', t, { peak_deg: +p.rollPeak.toFixed(1) })
    }

    // Throttle on
    if (throttleNow && !p.throttleOn) {
      logEvent('throttle_on', t, { throttle: +(frame.throttle ?? 0).toFixed(2), vx: +(frame.vx ?? 0).toFixed(2) })
    }

    // Interval snapshot every 30 sim-seconds
    if (t - p.lastInterval >= 30) {
      logEvent('interval', t, { vx: +(frame.vx ?? 0).toFixed(2) })
      p.lastInterval = t
    }

    // Update state
    p.braking    = brakingNow
    p.corner     = cornerNow
    p.throttleOn = throttleNow
    p.rollPeak   = cornerNow ? Math.max(p.rollPeak, roll) : 0
  }, [logEvent])

  return { start, stop, onFrame, sessionDir: sessionDirRef.current }
}
