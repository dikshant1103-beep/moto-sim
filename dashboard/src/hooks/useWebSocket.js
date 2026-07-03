import { useEffect, useRef, useState, useCallback } from 'react'

const MAX_POINTS = 1000
const D2R = Math.PI / 180

function isPlausibleFrame(frame) {
  if (!frame) return false
  const nums = [frame.t, frame.vx, frame.vy, frame.roll, frame.pitch, frame.yaw_rate, frame.f_Fz, frame.r_Fz]
  if (nums.some(v => v !== undefined && !Number.isFinite(v))) return false
  if (Math.abs(frame.vx ?? 0) > 120) return false
  if (Math.abs(frame.vy ?? 0) > 80) return false
  if (Math.abs(frame.roll ?? 0) > Math.PI) return false
  return true
}

function normalizeFrame(raw) {
  if (!raw || typeof raw !== 'object') return raw

  // Native moto_sim payload: already matches this dashboard.
  if ('t' in raw || 'roll' in raw || 'f_Fz' in raw || 'r_Fz' in raw) {
    return raw
  }

  // Legacy tire-rig payload: map into the motorcycle dashboard schema.
  return {
    ...raw,
    t: raw.time ?? 0,
    roll: raw.roll ?? 0,
    pitch: raw.pitch ?? 0,
    yaw: raw.yaw ?? 0,
    roll_rate: raw.roll_rate ?? 0,
    yaw_rate: raw.yaw_rate ?? 0,
    vx: raw.vx ?? 0,
    vy: raw.vy ?? 0,
    ax: raw.ax ?? 0,
    ay: raw.ay ?? 0,
    f_alpha: Number.isFinite(raw.alpha) ? raw.alpha * D2R : 0,
    f_alpha_prime: Number.isFinite(raw.alpha_prime) ? raw.alpha_prime * D2R : 0,
    f_kappa: raw.kappa ?? 0,
    f_Fz: raw.Fz ?? 0,
    f_Fx: raw.Fx ?? 0,
    f_Fy: raw.Fy ?? 0,
    f_Mz: raw.Mz ?? 0,
    r_alpha: raw.r_alpha ?? 0,
    r_alpha_prime: raw.r_alpha_prime ?? 0,
    r_kappa: raw.r_kappa ?? 0,
    r_Fz: raw.r_Fz ?? 0,
    r_Fx: raw.r_Fx ?? 0,
    r_Fy: raw.r_Fy ?? 0,
    r_Mz: raw.r_Mz ?? 0,
    throttle: raw.throttle ?? 0,
    brake_front: raw.brake_front ?? 0,
    brake_rear: raw.brake_rear ?? 0,
    steer_torque: raw.steer_torque ?? 0,
  }
}

export function useWebSocket(url = 'ws://localhost:8765') {
  const dataRef      = useRef([])
  const latestRef    = useRef(null)
  const wsRef        = useRef(null)
  const frameCount   = useRef(0)
  const fpsCountRef  = useRef(0)
  const [connected,   setConnected]   = useState(false)
  const [paused,      setPaused]      = useState(false)
  const [frameRate,   setFrameRate]   = useState(0)
  const [totalFrames, setTotalFrames] = useState(0)

  useEffect(() => {
    let ws
    let retryTimer

    function connect() {
      ws = new WebSocket(url)
      wsRef.current = ws

      ws.onopen = () => {
        console.log('[WS] connected to', url)
        setConnected(true)
      }

      ws.onmessage = (event) => {
        try {
          const frame = normalizeFrame(JSON.parse(event.data))
          if (!isPlausibleFrame(frame)) {
            console.warn('[WS] dropped implausible frame', frame)
            return
          }
          latestRef.current = frame
          dataRef.current.push(frame)
          if (dataRef.current.length > MAX_POINTS) dataRef.current.shift()
          frameCount.current  += 1
          fpsCountRef.current += 1

          if (frameCount.current % 100 === 0) {
            console.log(
              `[WS] frame #${frameCount.current}  buf=${dataRef.current.length}`,
              `t=${frame.t?.toFixed(3)}  vx=${frame.vx?.toFixed(2)}`,
              `roll=${frame.roll?.toFixed(3)} rad  yaw_rate=${frame.yaw_rate?.toFixed(4)} rad/s`,
              `f_Fz=${frame.f_Fz?.toFixed(0)} N  r_Fz=${frame.r_Fz?.toFixed(0)} N`,
            )
            if ((frame.f_Fz ?? 0) === 0) console.warn('[WS] f_Fz=0 — front ground contact broken?')
            if ((frame.r_Fz ?? 0) === 0) console.warn('[WS] r_Fz=0 — rear ground contact broken?')
            if (!Number.isFinite(frame.roll)) console.error('[WS] roll is NaN/Inf — physics exploded')
          }
        } catch (e) {
          console.error('[WS] parse error:', e)
        }
      }

      ws.onclose = (ev) => {
        console.warn('[WS] closed — code:', ev.code)
        setConnected(false)
        retryTimer = setTimeout(connect, 2000)
      }

      ws.onerror = () => ws.close()
    }

    connect()

    const fpsTimer = setInterval(() => {
      setFrameRate(fpsCountRef.current)
      setTotalFrames(frameCount.current)
      fpsCountRef.current = 0
    }, 1000)

    return () => {
      clearTimeout(retryTimer)
      clearInterval(fpsTimer)
      ws?.close()
    }
  }, [url])

  const sendCommand = useCallback((obj) => {
    if (wsRef.current?.readyState === WebSocket.OPEN)
      wsRef.current.send(JSON.stringify(obj))
  }, [])

  const togglePause = useCallback(() => {
    setPaused(prev => {
      const next = !prev
      if (wsRef.current?.readyState === WebSocket.OPEN)
        wsRef.current.send(JSON.stringify({ cmd: next ? 'pause' : 'resume' }))
      return next
    })
  }, [])

  const clearBuffer = useCallback(() => { dataRef.current = [] }, [])

  const exportCSV = useCallback(() => {
    const buf = dataRef.current
    if (buf.length === 0) return
    const headers = Object.keys(buf[0]).join(',')
    const rows    = buf.map(row => Object.values(row).join(','))
    const csv     = [headers, ...rows].join('\n')
    const blob    = new Blob([csv], { type: 'text/csv' })
    const url_    = URL.createObjectURL(blob)
    const a       = document.createElement('a')
    a.href        = url_
    a.download    = `moto_telemetry_${Date.now()}.csv`
    a.click()
    URL.revokeObjectURL(url_)
  }, [])

  return { dataRef, latestRef, connected, paused, frameRate, totalFrames,
           sendCommand, togglePause, clearBuffer, exportCSV }
}
