import { useState, useCallback } from 'react'

const ROADS = [
  { id: 'dry',      label: 'Dry',      cls: '',     mu: '1.0' },
  { id: 'wet',      label: 'Wet',      cls: 'wet',  mu: '0.6' },
  { id: 'race',     label: 'Race',     cls: 'race', mu: '1.1' },
  { id: 'mountain', label: 'Mountain', cls: 'mt',   mu: '0.9' },
]

export default function ControlPanel({ sendCommand, paused, onTogglePause }) {
  const [throttle,    setThrottle]    = useState(0.0)
  const [brakeFront,  setBrakeFront]  = useState(0.0)
  const [brakeRear,   setBrakeRear]   = useState(0.0)
  const [steerTorque, setSteerTorque] = useState(0.0)
  const [leanDelta,   setLeanDelta]   = useState(0.0)
  const [targetSpeed, setTargetSpeed] = useState(0.0)
  const [v0,          setV0]          = useState(10.0)
  const [road,        setRoad]        = useState('dry')

  function send(key, value) { sendCommand({ cmd: 'set', key, value }) }

  function handleReset() {
    setThrottle(0);   send('throttle', 0)
    setBrakeFront(0); send('brake_front', 0)
    setBrakeRear(0);  send('brake_rear', 0)
    setSteerTorque(0);send('steer_torque', 0)
    setLeanDelta(0);  send('lean_delta', 0)
    setTargetSpeed(0);send('target_speed', 0)
    sendCommand({ cmd: 'set', key: 'v0', value: v0 })
    sendCommand({ cmd: 'reset' })
  }

  const handleSteerRelease = useCallback(() => {
    setSteerTorque(0); send('steer_torque', 0)
  }, [])

  const ccActive = targetSpeed > 0
  const leanDeg  = leanDelta * 180 / Math.PI

  return (
    <div className="control-panel">

      <div className="ctrl-group">
        <span className="ctrl-label" title="Drive torque 0–100%">Throttle</span>
        <input type="range" className="ctrl-slider throttle-slider"
          min="0" max="1" step="0.02" value={throttle}
          onChange={e => { const v=parseFloat(e.target.value); setThrottle(v); send('throttle',v) }}
        />
        <span className="ctrl-value" style={{ color: throttle>0.5?'var(--green)':undefined }}>
          {(throttle*100).toFixed(0)}%
        </span>
      </div>

      <div className="ctrl-divider" />

      <div className="ctrl-group">
        <span className="ctrl-label">Brake F</span>
        <input type="range" className="ctrl-slider brake-slider"
          min="0" max="1" step="0.02" value={brakeFront}
          onChange={e => { const v=parseFloat(e.target.value); setBrakeFront(v); send('brake_front',v) }}
        />
        <span className="ctrl-value" style={{ color: brakeFront>0?'var(--red)':undefined }}>
          {(brakeFront*100).toFixed(0)}%
        </span>
      </div>

      <div className="ctrl-group">
        <span className="ctrl-label">Brake R</span>
        <input type="range" className="ctrl-slider brake-slider"
          min="0" max="1" step="0.02" value={brakeRear}
          onChange={e => { const v=parseFloat(e.target.value); setBrakeRear(v); send('brake_rear',v) }}
        />
        <span className="ctrl-value" style={{ color: brakeRear>0?'var(--red)':undefined }}>
          {(brakeRear*100).toFixed(0)}%
        </span>
      </div>

      <div className="ctrl-divider" />

      <div className="ctrl-group">
        <span className="ctrl-label" title="Steer torque — auto-zeroes on release">Steer τ</span>
        <input type="range" className="ctrl-slider steer-slider"
          min="-30" max="30" step="0.5" value={steerTorque}
          onChange={e => { const v=parseFloat(e.target.value); setSteerTorque(v); send('steer_torque',v) }}
          onMouseUp={handleSteerRelease} onTouchEnd={handleSteerRelease}
        />
        <span className="ctrl-value" style={{ color: Math.abs(steerTorque)>5?'var(--amber)':undefined }}>
          {steerTorque>=0?'+':''}{steerTorque.toFixed(1)}
        </span>
      </div>

      <div className="ctrl-group">
        <span className="ctrl-label" title="Rider upper-body lean command">Lean δ</span>
        <input type="range" className="ctrl-slider lean-slider"
          min="-17" max="17" step="0.5" value={leanDeg.toFixed(1)}
          onChange={e => {
            const rad = parseFloat(e.target.value) * Math.PI / 180
            setLeanDelta(rad); send('lean_delta', rad)
          }}
        />
        <span className="ctrl-value" style={{ color: Math.abs(leanDeg)>8?'var(--violet)':undefined }}>
          {leanDeg>=0?'+':''}{leanDeg.toFixed(1)}°
        </span>
      </div>

      <div className="ctrl-divider" />

      <div className="ctrl-group" style={ccActive?{borderColor:'var(--green)'}:{}}>
        <span className="ctrl-label">
          CC {ccActive && <span style={{color:'var(--green)',marginLeft:3,fontSize:9}}>ON</span>}
        </span>
        <input type="range" className="ctrl-slider"
          min="0" max="30" step="0.5" value={targetSpeed}
          onChange={e => { const v=parseFloat(e.target.value); setTargetSpeed(v); send('target_speed',v) }}
        />
        <span className="ctrl-value" style={{ color: ccActive?'var(--green)':undefined }}>
          {targetSpeed===0?'OFF':`${targetSpeed.toFixed(1)}`}
        </span>
      </div>

      <div className="ctrl-group">
        <span className="ctrl-label">v₀</span>
        <input type="range" className="ctrl-slider"
          min="1" max="25" step="0.5" value={v0}
          onChange={e => setV0(parseFloat(e.target.value))}
        />
        <span className="ctrl-value">{v0.toFixed(1)} m/s</span>
      </div>

      <div className="ctrl-divider" />

      <span className="ctrl-section-label">ROAD</span>
      <div className="road-chips">
        {ROADS.map(r => (
          <button key={r.id}
            className={`road-chip ${r.cls} ${road===r.id?'active':''}`}
            title={`μ=${r.mu}`}
            onClick={() => { setRoad(r.id); sendCommand({cmd:'set',key:'road',value:r.id}) }}
          >{r.label}</button>
        ))}
      </div>

      <div className="ctrl-divider" />

      <button className={`btn ${paused?'btn-resume':'btn-pause'}`} onClick={onTogglePause}>
        {paused ? '▶ Resume' : '⏸ Pause'}
      </button>
      <button className="btn btn-reset" onClick={handleReset}>↺ Reset</button>
    </div>
  )
}
