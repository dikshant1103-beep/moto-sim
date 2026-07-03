const fmt = (v, d=1) => (v===undefined||v===null) ? '—' : Number(v).toFixed(d)
const deg = (v, d=1) => (v===undefined||v===null) ? '—' : (v*180/Math.PI).toFixed(d)

export default function StatusBar({ connected, paused, latest, engineBadgeClass, onTogglePause, onClear, onExport }) {
  const isCpp = latest?.engine?.startsWith('C++')

  return (
    <div className="status-bar">
      <div className="status-left">
        {/* Connection status */}
        <span className="dot" style={{
          background: connected ? 'var(--green)' : 'var(--red)',
          boxShadow: `0 0 6px ${connected ? 'var(--green)' : 'var(--red)'}`,
          ...(connected ? {} : { animation: 'pulse-red 1.5s infinite' })
        }} />
        <span className="status-label" style={{ color: connected ? 'var(--green)' : 'var(--red)' }}>
          {connected ? 'Connected' : 'Disconnected — retrying…'}
        </span>

        {latest?.engine && (
          <span className={`engine-badge ${engineBadgeClass ?? 'cpp'}`}>
            {latest.engine}
          </span>
        )}

        {latest && <>
          <span className="readout">t <b>{fmt(latest.t, 3)} s</b></span>
          <span className="readout">v <b>{fmt(latest.vx, 2)} m/s</b></span>
          <span className="readout">roll <b style={{ color: Math.abs(latest.roll??0)>0.3?'var(--amber)':undefined }}>{deg(latest.roll, 1)}°</b></span>
          <span className="readout">ψ̇ <b>{deg(latest.yaw_rate, 2)}°/s</b></span>
          <span className="readout">Fz_f <b>{fmt(latest.f_Fz, 0)} N</b></span>
          <span className="readout">Fz_r <b>{fmt(latest.r_Fz, 0)} N</b></span>
          <span className="readout">δ <b>{deg(latest.steer_angle, 2)}°</b></span>
          {isCpp && <>
            <span className="readout">trail <b>{fmt((latest.effective_trail??0)*1000, 1)} mm</b></span>
          </>}
        </>}
      </div>

      <div className="status-right">
        <button className={`btn ${paused ? 'btn-resume' : 'btn-pause'}`} onClick={onTogglePause}>
          {paused ? '▶ Resume' : '⏸ Pause'}
        </button>
        <button className="btn" onClick={onClear}>Clear</button>
        <button className="btn btn-export" onClick={onExport}>↓ CSV</button>
      </div>
    </div>
  )
}
