/**
 * CoeffFitterPanel — Phase 3.1 real-time coefficient fitter diagnostics.
 * Shown when engine label contains a C++ backend with --coeff-fit active.
 *
 * Displays:
 *   - 6 live coefficient scales as animated gauge bars (vs nominal = 1.0)
 *   - Historical traces of mu_y and mu_x over time (grip evolution)
 *   - Fitter latency and update count
 *
 * Fields read from telemetry JSON:
 *   cf_mu_y, cf_p_Ky1, cf_E_Fy, cf_mu_x, cf_K_x, cf_tp0
 *   cf_updates, cf_lat_us
 */
import { useEffect, useRef, useState } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 200
const MAX_PTS   = 400

const COEFFS = [
  { key: 'cf_mu_y',   label: 'μ_y',   color: '#06b6d4', desc: 'lateral grip' },
  { key: 'cf_p_Ky1',  label: 'K_α',   color: '#a3e635', desc: 'lat. stiffness' },
  { key: 'cf_E_Fy',   label: 'E_y',   color: '#c084fc', desc: 'curvature' },
  { key: 'cf_mu_x',   label: 'μ_x',   color: '#f97316', desc: 'long. grip' },
  { key: 'cf_K_x',    label: 'K_κ',   color: '#fbbf24', desc: 'long. stiff.' },
  { key: 'cf_tp0',    label: 't_p',   color: '#f43f5e', desc: 'pneu. trail' },
]

// Physical scale ranges (must match SCALE_MIN/MAX in Python)
const SCALE_MIN = [0.45, 0.60, 0.70, 0.45, 0.70, 0.60]
const SCALE_MAX = [1.15, 1.40, 1.30, 1.15, 1.30, 1.50]

function ScaleBar({ label, value, color, desc, min, max }) {
  const pct = Math.max(0, Math.min(100, (value - min) / (max - min) * 100))
  const nomPct = (1.0 - min) / (max - min) * 100   // nominal = 1.0 position
  const warn = value < min + (max - min) * 0.25     // bottom 25% = warn

  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '3px 0' }}>
      <span style={{ width: 30, fontSize: 11, fontWeight: 700, color, textAlign: 'right',
                     fontFamily: 'monospace' }}>{label}</span>
      <div style={{ flex: 1, position: 'relative', height: 14,
                    background: 'var(--bg-overlay)', borderRadius: 3, overflow: 'visible' }}>
        {/* Nominal marker */}
        <div style={{
          position: 'absolute', left: `${nomPct}%`, top: -2, bottom: -2,
          width: 1, background: 'rgba(255,255,255,0.25)', zIndex: 2,
        }} />
        {/* Fill */}
        <div style={{
          position: 'absolute', left: 0, top: 0, bottom: 0,
          width: `${pct}%`,
          background: warn
            ? 'linear-gradient(90deg, var(--red), var(--amber))'
            : `linear-gradient(90deg, ${color}88, ${color})`,
          borderRadius: 3,
          transition: 'width 0.3s ease',
        }} />
      </div>
      <span style={{
        width: 46, fontSize: 11, fontVariantNumeric: 'tabular-nums',
        fontWeight: 700, textAlign: 'right',
        color: warn ? 'var(--red)' : value > 1.05 ? 'var(--green)' : 'var(--text-hi)',
      }}>
        {value.toFixed(3)}
      </span>
      <span style={{ width: 80, fontSize: 9, color: 'var(--text-dim)' }}>{desc}</span>
    </div>
  )
}

export default function CoeffFitterPanel({ dataRef, latestRef }) {
  const gripCanvasRef = useRef(null)
  const stiffCanvasRef= useRef(null)
  const chartRefs     = useRef({})
  const timerRef      = useRef(null)
  const [stats, setStats] = useState({ updates: 0, lat_us: 0.0 })
  const [scales, setScales] = useState(COEFFS.map(() => 1.0))

  useEffect(() => {
    const mkChart = (canvas, title, ds) => new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: { datasets: ds },
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        scales: {
          x: { type:'linear', title:{ display:true, text:'time [s]', color:'#475569', font:{size:9} },
               ticks:{ color:'#475569', font:{size:8}, maxTicksLimit:6 }, grid:{ color:'rgba(56,189,248,0.08)' } },
          y: { min:0.3, max:1.6,
               title:{ display:true, text:'scale factor', color:'#475569', font:{size:9} },
               ticks:{ color:'#475569', font:{size:8}, maxTicksLimit:5 }, grid:{ color:'rgba(56,189,248,0.08)' } },
        },
        plugins: {
          title:  { display:true, text:title, color:'#94a3b8', font:{size:10,weight:'bold'} },
          legend: { labels:{ color:'#64748b', font:{size:9}, boxWidth:10 } },
          annotation: {},
        },
      },
    })

    const gripDs  = [
      { label:'μ_y (lateral grip)', data:[], borderColor:'#06b6d4', borderWidth:1.8, pointRadius:0, tension:0 },
      { label:'μ_x (long. grip)',   data:[], borderColor:'#f97316', borderWidth:1.8, pointRadius:0, tension:0, borderDash:[4,2] },
    ]
    const stiffDs = [
      { label:'K_α (lat. stiff.)',  data:[], borderColor:'#a3e635', borderWidth:1.5, pointRadius:0, tension:0 },
      { label:'K_κ (long. stiff.)', data:[], borderColor:'#fbbf24', borderWidth:1.5, pointRadius:0, tension:0, borderDash:[4,2] },
      { label:'t_p (pneu. trail)',  data:[], borderColor:'#f43f5e', borderWidth:1.5, pointRadius:0, tension:0, borderDash:[2,2] },
    ]

    if (gripCanvasRef.current)   chartRefs.current.grip  = mkChart(gripCanvasRef.current,  'Grip Scales vs Time', gripDs)
    if (stiffCanvasRef.current)  chartRefs.current.stiff = mkChart(stiffCanvasRef.current, 'Stiffness & Trail Scales', stiffDs)

    // Nominal reference lines at 1.0
    const addNomLine = (ch) => {
      ch.data.datasets.push({
        label:'nominal (1.0)', data:[], borderColor:'rgba(255,255,255,0.15)',
        borderWidth:1, pointRadius:0, tension:0, borderDash:[6,3],
      })
    }
    if (chartRefs.current.grip)  addNomLine(chartRefs.current.grip)
    if (chartRefs.current.stiff) addNomLine(chartRefs.current.stiff)

    timerRef.current = setInterval(() => {
      const buf = dataRef.current
      if (!buf.length) return
      const s = buf.slice(-MAX_PTS)

      const { grip, stiff } = chartRefs.current
      if (grip) {
        grip.data.datasets[0].data = s.map(d => ({ x:d.t, y:d.cf_mu_y  ?? 1 }))
        grip.data.datasets[1].data = s.map(d => ({ x:d.t, y:d.cf_mu_x  ?? 1 }))
        grip.data.datasets[2].data = s.map(d => ({ x:d.t, y:1.0 }))
        grip.update('none')
      }
      if (stiff) {
        stiff.data.datasets[0].data = s.map(d => ({ x:d.t, y:d.cf_p_Ky1 ?? 1 }))
        stiff.data.datasets[1].data = s.map(d => ({ x:d.t, y:d.cf_K_x   ?? 1 }))
        stiff.data.datasets[2].data = s.map(d => ({ x:d.t, y:d.cf_tp0   ?? 1 }))
        stiff.data.datasets[3].data = s.map(d => ({ x:d.t, y:1.0 }))
        stiff.update('none')
      }

      const latest = latestRef.current
      if (latest) {
        setScales(COEFFS.map(c => latest[c.key] ?? 1.0))
        setStats({ updates: latest.cf_updates ?? 0, lat_us: latest.cf_lat_us ?? 0 })
      }
    }, UPDATE_MS)

    return () => {
      clearInterval(timerRef.current)
      Object.values(chartRefs.current).forEach(c => { try { c.destroy() } catch {} })
    }
  }, [dataRef, latestRef])

  const active = (latestRef.current?.cf_updates ?? 0) > 0

  return (
    <div style={{ padding:'6px 8px', height:'100%', display:'flex', flexDirection:'column', gap:6 }}>
      {/* Header */}
      <div style={{ display:'flex', alignItems:'center', gap:10, flexShrink:0 }}>
        <span style={{ fontSize:10, fontWeight:800, color:'var(--cyan)', textTransform:'uppercase', letterSpacing:'0.1em' }}>
          Phase 3.1 — PINN Coefficient Fitter
        </span>
        <span style={{
          padding:'2px 8px', borderRadius:4, fontSize:9, fontWeight:700,
          background: active ? 'var(--green-dim)' : 'var(--bg-overlay)',
          border:`1px solid ${active ? 'var(--green)' : 'var(--border-dim)'}`,
          color: active ? 'var(--green)' : 'var(--text-dim)',
        }}>
          {active ? `ACTIVE  ${stats.updates} updates  ${stats.lat_us.toFixed(1)}µs/call` : 'INACTIVE'}
        </span>
      </div>

      {/* Scale bars */}
      <div style={{ flexShrink:0, background:'var(--bg-raised)', borderRadius:'var(--radius-md)',
                    padding:'6px 10px', border:'1px solid var(--border-dim)' }}>
        {COEFFS.map((c, i) => (
          <ScaleBar key={c.key} label={c.label} value={scales[i]}
                    color={c.color} desc={c.desc}
                    min={SCALE_MIN[i]} max={SCALE_MAX[i]} />
        ))}
        <div style={{ marginTop:5, fontSize:9, color:'var(--text-dim)' }}>
          Nominal = 1.0 │ bar marker │ range = physical bounds
        </div>
      </div>

      {/* Charts */}
      <div style={{ flex:1, display:'grid', gridTemplateColumns:'1fr 1fr', gap:6, minHeight:0 }}>
        <div style={{ position:'relative', minHeight:120 }}>
          <canvas ref={gripCanvasRef} />
        </div>
        <div style={{ position:'relative', minHeight:120 }}>
          <canvas ref={stiffCanvasRef} />
        </div>
      </div>
    </div>
  )
}
