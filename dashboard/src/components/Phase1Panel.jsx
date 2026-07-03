/**
 * Phase1Panel — C++ 13-DOF engine diagnostics (shown when engine = "C++ 13-DOF …")
 *
 * Displays fields that only the C++ engine produces:
 *   - Effective trail vs time (dynamic Cossalter geometry)
 *   - Effective wheelbase vs time
 *   - Roll rate vs time
 *
 * Hidden automatically when Python simulation is connected (no engine field).
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 150

function ds(label, color, dash = []) {
  return { label, data: [], borderColor: color, borderWidth: 1.5,
           pointRadius: 0, tension: 0, borderDash: dash }
}

function makeChart(ctx, title, datasets, yLabel, yMin, yMax) {
  return new Chart(ctx, {
    type: 'line',
    data: { datasets },
    options: {
      animation: false, responsive: true, maintainAspectRatio: false,
      scales: {
        x: { type: 'linear',
             title: { display: true, text: 'time [s]', color: '#475569', font: { size: 9 } },
             ticks: { color: '#475569', maxTicksLimit: 6, font: { size: 9 } },
             grid: { color: 'rgba(56,189,248,0.08)' } },
        y: { min: yMin, max: yMax,
             title: { display: true, text: yLabel, color: '#475569', font: { size: 9 } },
             ticks: { color: '#475569', font: { size: 9 } },
             grid: { color: 'rgba(56,189,248,0.08)' } },
      },
      plugins: {
        legend: { labels: { color: '#475569', boxWidth: 10, font: { size: 9 } } },
        title: { display: true, text: title, color: '#0f172a', font: { size: 11, weight: 'bold' } },
      },
    },
  })
}

export default function Phase1Panel({ dataRef, latestRef }) {
  const trailRef   = useRef(null)
  const wbRef      = useRef(null)
  const rollRateRef = useRef(null)
  const charts     = useRef([])
  const timerRef   = useRef(null)

  useEffect(() => {
    const trailChart = makeChart(
      trailRef.current.getContext('2d'),
      'Dynamic Trail (Cossalter Kinematics)',
      [ds('Effective trail [mm]', '#f97316')],
      'Trail [mm]', 50, 130,
    )
    const wbChart = makeChart(
      wbRef.current.getContext('2d'),
      'Dynamic Wheelbase',
      [ds('Effective wheelbase [m]', '#3b82f6')],
      'Wheelbase [m]', 1.30, 1.42,
    )
    const rrChart = makeChart(
      rollRateRef.current.getContext('2d'),
      'Roll Rate',
      [ds('Roll rate [°/s]', '#a855f7')],
      'Roll rate [°/s]', -60, 60,
    )

    charts.current = [trailChart, wbChart, rrChart]

    function tick() {
      const buf = dataRef.current
      if (!buf || buf.length === 0) return

      const [d0] = trailChart.data.datasets
      d0.data = buf.map(d => ({ x: d.t, y: (d.effective_trail ?? 0.097) * 1000 }))
      trailChart.update('none')

      const [w0] = wbChart.data.datasets
      w0.data = buf.map(d => ({ x: d.t, y: d.effective_wheelbase ?? 1.37 }))
      wbChart.update('none')

      const R2D = 180 / Math.PI
      const [r0] = rrChart.data.datasets
      r0.data = buf.map(d => ({ x: d.t, y: (d.roll_rate ?? 0) * R2D }))
      rrChart.update('none')
    }

    tick()
    timerRef.current = setInterval(tick, UPDATE_MS)
    return () => { clearInterval(timerRef.current); charts.current.forEach(c => c.destroy()) }
  }, []) // eslint-disable-line

  return (
    <div style={{ display: 'flex', flexDirection: 'column', width: '100%', height: '100%' }}>
      <div style={{
        padding: '4px 10px', fontSize: 11, fontWeight: 700, color: '#1e3a8a',
        letterSpacing: '0.08em', borderBottom: '1px solid #e2e8f0',
        background: '#eff6ff', flexShrink: 0,
      }}>
        PART B PHASE 1 — C++ 13-DOF ENGINE DIAGNOSTICS
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr',
                    gap: 4, flex: 1, minHeight: 0, padding: '4px' }}>
        <div style={{ position: 'relative', minHeight: 0 }}><canvas ref={trailRef} /></div>
        <div style={{ position: 'relative', minHeight: 0 }}><canvas ref={wbRef} /></div>
        <div style={{ position: 'relative', minHeight: 0 }}><canvas ref={rollRateRef} /></div>
      </div>
    </div>
  )
}
