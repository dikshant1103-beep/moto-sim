/**
 * LoadPanel — Normal loads Fz (front + rear) vs time, plus a live weight-transfer bar.
 * The bar shows the front-axle fraction: f_Fz / (f_Fz + r_Fz).
 */
import { useEffect, useRef, useState } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 140

export default function LoadPanel({ dataRef, latestRef }) {
  const canvasRef   = useRef(null)
  const chartRef    = useRef(null)
  const timerRef    = useRef(null)
  const [ratio, setRatio] = useState(0.5)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'Fz_front [N]', data: [], borderColor: '#a3e635', borderWidth: 1.5, pointRadius: 0, tension: 0 },
          { label: 'Fz_rear  [N]', data: [], borderColor: '#22d3ee', borderWidth: 1.5, pointRadius: 0, tension: 0, borderDash: [4,2] },
        ],
      },
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        scales: {
          x: { type: 'linear', title: { display: true, text: 'time [s]', color: '#475569' },
               ticks: { color: '#475569', maxTicksLimit: 8 }, grid: { color: 'rgba(56,189,248,0.08)' } },
          y: { title: { display: true, text: 'Normal Load [N]', color: '#475569' },
               ticks: { color: '#475569' }, grid: { color: 'rgba(56,189,248,0.08)' }, min: 0 },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12, font: { size: 10 } } },
          title:  { display: true, text: 'Normal Loads — Front (lime) / Rear (cyan)', color: '#0f172a', font: { size: 13 } },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [d0, d1] = chartRef.current.data.datasets
        d0.data = buf.map(d => ({ x: d.t, y: d.f_Fz ?? 0 }))
        d1.data = buf.map(d => ({ x: d.t, y: d.r_Fz ?? 0 }))
        chartRef.current.update('none')
      }
      const fr = latestRef.current
      if (fr) {
        const fFz = fr.f_Fz ?? 0
        const rFz = fr.r_Fz ?? 0
        const total = fFz + rFz
        setRatio(total > 0 ? fFz / total : 0.5)
      }
    }
    tick()
    timerRef.current = setInterval(tick, UPDATE_MS)

    return () => { clearInterval(timerRef.current); chartRef.current.destroy() }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  const pct = (ratio * 100).toFixed(1)
  const barColor = ratio > 0.65 ? '#ef4444' : ratio < 0.30 ? '#f97316' : '#22d3ee'

  return (
    <div style={{ display: 'flex', flexDirection: 'column', width: '100%', height: '100%', gap: 6 }}>
      <div style={{ flex: 1, position: 'relative', minHeight: 0 }}>
        <canvas ref={canvasRef} />
      </div>
      <div style={{ padding: '4px 8px', background: 'rgba(241,245,249,0.8)', borderRadius: 6, flexShrink: 0 }}>
        <div style={{ fontSize: 10, color: '#64748b', marginBottom: 3 }}>
          Weight transfer &nbsp;|&nbsp; Front {pct}% &nbsp;/&nbsp; Rear {(100 - +pct).toFixed(1)}%
        </div>
        <div style={{ display: 'flex', height: 12, borderRadius: 4, overflow: 'hidden', background: '#22d3ee' }}>
          <div style={{
            width: `${ratio * 100}%`,
            background: barColor,
            transition: 'width 0.12s ease',
          }} />
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 9, color: '#94a3b8', marginTop: 2 }}>
          <span>← Front</span>
          <span>Rear →</span>
        </div>
      </div>
    </div>
  )
}
