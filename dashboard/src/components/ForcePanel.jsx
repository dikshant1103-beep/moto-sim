/**
 * ForcePanel — Longitudinal Fx and lateral Fy for front + rear tyres vs time.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 140

export default function ForcePanel({ dataRef }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const timerRef  = useRef(null)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'Fx_front [N]', data: [], borderColor: '#a3e635', borderWidth: 1.5, pointRadius: 0, tension: 0 },
          { label: 'Fx_rear  [N]', data: [], borderColor: '#22d3ee', borderWidth: 1.5, pointRadius: 0, tension: 0 },
          { label: 'Fy_front [N]', data: [], borderColor: '#f97316', borderWidth: 1.2, pointRadius: 0, tension: 0, borderDash: [4,2] },
          { label: 'Fy_rear  [N]', data: [], borderColor: '#c084fc', borderWidth: 1.2, pointRadius: 0, tension: 0, borderDash: [4,2] },
        ],
      },
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        interaction: { mode: 'index', intersect: false },
        scales: {
          x: { type: 'linear', title: { display: true, text: 'time [s]', color: '#475569' },
               ticks: { color: '#475569', maxTicksLimit: 8 }, grid: { color: 'rgba(56,189,248,0.08)' } },
          y: { title: { display: true, text: 'Fx / Fy [N]', color: '#475569' },
               ticks: { color: '#475569' }, grid: { color: 'rgba(56,189,248,0.08)' } },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12, font: { size: 10 } } },
          title:  { display: true, text: 'Longitudinal & Lateral Forces', color: '#0f172a', font: { size: 13 } },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [d0, d1, d2, d3] = chartRef.current.data.datasets
        d0.data = buf.map(d => ({ x: d.t, y: d.f_Fx ?? 0 }))
        d1.data = buf.map(d => ({ x: d.t, y: d.r_Fx ?? 0 }))
        d2.data = buf.map(d => ({ x: d.t, y: d.f_Fy ?? 0 }))
        d3.data = buf.map(d => ({ x: d.t, y: d.r_Fy ?? 0 }))
        chartRef.current.update('none')
      }
    }
    tick()
    timerRef.current = setInterval(tick, UPDATE_MS)

    return () => { clearInterval(timerRef.current); chartRef.current.destroy() }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <canvas ref={canvasRef} />
    </div>
  )
}
