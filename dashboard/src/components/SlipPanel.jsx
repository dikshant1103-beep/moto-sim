/**
 * SlipPanel — slip angle α and slip ratio κ for front + rear tyres vs time.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 140

export default function SlipPanel({ dataRef }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const timerRef  = useRef(null)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'α_f [°]', data: [], yAxisID: 'yAlpha',
            borderColor: '#7c3aed', borderWidth: 1.5, pointRadius: 0, tension: 0 },
          { label: 'α_r [°]', data: [], yAxisID: 'yAlpha',
            borderColor: '#c084fc', borderWidth: 1.5, pointRadius: 0, tension: 0, borderDash: [5,2] },
          { label: 'κ_f [–]', data: [], yAxisID: 'yKappa',
            borderColor: '#ea580c', borderWidth: 1.3, pointRadius: 0, tension: 0 },
          { label: 'κ_r [–]', data: [], yAxisID: 'yKappa',
            borderColor: '#f97316', borderWidth: 1.3, pointRadius: 0, tension: 0, borderDash: [5,2] },
        ],
      },
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        scales: {
          x: { type: 'linear', title: { display: true, text: 'time [s]', color: '#475569' },
               ticks: { color: '#475569', maxTicksLimit: 8 }, grid: { color: 'rgba(56,189,248,0.08)' } },
          yAlpha: { type: 'linear', position: 'left',
                    title: { display: true, text: 'α [°]', color: '#7c3aed' },
                    ticks: { color: '#7c3aed' }, grid: { color: 'rgba(56,189,248,0.08)' } },
          yKappa: { type: 'linear', position: 'right',
                    title: { display: true, text: 'κ [–]', color: '#ea580c' },
                    ticks: { color: '#ea580c' }, grid: { drawOnChartArea: false } },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12, font: { size: 10 } } },
          title:  { display: true, text: 'Slip — Front (solid) / Rear (dashed)', color: '#0f172a', font: { size: 13 } },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [d0, d1, d2, d3] = chartRef.current.data.datasets
        d0.data = buf.map(d => ({ x: d.t, y: (d.f_alpha ?? 0) * 180 / Math.PI }))
        d1.data = buf.map(d => ({ x: d.t, y: (d.r_alpha ?? 0) * 180 / Math.PI }))
        d2.data = buf.map(d => ({ x: d.t, y: d.f_kappa ?? 0 }))
        d3.data = buf.map(d => ({ x: d.t, y: d.r_kappa ?? 0 }))
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
