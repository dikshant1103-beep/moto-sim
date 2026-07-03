/**
 * GeometryPanel — Camber angle γ (left axis) and contact patch offset y_cp (right axis) vs time.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)

export default function GeometryPanel({ dataRef }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const rafRef    = useRef(null)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'γ [°]', data: [], yAxisID: 'yGamma',
            borderColor: '#b45309', borderWidth: 1.5, pointRadius: 0, tension: 0 },
          { label: 'y_cp [mm]', data: [], yAxisID: 'yYcp',
            borderColor: '#7e22ce', borderWidth: 1.5, pointRadius: 0, tension: 0 },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: { type: 'linear', title: { display: true, text: 'time [s]', color: '#475569' },
               ticks: { color: '#475569', maxTicksLimit: 8 }, grid: { color: '#e2e8f0' } },
          yGamma: { type: 'linear', position: 'left',
                    title: { display: true, text: 'γ [°]', color: '#b45309' },
                    ticks: { color: '#b45309' }, grid: { color: '#e2e8f0' } },
          yYcp:   { type: 'linear', position: 'right',
                    title: { display: true, text: 'y_cp [mm]', color: '#7e22ce' },
                    ticks: { color: '#7e22ce' }, grid: { drawOnChartArea: false } },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12 } },
          title:  { display: true, text: 'Geometry', color: '#0f172a', font: { size: 13 } },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [ds0, ds1] = chartRef.current.data.datasets
        ds0.data = buf.map(d => ({ x: d.time, y: d.gamma }))
        ds1.data = buf.map(d => ({ x: d.time, y: d.y_cp * 1000 }))   // m → mm
        chartRef.current.update('none')
      }
      rafRef.current = requestAnimationFrame(tick)
    }
    rafRef.current = requestAnimationFrame(tick)

    return () => {
      cancelAnimationFrame(rafRef.current)
      chartRef.current.destroy()
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <canvas ref={canvasRef} />
    </div>
  )
}
