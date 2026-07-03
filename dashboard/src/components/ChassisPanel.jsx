/**
 * ChassisPanel — roll, pitch, yaw (left axis °) and roll_rate, yaw_rate (right axis °/s) vs time.
 * Replaces the old tire-rig GeometryPanel.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 140

const R2D = 180 / Math.PI

export default function ChassisPanel({ dataRef }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const timerRef  = useRef(null)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          { label: 'roll [°]',      data: [], yAxisID: 'yAngle',
            borderColor: '#ef4444', borderWidth: 2, pointRadius: 0, tension: 0 },
          { label: 'pitch [°]',     data: [], yAxisID: 'yAngle',
            borderColor: '#f59e0b', borderWidth: 1.3, pointRadius: 0, tension: 0, borderDash: [4,3] },
          { label: 'yaw [°]',       data: [], yAxisID: 'yAngle',
            borderColor: '#3b82f6', borderWidth: 1.3, pointRadius: 0, tension: 0, borderDash: [2,3] },
          { label: 'roll_rate [°/s]',  data: [], yAxisID: 'yRate',
            borderColor: '#f87171', borderWidth: 1.2, pointRadius: 0, tension: 0 },
          { label: 'yaw_rate [°/s]',   data: [], yAxisID: 'yRate',
            borderColor: '#60a5fa', borderWidth: 1.2, pointRadius: 0, tension: 0 },
        ],
      },
      options: {
        animation: false, responsive: true, maintainAspectRatio: false,
        scales: {
          x: { type: 'linear', title: { display: true, text: 'time [s]', color: '#475569' },
               ticks: { color: '#475569', maxTicksLimit: 8 }, grid: { color: 'rgba(56,189,248,0.08)' } },
          yAngle: { type: 'linear', position: 'left',
                    title: { display: true, text: 'angle [°]', color: '#ef4444' },
                    ticks: { color: '#475569' }, grid: { color: 'rgba(56,189,248,0.08)' } },
          yRate:  { type: 'linear', position: 'right',
                    title: { display: true, text: 'rate [°/s]', color: '#60a5fa' },
                    ticks: { color: '#475569' }, grid: { drawOnChartArea: false } },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12, font: { size: 10 } } },
          title:  { display: true, text: 'Chassis Orientation  (roll in red — G4 limit ±5°)',
                    color: '#0f172a', font: { size: 13 } },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [d0, d1, d2, d3, d4] = chartRef.current.data.datasets
        d0.data = buf.map(d => ({ x: d.t, y: (d.roll      ?? 0) * R2D }))
        d1.data = buf.map(d => ({ x: d.t, y: (d.pitch     ?? 0) * R2D }))
        d2.data = buf.map(d => ({ x: d.t, y: (d.yaw       ?? 0) * R2D }))
        d3.data = buf.map(d => ({ x: d.t, y: (d.roll_rate ?? 0) * R2D }))
        d4.data = buf.map(d => ({ x: d.t, y: (d.yaw_rate  ?? 0) * R2D }))
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
