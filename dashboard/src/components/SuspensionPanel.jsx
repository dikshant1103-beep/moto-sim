/**
 * SuspensionPanel — Fork compression and rear shock travel vs time.
 * Positive values = compressed (spring shorter than rest length).
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 140

export default function SuspensionPanel({ dataRef }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const timerRef  = useRef(null)

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          {
            label: 'Fork travel [mm]',
            data: [],
            borderColor: '#f59e0b',
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0,
          },
          {
            label: 'Rear travel [mm]',
            data: [],
            borderColor: '#818cf8',
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0,
            borderDash: [5, 2],
          },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            type: 'linear',
            title: { display: true, text: 'time [s]', color: '#475569' },
            ticks: { color: '#475569', maxTicksLimit: 8 },
            grid: { color: 'rgba(56,189,248,0.08)' },
          },
          y: {
            title: { display: true, text: 'Travel [mm]  (+ = compressed)', color: '#475569' },
            ticks: { color: '#475569' },
            grid: { color: 'rgba(56,189,248,0.08)' },
          },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12, font: { size: 10 } } },
          title: {
            display: true,
            text: 'Suspension Travel — Fork (amber) / Rear (purple)',
            color: '#0f172a',
            font: { size: 13 },
          },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length > 0) {
        const [d0, d1] = chartRef.current.data.datasets
        d0.data = buf.map(d => ({ x: d.t, y: (d.fork_travel ?? 0) * 1000 }))
        d1.data = buf.map(d => ({ x: d.t, y: (d.rear_travel ?? 0) * 1000 }))
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
