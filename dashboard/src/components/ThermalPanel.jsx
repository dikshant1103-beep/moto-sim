/**
 * ThermalPanel — M5 Thermal / Wear / Pressure live charts.
 *
 * Three side-by-side Chart.js line charts:
 *   1. Tread temperature  [°C]  — front (orange) + rear (cyan)
 *   2. Wear level         [%]   — front (amber)  + rear (indigo)
 *   3. Tyre pressure      [kPa] — front (green)  + rear (rose)
 *
 * Data keys from _flatten() (moto_sim_main.py):
 *   f_T_tread, r_T_tread, f_T_carcass, r_T_carcass,
 *   f_wear,    r_wear,    f_pressure,  r_pressure
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS = 150

function makeChart(ctx, title, datasets, yLabel, yMin, yMax) {
  return new Chart(ctx, {
    type: 'line',
    data: { datasets },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: {
          type: 'linear',
          title: { display: true, text: 'time [s]', color: '#475569', font: { size: 9 } },
          ticks: { color: '#475569', maxTicksLimit: 6, font: { size: 9 } },
          grid: { color: 'rgba(56,189,248,0.08)' },
        },
        y: {
          min: yMin,
          max: yMax,
          title: { display: true, text: yLabel, color: '#475569', font: { size: 9 } },
          ticks: { color: '#475569', font: { size: 9 } },
          grid: { color: 'rgba(56,189,248,0.08)' },
        },
      },
      plugins: {
        legend: {
          labels: { color: '#475569', boxWidth: 10, font: { size: 9 } },
        },
        title: {
          display: true,
          text: title,
          color: '#0f172a',
          font: { size: 11, weight: 'bold' },
        },
      },
    },
  })
}

function ds(label, color, dash = []) {
  return {
    label,
    data: [],
    borderColor: color,
    borderWidth: 1.5,
    pointRadius: 0,
    tension: 0,
    borderDash: dash,
  }
}

export default function ThermalPanel({ dataRef }) {
  const tempCanvasRef = useRef(null)
  const wearCanvasRef = useRef(null)
  const pressCanvasRef = useRef(null)
  const charts = useRef([])
  const timerRef = useRef(null)

  useEffect(() => {
    // 1. Temperature chart
    const tempChart = makeChart(
      tempCanvasRef.current.getContext('2d'),
      'Tread Temperature',
      [
        ds('Front T_tread [°C]', '#f97316'),
        ds('Rear T_tread [°C]',  '#22d3ee', [5, 2]),
        ds('Front T_carcass',    '#fb923c', [2, 3]),
        ds('Rear T_carcass',     '#67e8f9', [2, 3]),
      ],
      'Temperature [°C]',
      0, 200,
    )

    // 2. Wear chart
    const wearChart = makeChart(
      wearCanvasRef.current.getContext('2d'),
      'Tyre Wear',
      [
        ds('Front wear [%]', '#f59e0b'),
        ds('Rear wear [%]',  '#818cf8', [5, 2]),
      ],
      'Wear [%]  (0=new, 100=bald)',
      0, 100,
    )

    // 3. Pressure chart
    const pressChart = makeChart(
      pressCanvasRef.current.getContext('2d'),
      'Tyre Pressure',
      [
        ds('Front [kPa]', '#4ade80'),
        ds('Rear [kPa]',  '#f43f5e', [5, 2]),
      ],
      'Pressure [kPa]',
      180, 360,
    )

    charts.current = [tempChart, wearChart, pressChart]

    function tick() {
      const buf = dataRef.current
      if (!buf || buf.length === 0) return

      // Temperature datasets
      const [d0, d1, d2, d3] = tempChart.data.datasets
      d0.data = buf.map(d => ({ x: d.t, y: d.f_T_tread   ?? 85 }))
      d1.data = buf.map(d => ({ x: d.t, y: d.r_T_tread   ?? 85 }))
      d2.data = buf.map(d => ({ x: d.t, y: d.f_T_carcass ?? 80 }))
      d3.data = buf.map(d => ({ x: d.t, y: d.r_T_carcass ?? 80 }))
      tempChart.update('none')

      // Wear datasets (multiply by 100 for %)
      const [w0, w1] = wearChart.data.datasets
      w0.data = buf.map(d => ({ x: d.t, y: (d.f_wear ?? 0) * 100 }))
      w1.data = buf.map(d => ({ x: d.t, y: (d.r_wear ?? 0) * 100 }))
      wearChart.update('none')

      // Pressure datasets
      const [p0, p1] = pressChart.data.datasets
      p0.data = buf.map(d => ({ x: d.t, y: d.f_pressure ?? 240 }))
      p1.data = buf.map(d => ({ x: d.t, y: d.r_pressure ?? 240 }))
      pressChart.update('none')
    }

    tick()
    timerRef.current = setInterval(tick, UPDATE_MS)

    return () => {
      clearInterval(timerRef.current)
      charts.current.forEach(c => c.destroy())
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      width: '100%',
      height: '100%',
      gap: 0,
    }}>
      {/* Section header */}
      <div style={{
        padding: '4px 10px',
        fontSize: 11,
        fontWeight: 700,
        color: '#f97316',
        letterSpacing: '0.08em',
        borderBottom: '1px solid #e2e8f0',
        background: '#fff7ed',
        flexShrink: 0,
      }}>
        M5 TYRE THERMAL &amp; WEAR MODEL
      </div>

      {/* Three charts in a row */}
      <div style={{
        display: 'grid',
        gridTemplateColumns: '1fr 1fr 1fr',
        gap: 4,
        flex: 1,
        minHeight: 0,
        padding: '4px 4px',
      }}>
        <div style={{ position: 'relative', minHeight: 0 }}>
          <canvas ref={tempCanvasRef} />
        </div>
        <div style={{ position: 'relative', minHeight: 0 }}>
          <canvas ref={wearCanvasRef} />
        </div>
        <div style={{ position: 'relative', minHeight: 0 }}>
          <canvas ref={pressCanvasRef} />
        </div>
      </div>
    </div>
  )
}
