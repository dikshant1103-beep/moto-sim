/**
 * FrictionCircle — Normalised μ scatter: Fx/Fz vs Fy/Fz for front tyre.
 *
 * The circle radius tracks road_mu_f from live telemetry (M4-road).
 * Falls back to the `mu` prop (= setup.frictionCoefficient) before data arrives.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)
const UPDATE_MS  = 140
const DEFAULT_MU = 0.9
const TIRE_MU    = 0.9   // Pacejka compound friction (TireCoeffs.mu)

function makeUnitCircle(mu) {
  return Array.from({ length: 73 }, (_, i) => {
    const θ = (i / 72) * 2 * Math.PI
    return { x: mu * Math.cos(θ), y: mu * Math.sin(θ) }
  })
}

export default function FrictionCircle({ dataRef, mu }) {
  const canvasRef = useRef(null)
  const chartRef  = useRef(null)
  const timerRef  = useRef(null)
  const muRef     = useRef(mu ?? DEFAULT_MU)

  // Keep muRef in sync with prop changes (e.g. user edits frictionCoefficient)
  useEffect(() => { muRef.current = mu ?? DEFAULT_MU }, [mu])

  useEffect(() => {
    const ctx = canvasRef.current.getContext('2d')
    const initMu = muRef.current
    chartRef.current = new Chart(ctx, {
      type: 'scatter',
      data: {
        datasets: [
          {
            label: 'Operating locus',
            data: [],
            backgroundColor: 'rgba(56,189,248,0.6)',
            pointRadius: 2,
            showLine: true,
            borderColor: 'rgba(56,189,248,0.3)',
            borderWidth: 1,
            tension: 0,
          },
          {
            label: `Friction limit μ = ${initMu.toFixed(2)}`,
            data: makeUnitCircle(initMu),
            borderColor: 'rgba(251,113,133,0.85)',
            borderWidth: 2,
            backgroundColor: 'transparent',
            pointRadius: 0,
            showLine: true,
            tension: 0,
          },
          {
            label: 'Current',
            data: [],
            backgroundColor: '#fbbf24',
            pointRadius: 6,
            showLine: false,
          },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        scales: {
          x: {
            title: { display: true, text: 'μx = Fx/Fz [–]', color: '#475569' },
            ticks: { color: '#475569' },
            grid:  { color: 'rgba(56,189,248,0.08)' },
            min: -1.2, max: 1.2,
          },
          y: {
            title: { display: true, text: 'μy = Fy/Fz [–]', color: '#475569' },
            ticks: { color: '#475569' },
            grid:  { color: 'rgba(56,189,248,0.08)' },
            min: -1.2, max: 1.2,
          },
        },
        plugins: {
          legend: { labels: { color: '#475569', boxWidth: 12 } },
          title:  {
            display: true,
            text: 'Friction Circle (normalised)',
            color: '#0f172a',
            font: { size: 13 },
          },
        },
      },
    })

    function tick() {
      const buf = dataRef.current
      if (buf.length === 0) return

      const [locus, circle, current] = chartRef.current.data.datasets

      // Operating locus (history)
      locus.data = buf.map(d => {
        const fz = d.f_Fz || 1
        return { x: (d.f_Fx ?? 0) / fz, y: (d.f_Fy ?? 0) / fz }
      })

      // Current operating point
      const last = buf[buf.length - 1]
      const lastFz = last.f_Fz || 1
      current.data = [{ x: (last.f_Fx ?? 0) / lastFz, y: (last.f_Fy ?? 0) / lastFz }]

      // Friction limit circle — update radius from road_mu_f telemetry (M4-road)
      // road_mu_f is the road surface mu (0–1); combined limit = TIRE_MU * road_mu_f
      const roadMu     = last.road_mu_f != null ? last.road_mu_f : muRef.current
      const circleR    = TIRE_MU * roadMu
      const circleRFmt = circleR.toFixed(2)
      if (circle.label !== `Friction limit μ = ${circleRFmt}`) {
        circle.data  = makeUnitCircle(circleR)
        circle.label = `Friction limit μ = ${circleRFmt}`
      }

      chartRef.current.update('none')
    }

    tick()
    timerRef.current = setInterval(tick, UPDATE_MS)

    return () => {
      clearInterval(timerRef.current)
      chartRef.current.destroy()
    }
  }, []) // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <canvas ref={canvasRef} />
    </div>
  )
}
