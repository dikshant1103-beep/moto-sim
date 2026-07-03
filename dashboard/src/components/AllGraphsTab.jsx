/**
 * AllGraphsTab — Full-screen real-time chart grid.
 * 3-column layout, all 11 channels visible simultaneously.
 *
 * Fix applied: borderDash must always be an array, never ''.
 * ctx.setLineDash('') throws DOMException → renderer crash.
 */
import { useEffect, useRef } from 'react'
import { Chart, registerables } from 'chart.js'

Chart.register(...registerables)

/* ── design tokens ───────────────────────────────────────────────────────── */
const GRID = 'rgba(56,189,248,0.08)'
const TICK = '#475569'
const R2D  = 180 / Math.PI
const UPD  = 180   // ms between chart updates
const MAX  = 800   // max data points per series

/* ── chart builders ──────────────────────────────────────────────────────── */
function line(label, color, dash) {
  // dash MUST be an array — never pass '' or undefined
  return {
    label, data: [], borderColor: color, borderWidth: 1.5,
    pointRadius: 0, tension: 0,
    borderDash: Array.isArray(dash) ? dash : [],
  }
}

function line2(label, color, dash) {
  // same as line() but bound to the right y-axis (y2)
  return { ...line(label, color, dash), yAxisID: 'y2' }
}

function axisLeft(label)  { return { title: { display: true, text: label, color: TICK, font: { size: 9 } }, ticks: { color: TICK, font: { size: 8 }, maxTicksLimit: 5 }, grid: { color: GRID } } }
function axisRight(label) { return { position: 'right', display: true, title: { display: true, text: label, color: TICK, font: { size: 9 } }, ticks: { color: TICK, font: { size: 8 }, maxTicksLimit: 5 }, grid: { drawOnChartArea: false, color: GRID } } }
function axisX()          { return { type: 'linear', title: { display: true, text: 'time [s]', color: TICK, font: { size: 9 } }, ticks: { color: TICK, font: { size: 8 }, maxTicksLimit: 6 }, grid: { color: GRID } } }

function buildChart(ctx2d, title, datasets, yLabel, y2Label) {
  return new Chart(ctx2d, {
    type: 'line',
    data: { datasets },
    options: {
      animation: false, responsive: true, maintainAspectRatio: false,
      scales: {
        x:  axisX(),
        y:  axisLeft(yLabel),
        ...(y2Label ? { y2: axisRight(y2Label) } : {}),
      },
      plugins: {
        title:  { display: true, text: title, color: '#94a3b8', font: { size: 10, weight: 'bold' }, padding: { bottom: 4 } },
        legend: { labels: { color: '#64748b', font: { size: 9 }, boxWidth: 10, padding: 8 } },
      },
    },
  })
}

/* ── chart specs ─────────────────────────────────────────────────────────── */
// Each spec: { title, yLabel, y2Label?, datasets[], update(chart, buf) }

const SPECS = [
  {
    title: 'Speed & Longitudinal Accel', yLabel: 'vx [m/s]', y2Label: 'ax [m/s²]',
    datasets: () => [line('vx [m/s]', '#06b6d4', []), line2('ax [m/s²]', '#f97316', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: d.vx ?? 0 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: d.ax ?? 0 }))
    },
  },
  {
    title: 'Chassis Angles', yLabel: '[°]',
    datasets: () => [
      line('roll [°]', '#ef4444', []),
      line('pitch [°]', '#f59e0b', [4, 2]),
      line('yaw_rate [°/s]', '#3b82f6', [2, 2]),
    ],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: (d.roll     ?? 0) * R2D }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: (d.pitch    ?? 0) * R2D }))
      ch.data.datasets[2].data = s.map(d => ({ x: d.t, y: (d.yaw_rate ?? 0) * R2D }))
    },
  },
  {
    title: 'Roll Rate & Yaw Rate', yLabel: '[°/s]',
    datasets: () => [line('roll_rate [°/s]', '#ef4444', []), line('yaw_rate [°/s]', '#3b82f6', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: (d.roll_rate ?? 0) * R2D }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: (d.yaw_rate  ?? 0) * R2D }))
    },
  },
  {
    title: 'Normal Load Transfer', yLabel: 'Fz [N]',
    datasets: () => [line('Fz_front [N]', '#a3e635', []), line('Fz_rear [N]', '#22d3ee', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: d.f_Fz ?? 0 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: d.r_Fz ?? 0 }))
    },
  },
  {
    title: 'Tyre Forces Fx / Fy', yLabel: 'Force [N]',
    datasets: () => [
      line('Fx_front', '#f59e0b', []),
      line('Fx_rear',  '#fb923c', [4, 2]),
      line('Fy_front', '#c084fc', []),
      line('Fy_rear',  '#818cf8', [4, 2]),
    ],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: d.f_Fx ?? 0 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: d.r_Fx ?? 0 }))
      ch.data.datasets[2].data = s.map(d => ({ x: d.t, y: d.f_Fy ?? 0 }))
      ch.data.datasets[3].data = s.map(d => ({ x: d.t, y: d.r_Fy ?? 0 }))
    },
  },
  {
    title: 'Aligning Torque Mz', yLabel: 'Mz [N·m]',
    datasets: () => [line('Mz_front', '#4ade80', []), line('Mz_rear', '#c084fc', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: d.f_Mz ?? 0 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: d.r_Mz ?? 0 }))
    },
  },
  {
    title: 'Slip Angles α & Ratio κ', yLabel: 'α [°]', y2Label: 'κ [-]',
    datasets: () => [
      line ('α_front [°]', '#c084fc', []),
      line ('α_rear [°]',  '#818cf8', [4, 2]),
      line2('κ_front [-]', '#f97316', []),       // [] not '' — key fix
      line2('κ_rear [-]',  '#fbbf24', [4, 2]),
    ],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: (d.f_alpha ?? 0) * R2D }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: (d.r_alpha ?? 0) * R2D }))
      ch.data.datasets[2].data = s.map(d => ({ x: d.t, y: d.f_kappa ?? 0 }))
      ch.data.datasets[3].data = s.map(d => ({ x: d.t, y: d.r_kappa ?? 0 }))
    },
  },
  {
    title: 'Steer Angle & Torque', yLabel: 'δ [°]', y2Label: 'τ [N·m]',
    datasets: () => [line('δ [°]', '#f59e0b', []), line2('τ [N·m]', '#a3e635', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: (d.steer_angle  ?? 0) * R2D }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y:  d.steer_torque ?? 0 }))
    },
  },
  {
    title: 'Suspension Travel', yLabel: 'Travel [mm]',
    datasets: () => [line('Fork [mm]', '#f59e0b', []), line('Rear [mm]', '#c084fc', [4, 2])],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y: (d.fork_travel ?? 0) * 1000 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y: (d.rear_travel ?? 0) * 1000 }))
    },
  },
  {
    title: 'Tyre Thermal — Front', yLabel: 'Temp [°C]', y2Label: 'wear% / kPa',
    datasets: () => [
      line ('T_tread_f [°C]',   '#f43f5e', []),
      line ('T_carcass_f [°C]', '#fb923c', [4, 2]),
      line2('wear_f [%]',       '#fbbf24', [2, 2]),
      line2('pressure_f [kPa]', '#06b6d4', [6, 2]),
    ],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y:  d.f_T_tread   ?? 85 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y:  d.f_T_carcass ?? 80 }))
      ch.data.datasets[2].data = s.map(d => ({ x: d.t, y: (d.f_wear      ?? 0) * 100 }))
      ch.data.datasets[3].data = s.map(d => ({ x: d.t, y:  d.f_pressure  ?? 240 }))
    },
  },
  {
    title: 'Tyre Thermal — Rear', yLabel: 'Temp [°C]', y2Label: 'wear% / kPa',
    datasets: () => [
      line ('T_tread_r [°C]',   '#f43f5e', []),
      line ('T_carcass_r [°C]', '#fb923c', [4, 2]),
      line2('wear_r [%]',       '#fbbf24', [2, 2]),
      line2('pressure_r [kPa]', '#06b6d4', [6, 2]),
    ],
    update(ch, b) {
      const s = b.slice(-MAX)
      ch.data.datasets[0].data = s.map(d => ({ x: d.t, y:  d.r_T_tread   ?? 85 }))
      ch.data.datasets[1].data = s.map(d => ({ x: d.t, y:  d.r_T_carcass ?? 80 }))
      ch.data.datasets[2].data = s.map(d => ({ x: d.t, y: (d.r_wear      ?? 0) * 100 }))
      ch.data.datasets[3].data = s.map(d => ({ x: d.t, y:  d.r_pressure  ?? 240 }))
    },
  },
]

/* ── single chart cell ───────────────────────────────────────────────────── */
function ChartCell({ spec, dataRef }) {
  const ref = useRef(null)

  useEffect(() => {
    const canvas = ref.current
    if (!canvas) return

    let chart
    try {
      const ctx = canvas.getContext('2d')
      chart = buildChart(ctx, spec.title, spec.datasets(), spec.yLabel, spec.y2Label)
    } catch (e) {
      console.error('[AllGraphsTab] chart build error:', spec.title, e)
      return
    }

    const timer = setInterval(() => {
      const buf = dataRef.current
      if (!buf.length || !chart) return
      try {
        spec.update(chart, buf)
        chart.update('none')
      } catch {}
    }, UPD)

    return () => {
      clearInterval(timer)
      try { chart.destroy() } catch {}
    }
  }, [])   // mount once

  return (
    <div style={{
      background: 'var(--bg-panel)',
      border: '1px solid var(--border-dim)',
      borderRadius: 'var(--radius-lg)',
      padding: '6px',
      transition: 'border-color 0.12s',
    }}
      onMouseEnter={e => e.currentTarget.style.borderColor = 'var(--border-mid)'}
      onMouseLeave={e => e.currentTarget.style.borderColor = ''}
    >
      {/* Explicit height required by Chart.js responsive mode — minHeight is NOT enough */}
      <div style={{ position: 'relative', height: '230px', width: '100%' }}>
        <canvas ref={ref} />
      </div>
    </div>
  )
}

/* ── tab root ────────────────────────────────────────────────────────────── */
export default function AllGraphsTab({ dataRef }) {
  return (
    <div style={{
      flex: 1, minHeight: 0,
      overflowY: 'auto',
      display: 'grid',
      gridTemplateColumns: 'repeat(3, minmax(0, 1fr))',
      gap: 8,
      padding: '0 4px 8px',
      alignContent: 'start',
    }}>
      {SPECS.map((spec, i) => (
        <ChartCell key={i} spec={spec} dataRef={dataRef} />
      ))}
    </div>
  )
}
