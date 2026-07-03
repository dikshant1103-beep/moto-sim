/**
 * Phase2Panel — Phase 2 tire diagnostics
 * Shown when engine label contains "Phase2" or "FullMF".
 *
 * Charts:
 *   1. Contact patch area (front + rear) vs time
 *   2. LuGre sliding fraction (front + rear) vs time — 0=full stick, 1=full slide
 *   3. Lateral force Fy (front + rear) — shows turn-slip and camber thrust contributions
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
             ticks: { color: '#475569', font: { size: 8 } }, grid: { color: '#1e293b' } },
        y: { min: yMin, max: yMax,
             title: { display: true, text: yLabel, color: '#475569', font: { size: 9 } },
             ticks: { color: '#475569', font: { size: 8 } }, grid: { color: '#334155' } },
      },
      plugins: {
        title:  { display: true, text: title, color: '#94a3b8', font: { size: 9 } },
        legend: { labels: { color: '#94a3b8', font: { size: 8 }, boxWidth: 10 } },
      },
    },
  })
}

const MAX_PTS = 600

export default function Phase2Panel({ dataRef, latestRef }) {
  const areaRef   = useRef(null)
  const slideRef  = useRef(null)
  const fyRef     = useRef(null)
  const charts    = useRef({})
  const timerRef  = useRef(null)
  const lastIdx   = useRef(0)

  useEffect(() => {
    const area = makeChart(areaRef.current, 'Contact Patch Area',
      [ds('Front [cm²]', '#38bdf8'), ds('Rear [cm²]', '#4ade80', [4,2])],
      'Area [cm²]', 0, 60)

    const slide = makeChart(slideRef.current, 'LuGre Sliding Fraction',
      [ds('Front', '#f87171'), ds('Rear', '#fb923c', [4,2])],
      'Fraction [0=stick, 1=slide]', 0, 1)

    const fy = makeChart(fyRef.current, 'Lateral Force Fy',
      [ds('Fy_front [N]', '#c084fc'), ds('Fy_rear [N]', '#fbbf24', [4,2])],
      'Fy [N]', null, null)

    charts.current = { area, slide, fy }

    timerRef.current = setInterval(() => {
      const buf = dataRef.current
      if (!buf.length) return

      const newFrames = buf.slice(lastIdx.current)
      lastIdx.current = buf.length

      if (!newFrames.length) return

      const cap = MAX_PTS
      for (const f of newFrames) {
        const t = f.t ?? 0
        area.data.datasets[0].data.push({ x: t, y: f.f_contact_cm2 ?? 0 })
        area.data.datasets[1].data.push({ x: t, y: f.r_contact_cm2 ?? 0 })
        slide.data.datasets[0].data.push({ x: t, y: f.f_sliding_frac ?? 0 })
        slide.data.datasets[1].data.push({ x: t, y: f.r_sliding_frac ?? 0 })
        fy.data.datasets[0].data.push({ x: t, y: f.f_Fy ?? 0 })
        fy.data.datasets[1].data.push({ x: t, y: f.r_Fy ?? 0 })
      }

      for (const ch of Object.values(charts.current)) {
        for (const ds of ch.data.datasets) {
          if (ds.data.length > cap) ds.data.splice(0, ds.data.length - cap)
        }
        ch.update('none')
      }
    }, UPDATE_MS)

    return () => {
      clearInterval(timerRef.current)
      Object.values(charts.current).forEach(c => c.destroy())
    }
  }, [dataRef])

  // Live readout from latest frame
  const latest = latestRef?.current
  const isPhase2 = latest?.engine?.includes('Phase2') || latest?.engine?.includes('FullMF')

  return (
    <div style={{ padding: '6px 8px', height: '100%', display: 'flex', flexDirection: 'column', gap: 4 }}>
      <div style={{ color: '#38bdf8', fontSize: 10, fontWeight: 700, marginBottom: 2 }}>
        Phase 2 Tire — Contact Patch &amp; LuGre Bristle Model
        {latest && (
          <span style={{
            marginLeft: 8, background: isPhase2 ? '#1e3a8a' : '#374151',
            color: '#fff', borderRadius: 3, padding: '1px 6px', fontSize: 9
          }}>
            {latest.engine ?? 'unknown'}
          </span>
        )}
      </div>

      {/* Stat pills */}
      {latest && (
        <div style={{ display: 'flex', gap: 8, flexWrap: 'wrap', marginBottom: 4 }}>
          {[
            { label: 'A_f', val: `${(latest.f_contact_cm2 ?? 0).toFixed(1)} cm²`, color: '#38bdf8' },
            { label: 'A_r', val: `${(latest.r_contact_cm2 ?? 0).toFixed(1)} cm²`, color: '#4ade80' },
            { label: 'slip_f', val: `${((latest.f_sliding_frac ?? 0) * 100).toFixed(0)}%`, color: '#f87171' },
            { label: 'slip_r', val: `${((latest.r_sliding_frac ?? 0) * 100).toFixed(0)}%`, color: '#fb923c' },
            { label: 'Fy_f', val: `${(latest.f_Fy ?? 0).toFixed(0)} N`, color: '#c084fc' },
            { label: 'Fy_r', val: `${(latest.r_Fy ?? 0).toFixed(0)} N`, color: '#fbbf24' },
          ].map(({ label, val, color }) => (
            <div key={label} style={{
              background: '#1e293b', borderRadius: 4, padding: '2px 7px',
              fontSize: 9, border: `1px solid ${color}44`
            }}>
              <span style={{ color: '#64748b' }}>{label} </span>
              <span style={{ color, fontWeight: 700 }}>{val}</span>
            </div>
          ))}
        </div>
      )}

      <div style={{ flex: 1, display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 4, minHeight: 0 }}>
        <div style={{ position: 'relative', minHeight: 80 }}>
          <canvas ref={areaRef} />
        </div>
        <div style={{ position: 'relative', minHeight: 80 }}>
          <canvas ref={slideRef} />
        </div>
        <div style={{ position: 'relative', minHeight: 80 }}>
          <canvas ref={fyRef} />
        </div>
      </div>
    </div>
  )
}
