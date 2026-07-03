import TermBadge from './TermBadge.jsx'

const g = 9.81

function clamp(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value))
}

function computeAnalysis(setup) {
  const Rf   = setup.frontWheelDiameter / 2
  const Rr   = setup.rearWheelDiameter / 2
  const L    = Math.max(setup.wheelbase, 0.5)
  const h    = Math.max(setup.centreOfGravityHeight, 0.1)
  const t    = Math.max(setup.trail, 0.01)
  const rake = (setup.rakeDegrees * Math.PI) / 180
  const m    = Math.max(setup.totalMass, 10)
  const ff   = clamp(setup.frontWeightFraction, 0.2, 0.8)
  const v    = Math.max(setup.simulationSpeed, 1)

  // ── Braking ──────────────────────────────────────────────────────────────
  const frictionLimitedDecel = setup.frictionCoefficient * g
  const avgR = (Rf + Rr) / 2
  const brakeLimitedDecel = setup.brakeTorqueMaximum / (m * Math.max(avgR, 0.05))
  const effectiveDecel = Math.max(0.5, Math.min(frictionLimitedDecel, brakeLimitedDecel))
  const stopTime     = v / effectiveDecel
  const stopDistance = (v ** 2) / (2 * effectiveDecel)

  // ── Suspension ride frequencies ───────────────────────────────────────────
  const rideFrequencyFront = (1 / (2 * Math.PI)) *
    Math.sqrt(setup.frontSpringRate / (m * ff))
  const rideFrequencyRear  = (1 / (2 * Math.PI)) *
    Math.sqrt(setup.rearSpringRate  / (m * (1 - ff)))

  // ── Wobble frequency (speed-dependent steering oscillation) ───────────────
  // Natural frequency of steering oscillation driven by front tyre cornering
  // stiffness reacting against trail self-alignment.
  // C_αf ≈ 10 × Fz_front_static  (empirical estimate, N/rad)
  // I_steer ≈ 0.5 kg·m²  (typical CBR-class fork + wheel assembly)
  // ω_wobble = sqrt((C_αf * t + m_front * v² * t / L) / I_steer)
  const C_alphaf    = 10 * m * g * ff         // N/rad
  const m_front     = m * ff
  const I_steer     = 0.5                     // kg·m² (fixed estimate)
  const wobbleFrequency = (1 / (2 * Math.PI)) *
    Math.sqrt((C_alphaf * t + m_front * v * v * t / L) / I_steer)

  // ── Weave frequency (speed-dependent yaw/roll oscillation) ───────────────
  // Coupled roll–yaw mode. Iz ≈ m × L² / 12 (slender rod approximation).
  // ω_weave = (v/L) * sqrt(m × h / Iz) = (v/L) * sqrt(12 × h / L²)
  //         = v * sqrt(12 × h) / L²
  const weaveFrequency = v * Math.sqrt(12 * h) / (2 * Math.PI * L * L)

  // ── Capsize speed (above which capsize eigenvalue becomes stable) ─────────
  // From Sharp (1971) linearised analysis:
  // v_cap = sqrt(g × t × L / (h × ff))
  const capsizeSpeed = Math.sqrt(g * t * L / (h * ff))

  // ── Damping ratios ────────────────────────────────────────────────────────
  const dampingRatioFront = setup.frontDamping /
    (2 * Math.sqrt(setup.frontSpringRate * m * ff))
  const dampingRatioRear  = setup.rearDamping  /
    (2 * Math.sqrt(setup.rearSpringRate  * m * (1 - ff)))

  // ── Disturbance indices ───────────────────────────────────────────────────
  const bumpAcceleration = (v ** 2 / Math.max(setup.cornerRadius, 1)) + (setup.bumpHeight * 120)
  const potholeSeverity  = setup.potholeDepth * v * 18

  // ── M2 dynamic Fz (weight transfer under braking/acceleration) ───────────
  // ΔFz = m × a × h / L  (positive braking a → weight forward)
  const Fz_static_front = m * g * ff
  const Fz_static_rear  = m * g * (1 - ff)
  const decelSteps = [0, 0.25, 0.5, 0.75, 1.0]   // fractions of max decel [g]
  const dynamicFzSeries = decelSteps.map(frac => {
    const a = frac * effectiveDecel
    const dFz = m * a * h / L
    return {
      a_g: frac * effectiveDecel / g,
      Fz_f: Math.max(0, Fz_static_front + dFz),
      Fz_r: Math.max(0, Fz_static_rear  - dFz),
    }
  })

  return {
    effectiveDecel, stopTime, stopDistance,
    rideFrequencyFront, rideFrequencyRear,
    wobbleFrequency, weaveFrequency, capsizeSpeed,
    bumpAcceleration, potholeSeverity,
    dampingRatioFront, dampingRatioRear,
    dynamicFzSeries,
    Fz_static_front, Fz_static_rear,
  }
}

function format(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—'
}

function makeStoppingSeries(setup, analysis) {
  const maxSpeed = Math.max(setup.simulationSpeed * 1.8, 10)
  return Array.from({ length: 9 }, (_, i) => {
    const speed = (maxSpeed / 8) * i
    const distance = (speed ** 2) / (2 * analysis.effectiveDecel)
    return { speed, distance }
  })
}

function makeStabilitySeries(analysis) {
  return [
    ['Wobble',      analysis.wobbleFrequency,     12],
    ['Weave',       analysis.weaveFrequency,        6],
    ['Front Ride',  analysis.rideFrequencyFront,    4],
    ['Rear Ride',   analysis.rideFrequencyRear,     4],
  ]
}

export default function AnalysisWorkspace({ setup }) {
  const analysis       = computeAnalysis(setup)
  const stoppingSeries = makeStoppingSeries(setup, analysis)
  const stabilitySeries = makeStabilitySeries(analysis)
  const maxDistance    = Math.max(...stoppingSeries.map(p => p.distance), 1)

  const maxFz = Math.max(
    ...analysis.dynamicFzSeries.map(p => p.Fz_f),
    ...analysis.dynamicFzSeries.map(p => p.Fz_r),
    1,
  )

  return (
    <div className="workspace-grid">
      <div className="workspace-main">
        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Analytical Metrics</h2>
            <p>Values from corrected linearised motorcycle equations using the current geometry and mass setup.</p>
          </div>

          <div className="metric-grid">
            <div className="metric-card">
              <TermBadge label="Stopping Distance" definition="Distance to stop from reference speed under effective deceleration." />
              <strong>{format(analysis.stopDistance)} m</strong>
            </div>
            <div className="metric-card">
              <TermBadge label="Stopping Time" definition="Time to stop from reference speed under effective deceleration." />
              <strong>{format(analysis.stopTime)} s</strong>
            </div>
            <div className="metric-card">
              <TermBadge label="Effective Deceleration" definition="Lesser of friction-limited and brake-torque-limited deceleration." />
              <strong>{format(analysis.effectiveDecel)} m/s²</strong>
            </div>
            <div className="metric-card">
              <TermBadge
                label="Wobble Frequency"
                definition="Steering oscillation driven by front-tyre cornering stiffness vs. trail. Speed-dependent. Correct M2 formula: ω = sqrt((Cαf·t + mf·v²·t/L) / Iδ)."
              />
              <strong>{format(analysis.wobbleFrequency)} Hz</strong>
            </div>
            <div className="metric-card">
              <TermBadge
                label="Weave Frequency"
                definition="Coupled yaw–roll oscillation. M2 formula: f = v·sqrt(12·h) / (2π·L²). Speed-dependent — rises linearly with v."
              />
              <strong>{format(analysis.weaveFrequency)} Hz</strong>
            </div>
            <div className="metric-card">
              <TermBadge
                label="Capsize Speed"
                definition="Speed above which the capsize (lean-fall) eigenvalue becomes stable. M2 formula: v_cap = sqrt(g·t·L / (h·frontFrac))."
              />
              <strong>{format(analysis.capsizeSpeed)} m/s</strong>
            </div>
            <div className="metric-card">
              <TermBadge label="Front Damping Ratio" definition="Front damping vs. critical damping. Target: 0.2–0.4." />
              <strong>{format(analysis.dampingRatioFront)}</strong>
            </div>
            <div className="metric-card">
              <TermBadge label="Rear Damping Ratio" definition="Rear damping vs. critical damping. Target: 0.2–0.4." />
              <strong>{format(analysis.dampingRatioRear)}</strong>
            </div>
          </div>
        </section>

        {/* M2 Dynamic Fz Preview */}
        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Dynamic Fz — Weight Transfer Preview</h2>
            <p>
              Front and rear normal loads under increasing braking deceleration.
              ΔFz = m·a·h_cg / L shifts load forward under braking.
            </p>
          </div>
          <svg className="analysis-svg" viewBox="0 0 640 220" role="img" aria-label="Dynamic Fz graph">
            <rect x="0" y="0" width="640" height="220" rx="14" fill="rgba(255,255,255,0.55)" />
            {analysis.dynamicFzSeries.map((pt, i) => {
              const x = 60 + (i / (analysis.dynamicFzSeries.length - 1)) * 540
              const yf = 180 - (pt.Fz_f / maxFz) * 140
              const yr = 180 - (pt.Fz_r / maxFz) * 140
              return (
                <g key={i}>
                  <circle cx={x} cy={yf} r="4" fill="#a3e635" />
                  <circle cx={x} cy={yr} r="4" fill="#22d3ee" />
                  <text x={x} y="200" textAnchor="middle" className="svg-axis">{format(pt.a_g, 2)}g</text>
                </g>
              )
            })}
            {analysis.dynamicFzSeries.slice(1).map((pt, i) => {
              const prev = analysis.dynamicFzSeries[i]
              const x1 = 60 + (i / (analysis.dynamicFzSeries.length - 1)) * 540
              const x2 = 60 + ((i + 1) / (analysis.dynamicFzSeries.length - 1)) * 540
              return (
                <g key={i}>
                  <line x1={x1} y1={180 - (prev.Fz_f / maxFz) * 140} x2={x2} y2={180 - (pt.Fz_f / maxFz) * 140}
                        stroke="#a3e635" strokeWidth="2" />
                  <line x1={x1} y1={180 - (prev.Fz_r / maxFz) * 140} x2={x2} y2={180 - (pt.Fz_r / maxFz) * 140}
                        stroke="#22d3ee" strokeWidth="2" strokeDasharray="5 2" />
                </g>
              )
            })}
            <line x1="60" y1="180" x2="600" y2="180" stroke="#5e544b" strokeWidth="1.5" />
            <line x1="60" y1="30"  x2="60"  y2="180" stroke="#5e544b" strokeWidth="1.5" />
            <text x="320" y="16" textAnchor="middle" className="svg-title">Fz Front (lime) / Rear (cyan) vs Deceleration</text>
            <text x="320" y="215" textAnchor="middle" className="svg-axis-label">Deceleration [g]</text>
            <text x="20" y="110" textAnchor="middle" transform="rotate(-90 20 110)" className="svg-axis-label">Load [N]</text>
          </svg>
          <div style={{ display: 'flex', gap: 24, marginTop: 6, fontSize: 12, color: '#475569' }}>
            <span>Static: Fz_f = <b>{format(analysis.Fz_static_front, 0)} N</b></span>
            <span>Static: Fz_r = <b>{format(analysis.Fz_static_rear, 0)} N</b></span>
            <span>At max brake: Fz_f = <b>{format(analysis.dynamicFzSeries[analysis.dynamicFzSeries.length - 1].Fz_f, 0)} N</b></span>
            <span>At max brake: Fz_r = <b>{format(analysis.dynamicFzSeries[analysis.dynamicFzSeries.length - 1].Fz_r, 0)} N</b></span>
          </div>
        </section>

        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Stopping Distance Graph</h2>
            <p>Distance rises quadratically with speed — braking setup matters most at high speed.</p>
          </div>
          <svg className="analysis-svg" viewBox="0 0 640 260" role="img" aria-label="Stopping distance graph">
            <rect x="0" y="0" width="640" height="260" rx="18" fill="rgba(255,255,255,0.55)" />
            {stoppingSeries.map((point, index) => {
              const x = 50 + (index / (stoppingSeries.length - 1)) * 540
              const y = 220 - (point.distance / maxDistance) * 170
              return (
                <g key={index}>
                  <line x1={x} y1="220" x2={x} y2="226" stroke="#8b7d6f" strokeWidth="1" />
                  <text x={x} y="244" textAnchor="middle" className="svg-axis">
                    {format(point.speed, 0)}
                  </text>
                  <circle cx={x} cy={y} r="4.5" fill="#c44b2d" />
                </g>
              )
            })}
            {stoppingSeries.slice(1).map((point, index) => {
              const prev = stoppingSeries[index]
              const x1 = 50 + (index / (stoppingSeries.length - 1)) * 540
              const y1 = 220 - (prev.distance / maxDistance) * 170
              const x2 = 50 + ((index + 1) / (stoppingSeries.length - 1)) * 540
              const y2 = 220 - (point.distance / maxDistance) * 170
              return <line key={index} x1={x1} y1={y1} x2={x2} y2={y2} stroke="#c44b2d" strokeWidth="3" />
            })}
            <line x1="50" y1="220" x2="590" y2="220" stroke="#5e544b" strokeWidth="1.5" />
            <line x1="50" y1="30"  x2="50"  y2="220" stroke="#5e544b" strokeWidth="1.5" />
            <text x="320" y="18" textAnchor="middle" className="svg-title">Stopping Distance Versus Speed</text>
            <text x="320" y="255" textAnchor="middle" className="svg-axis-label">Speed (metres per second)</text>
            <text x="18" y="126" textAnchor="middle" transform="rotate(-90 18 126)" className="svg-axis-label">Distance (metres)</text>
          </svg>
        </section>
      </div>

      <aside className="workspace-side">
        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Stability Frequencies</h2>
            <p>Corrected M2 speed-dependent wobble and weave estimates at the reference speed.</p>
          </div>
          <div className="stability-list">
            {stabilitySeries.map(([label, value, scale]) => (
              <div key={label} className="stability-row">
                <span>{label}</span>
                <div className="stability-bar">
                  <div
                    className="stability-fill"
                    style={{ width: `${clamp((value / scale) * 100, 6, 100)}%` }}
                  />
                </div>
                <strong>{format(value)} Hz</strong>
              </div>
            ))}
          </div>
        </section>

        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Interpretation</h2>
            <p>Rules for judging the setup before running the live simulation.</p>
          </div>
          <ul className="workspace-list">
            <li>Capsize speed below current reference speed means the bike is dynamically stable in capsize at that speed.</li>
            <li>Wobble above 6 Hz and weave below 4 Hz is a well-separated, controllable pairing.</li>
            <li>Damping ratio 0.2–0.4 is a good first suspension target.</li>
            <li>At max braking the front Fz should stay below ~80% of total load to avoid loss of rear grip.</li>
          </ul>
        </section>
      </aside>
    </div>
  )
}
