import { useState, useEffect } from 'react'
import TermBadge from './TermBadge.jsx'

const FIELD_GROUPS = [
  {
    title: 'Motorcycle Geometry',
    fields: [
      ['wheelbase', 'Wheelbase', 'Distance between front and rear axle centres.', 'm', 0.01],
      ['frontWheelDiameter', 'Front Wheel Diameter', 'Overall diameter of the front wheel and tyre.', 'm', 0.005],
      ['rearWheelDiameter', 'Rear Wheel Diameter', 'Overall diameter of the rear wheel and tyre.', 'm', 0.005],
      ['frontTyreWidth', 'Front Tyre Width', 'Nominal front tyre section width.', 'm', 0.005],
      ['rearTyreWidth', 'Rear Tyre Width', 'Nominal rear tyre section width.', 'm', 0.005],
      ['rakeDegrees', 'Rake Angle', 'Angle between steering axis and vertical.', 'deg', 0.1],
      ['trail', 'Trail', 'Distance between steering-axis ground intercept and tyre contact patch.', 'm', 0.001],
      ['centreOfGravityHeight', 'Centre Of Gravity Height', 'Height of combined motorcycle and rider centre of gravity above ground.', 'm', 0.005],
    ],
  },
  {
    title: 'Mass And Brake Model',
    fields: [
      ['totalMass', 'Total Mass', 'Combined mass of motorcycle and rider.', 'kg', 1],
      ['frontWeightFraction', 'Front Weight Fraction', 'Static fraction of total load carried by the front wheel.', '', 0.01],
      ['engineTorqueMaximum', 'Maximum Engine Torque', 'Peak drive torque available at the rear wheel.', 'N·m', 1],
      ['brakeTorqueMaximum', 'Maximum Brake Torque', 'Combined brake torque capacity used in the simplified model.', 'N·m', 1],
      ['frontSpringRate', 'Front Spring Rate', 'Front suspension spring stiffness.', 'N/m', 100],
      ['rearSpringRate', 'Rear Spring Rate', 'Rear suspension spring stiffness.', 'N/m', 100],
      ['frontDamping', 'Front Damping Coefficient', 'Front suspension viscous damping.', 'N·s/m', 10],
      ['rearDamping', 'Rear Damping Coefficient', 'Rear suspension viscous damping.', 'N·s/m', 10],
      ['riderMass', 'Rider Mass', 'Mass of the rider body. Already included in Total Mass — used only to split physical rider lean torque from constraint compensation.', 'kg', 1],
      ['riderLeanArm', 'Rider Lean Arm', 'Effective lateral centre-of-mass shift per radian of roll. Controls physical portion of roll restoring torque.', 'm/rad', 0.01],
    ],
  },
  {
    title: 'Track And Disturbance Inputs',
    fields: [
      ['trackLength', 'Track Length', 'Total length of the analysed road segment.', 'm', 1],
      ['cornerRadius', 'Corner Radius', 'Reference corner radius for path analysis.', 'm', 1],
      ['bumpHeight', 'Bump Height', 'Vertical bump amplitude for rough-road scenarios.', 'm', 0.001],
      ['potholeDepth', 'Pothole Depth', 'Depth of pothole events for disturbance analysis.', 'm', 0.001],
      ['frictionCoefficient', 'Friction Coefficient', 'Road surface friction coefficient (mu_dry). Scales all Pacejka tyre forces. 1.0 = dry asphalt, 0.5 = wet.', '', 0.01],
      ['straightLength', 'Straight Length', 'Length of each straight section in the M4-road curved track model.', 'm', 5],
      ['cornerBankAngle', 'Corner Bank Angle', 'Road banking angle on curved corners (M4-road). Tilts road surface inward, correcting effective camber.', 'rad', 0.01],
      ['simulationSpeed', 'Reference Speed', 'Input speed used for analytical calculations and preview animation.', 'm/s', 0.1],
    ],
  },
]

const TRACK_MODES = [
  ['flat',     'Flat Road'],
  ['curved',   'Curved Road (M4)'],
  ['bumps',    'Bumps'],
  ['potholes', 'Potholes'],
  ['mixed',    'Mixed Disturbance'],
]

export default function ParameterWorkspace({ setup, onFieldChange, onApply, appliedAt }) {
  const [showBadge, setShowBadge] = useState(false)

  useEffect(() => {
    if (!appliedAt) return
    setShowBadge(true)
    const id = setTimeout(() => setShowBadge(false), 6000)
    return () => clearTimeout(id)
  }, [appliedAt])

  const appliedLabel = appliedAt
    ? appliedAt.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' })
    : ''

  return (
    <div className="workspace-grid">
      <div className="workspace-main">
        {FIELD_GROUPS.map(group => (
          <section key={group.title} className="workspace-panel">
            <div className="workspace-heading">
              <h2>{group.title}</h2>
              <p>All labels use full terminology. Hover any label badge to read its definition.</p>
            </div>

            <div className="parameter-grid">
              {group.fields.map(([key, label, definition, unit, step]) => (
                <label key={key} className="parameter-card">
                  <TermBadge label={label} definition={definition} />
                  <div className="parameter-input-row">
                    <input
                      type="number"
                      step={step}
                      value={setup[key]}
                      onChange={e => onFieldChange(key, Number(e.target.value))}
                    />
                    <span>{unit || 'ratio'}</span>
                  </div>
                </label>
              ))}
            </div>
          </section>
        ))}

        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Simulation Transfer</h2>
            <p>Send the current motorcycle and track setup to the live simulation backend, then rebuild the model.</p>
          </div>
          <div className="workspace-actions" style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
            <button className="btn btn-apply" onClick={onApply}>Apply Inputs To Simulation</button>
            {showBadge && (
              <span style={{
                display: 'inline-flex', alignItems: 'center', gap: 5,
                background: '#dcfce7', color: '#15803d',
                border: '1px solid #86efac', borderRadius: 6,
                padding: '4px 10px', fontSize: 12, fontWeight: 600,
                animation: 'fadeIn 0.2s ease',
              }}>
                ✓ Applied at {appliedLabel}
              </span>
            )}
          </div>
        </section>
      </div>

      <aside className="workspace-side">
        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Track Mode</h2>
            <p>Choose the disturbance profile that will drive the future animation tab.</p>
          </div>
          <div className="track-mode-list">
            {TRACK_MODES.map(([value, label]) => (
              <button
                key={value}
                className={`mode-pill ${setup.trackMode === value ? 'active' : ''}`}
                onClick={() => onFieldChange('trackMode', value)}
              >
                {label}
              </button>
            ))}
          </div>
        </section>

        <section className="workspace-panel">
          <div className="workspace-heading">
            <h2>Planning Notes</h2>
            <p>The parameter tab is now the first step in the workflow.</p>
          </div>
          <ul className="workspace-list">
            <li>Set motorcycle geometry and brake limits.</li>
            <li>Set the road mode and disturbance size.</li>
            <li>Review analytical metrics in the analysis tab.</li>
            <li>Use the live simulation tab after the setup is reasonable.</li>
          </ul>
        </section>
      </aside>
    </div>
  )
}
