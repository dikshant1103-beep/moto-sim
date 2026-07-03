/**
 * ValidationPanel.jsx — Virtual Validation tab.
 *
 * Runs the real-physics validation deliverables (conda-python, via Electron IPC) and
 * shows the results — the industrial "screening" face of the platform:
 *   • Road Validation Suite — bumps/cleats/washboard/ISO-8608 → pass/fail (DAF +
 *     bottoming + ISO ride), with a CAE load-case export for the Chassis Workbench.
 *   • Correlation Anchor — suspension ride dynamics vs Foale sag-law + published ranges.
 *   • Whipple/Meijaard Benchmark — reproduces the canonical stability reference.
 */
import { useEffect, useRef, useState, useCallback } from 'react'

const val = typeof window !== 'undefined' ? window.validationAPI : undefined

const RUNS = [
  { id: 'suite',       label: 'Road Validation Suite', desc: 'bumps · cleats · washboard · ISO-8608 → pass/fail + load export (~2–4 min)' },
  { id: 'correlation', label: 'Correlation Anchor',    desc: 'ride freq + damping + Foale sag-law vs published (~15 s)' },
  { id: 'longitudinal',label: 'Longitudinal Benchmarks',desc: 'static load + braking transfer + stoppie vs closed-form (~2 min)' },
  { id: 'whipple',     label: 'Whipple Benchmark',     desc: 'reproduce canonical Meijaard stability eigenvalues (~1 s)' },
]

const vColor = (v) => v === 'pass' ? 'var(--green,#34d399)' : v === 'warn' ? 'var(--amber,#fbbf24)' : 'var(--red,#f87171)'

export default function ValidationPanel() {
  const [running, setRunning] = useState(null)
  const [log, setLog] = useState('')
  const [report, setReport] = useState(null)
  const logRef = useRef(null)

  useEffect(() => {
    if (!val) return
    const offLog = val.onLog(({ text }) => setLog(l => (l + text).slice(-20000)))
    const offDone = val.onDone(({ kind, code, report }) => {
      setRunning(null)
      setLog(l => l + `\n[${kind}] finished (exit ${code})\n`)
      if (report) setReport(report)
    })
    return () => { offLog(); offDone() }
  }, [])

  useEffect(() => { if (logRef.current) logRef.current.scrollTop = logRef.current.scrollHeight }, [log])

  const run = useCallback(async (id) => {
    if (!val) return
    setRunning(id); setLog(''); if (id === 'suite') setReport(null)
    const r = await val.run(id)
    if (!r.ok) { setRunning(null); setLog(`error: ${r.error}`) }
  }, [])

  const card = { background: 'var(--bg-raised,#0e1726)', border: '1px solid var(--border,#1e2d44)', borderRadius: 10, padding: 12 }
  const btn = { fontSize: 12, padding: '6px 12px', borderRadius: 6, border: '1px solid var(--border,#26405f)',
    background: 'var(--bg,#0a1422)', color: 'var(--text,#cbd5e1)', cursor: 'pointer' }

  const summary = report?.summary

  return (
    <div style={{ padding: 16, display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 14, height: '100%', overflow: 'hidden' }}>
      {/* LEFT — controls + report */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: 12, minHeight: 0, overflow: 'auto' }}>
        <div style={card}>
          <h2 style={{ margin: '0 0 4px', fontSize: 16, color: 'var(--text,#e2e8f0)' }}>Virtual Validation</h2>
          <p style={{ margin: '0 0 10px', fontSize: 11, color: 'var(--text-dim,#94a3b8)' }}>
            Real-physics durability/ride <b>screening</b> + credibility correlation. Screening-grade
            (reduces physical iterations), not certification.
          </p>
          {!val && <div style={{ fontSize: 11, color: 'var(--amber,#fbbf24)' }}>
            Needs the Electron app (npm start) to run the physics scripts.</div>}
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
            {RUNS.map(r => (
              <div key={r.id} style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
                <div>
                  <div style={{ fontSize: 12, color: 'var(--text,#e2e8f0)' }}>{r.label}</div>
                  <div style={{ fontSize: 9, color: 'var(--text-dim,#64748b)' }}>{r.desc}</div>
                </div>
                <button onClick={() => run(r.id)} disabled={!val || running}
                  style={{ ...btn, opacity: (!val || running) ? 0.5 : 1,
                    borderColor: running === r.id ? 'var(--amber,#fbbf24)' : 'var(--green,#34d399)',
                    color: running === r.id ? 'var(--amber,#fbbf24)' : 'var(--green,#34d399)' }}>
                  {running === r.id ? 'Running…' : 'Run'}
                </button>
              </div>
            ))}
          </div>
        </div>

        {/* Pass/fail report table (after suite) */}
        {report && (
          <div style={card}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 8 }}>
              <h3 style={{ margin: 0, fontSize: 13, color: 'var(--text,#e2e8f0)' }}>Road Validation Report</h3>
              {summary && <div style={{ fontSize: 11 }}>
                <span style={{ color: vColor('pass') }}>{summary.pass} pass</span> ·{' '}
                <span style={{ color: vColor('warn') }}>{summary.warn} warn</span> ·{' '}
                <span style={{ color: vColor('fail') }}>{summary.fail} fail</span>
              </div>}
            </div>
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: 11 }}>
              <thead><tr style={{ color: 'var(--text-dim,#94a3b8)', textAlign: 'left' }}>
                <th style={{ padding: '3px 4px' }}>Test</th><th>Verdict</th><th>DAF</th><th>Susp%</th><th>ISO aw</th>
              </tr></thead>
              <tbody>
                {report.tests.map((t, i) => {
                  const m = t.metrics || {}
                  const daf = Math.max(m.loads?.DAF_front ?? 0, m.loads?.DAF_rear ?? 0)
                  return (
                    <tr key={i} style={{ borderTop: '1px solid var(--border,#1e2d44)' }}>
                      <td style={{ padding: '3px 4px', color: 'var(--text,#e2e8f0)' }}>{t.test}</td>
                      <td style={{ color: vColor(t.verdict), fontWeight: 700 }}>{t.verdict?.toUpperCase()}</td>
                      <td style={{ fontFamily: 'monospace' }}>{daf.toFixed(2)}</td>
                      <td style={{ fontFamily: 'monospace' }}>{m.suspension?.fork_use_pct ?? '–'}</td>
                      <td style={{ fontFamily: 'monospace' }}>{m.iso2631?.aw_rms ?? '–'}</td>
                    </tr>
                  )
                })}
              </tbody>
            </table>
            <div style={{ fontSize: 9, color: 'var(--text-dim,#64748b)', marginTop: 6 }}>
              DAF + suspension = robust; ISO aw advisory. Loads exported → /tmp/moto_validation/road_loads_loadcase.json (Chassis Workbench schema).
            </div>
          </div>
        )}
      </div>

      {/* RIGHT — live console */}
      <div style={{ ...card, display: 'flex', flexDirection: 'column', minHeight: 0 }}>
        <h3 style={{ margin: '0 0 6px', fontSize: 13, color: 'var(--text,#e2e8f0)' }}>Output</h3>
        <pre ref={logRef} style={{ flex: 1, overflow: 'auto', margin: 0, fontFamily: 'ui-monospace,monospace',
          fontSize: 10.5, lineHeight: 1.45, color: 'var(--text-dim,#a8b6c8)', background: 'var(--bg,#070e18)',
          borderRadius: 6, padding: 10, whiteSpace: 'pre-wrap' }}>
          {log || 'Run a validation to see live output.\n\n• Road Validation Suite → durability/ride screening + pass/fail report.\n• Correlation Anchor → proves the physics matches published motorcycle data.\n• Whipple Benchmark → reproduces the canonical stability reference.'}
        </pre>
      </div>
    </div>
  )
}
