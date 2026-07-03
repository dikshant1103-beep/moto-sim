// Shared Chart.js theme — reads from CSS variables where possible.
// Returns a consistent options block for line charts.

export const GRID   = 'rgba(56,189,248,0.08)'
export const TICK   = '#475569'
export const LEGEND = '#64748b'

export function chartOpts(yLabel, yMin, yMax) {
  return {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: {
        type: 'linear',
        title: { display: true, text: 'time [s]', color: TICK, font: { size: 9 } },
        ticks: { color: TICK, font: { size: 8 }, maxTicksLimit: 6 },
        grid:  { color: GRID },
      },
      y: {
        ...(yMin !== undefined && yMin !== null ? { min: yMin } : {}),
        ...(yMax !== undefined && yMax !== null ? { max: yMax } : {}),
        title: { display: true, text: yLabel, color: TICK, font: { size: 9 } },
        ticks: { color: TICK, font: { size: 8 }, maxTicksLimit: 6 },
        grid:  { color: GRID },
      },
    },
    plugins: {
      legend: { labels: { color: LEGEND, font: { size: 9 }, boxWidth: 10 } },
    },
  }
}

export function ds(label, color, dash = []) {
  return {
    label,
    data: [],
    borderColor: color,
    borderWidth: 1.6,
    pointRadius: 0,
    tension: 0,
    borderDash: dash,
  }
}
