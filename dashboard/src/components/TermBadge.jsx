export default function TermBadge({ label, definition }) {
  return (
    <span className="term-badge" title={definition}>
      {label}
    </span>
  )
}
